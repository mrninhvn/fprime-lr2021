// ======================================================================
// \title  LR2021Beacon.cpp
// \author ninhdh4
// \brief  Beacon TX and TM frame-count ownership for the LR2021Manager
//         component.
//
// A beacon (beaconIn: F Prime packet [U16 descriptor = APID][data]) is
// framed here exactly as SpacePacketFramer + TmFramer would frame it: one
// space packet (the beacon's APID, its own sequence count, the whole F Prime
// packet as data) at the start of a fixed-size TM frame on the F Prime TM
// virtual channel, idle-packet fill, FECF. It is sent between framer frames
// on the BEACON route, outside the comStatus credit flow:
//   beaconIn_handler() - copy the packet (a newer one replaces one not yet
//                        sent), return the buffer, serviceBeacon()
//   serviceBeacon()    - from beaconIn and every run() tick: UART target ->
//                        sent at once; radio target -> becomes the slot's
//                        working frame (beaconFrame) once the slot is free
//   dataIn             - a framer frame for a radio busy with a beacon waits
//                        in the slot's pendingFramer; run() sends it next
//
// Frame counts: the beacon frames share the framer's virtual channel, and the
// ground (YAMCS) treats a gap in the VC frame count as frame loss. So this
// component owns the master / VC frame counts at egress: stampTmFrame()
// re-stamps every TM frame of the configured SCID/VCID in send order (framer
// frames and beacons, radio or UART) and recomputes its FECF.
// ======================================================================

#include "Components/LR2021Manager/LR2021Manager.hpp"

#include <Svc/Ccsds/Utils/CRC16.hpp>

#include <cstring>

namespace LR2021 {

namespace {

// TM primary header (CCSDS 132.0-B): GVCID (2), MC count, VC count, data field status (2)
constexpr FwSizeType TM_HEADER_SIZE = 6;
constexpr FwSizeType TM_MC_COUNT_OFFSET = 2;
constexpr FwSizeType TM_VC_COUNT_OFFSET = 3;
constexpr FwSizeType TM_FECF_SIZE = 2;
constexpr U16 TM_SCID_OFFSET = 4;
constexpr U16 TM_VCID_OFFSET = 1;
// Data field status: all flags 0, segment length id 0b11, first header pointer 0 (as TmFramer)
constexpr U16 TM_DATA_FIELD_STATUS = 0x3 << 11;

// Space packet primary header (CCSDS 133.0-B)
constexpr FwSizeType SPP_HEADER_SIZE = 6;
constexpr U16 SPP_APID_MASK = 0x07FF;
constexpr U16 SPP_SEQ_MASK = 0x3FFF;
constexpr U16 SPP_SEQ_UNSEGMENTED = 0x3 << 14;
constexpr U16 SPP_IDLE_APID = 0x07FF;
constexpr U8 SPP_IDLE_DATA = 0x44;  // TmFramer::IDLE_DATA_PATTERN

// F Prime packet descriptor in front of the beacon data
constexpr FwSizeType DESCRIPTOR_SIZE = 2;

void putU16(U8* at, U16 value) {
    at[0] = static_cast<U8>(value >> 8);
    at[1] = static_cast<U8>(value);
}

}  // namespace

void LR2021Manager ::setTmFraming(U16 scid, U8 vcid, U16 frameSize) {
    FW_ASSERT(frameSize <= TM_FRAME_MAX_SIZE, static_cast<FwAssertArgType>(frameSize));
    FW_ASSERT(frameSize > TM_BEACON_OVERHEAD + DESCRIPTOR_SIZE, static_cast<FwAssertArgType>(frameSize));
    this->m_tmScid = scid;
    this->m_tmVcid = vcid;
    this->m_tmFrameSize = frameSize;
    this->m_tmFraming = true;
}

void LR2021Manager ::stampTmFrame(U8* frame, FwSizeType len) {
    if (!this->m_tmFraming || (frame == nullptr) || (len != this->m_tmFrameSize)) {
        return;
    }
    const U16 frameId = static_cast<U16>((frame[0] << 8) | frame[1]);
    if (((frameId >> 14) != 0) || (((frameId >> TM_SCID_OFFSET) & 0x3FF) != this->m_tmScid) ||
        (((frameId >> TM_VCID_OFFSET) & 0x7) != this->m_tmVcid)) {
        return;  // not a frame of our TM virtual channel (e.g. a relayed frame)
    }
    frame[TM_MC_COUNT_OFFSET] = this->m_mcFrameCount++;  // U8, wraps modulo 256
    frame[TM_VC_COUNT_OFFSET] = this->m_vcFrameCount++;
    const U16 crc = Svc::Ccsds::Utils::CRC16::compute(frame, static_cast<U32>(len - TM_FECF_SIZE));
    putU16(&frame[len - TM_FECF_SIZE], crc);
}

void LR2021Manager ::buildBeaconFrame() {
    U8* f = this->m_beaconFrame;
    const FwSizeType frameSize = this->m_tmFrameSize;
    const FwSizeType packetLen = this->m_beaconPacketLen;
    const U16 apid = static_cast<U16>(((this->m_beaconPacket[0] << 8) | this->m_beaconPacket[1]) & SPP_APID_MASK);

    // Own space packet sequence count per beacon APID
    FwIndexType slot = 0;
    while ((slot < this->m_beaconApidCount) && (this->m_beaconApid[slot] != apid)) {
        slot++;
    }
    if (slot == this->m_beaconApidCount) {
        if (slot == BEACON_APID_SLOTS) {
            slot = BEACON_APID_SLOTS - 1;  // table full: share the last counter
        } else {
            this->m_beaconApid[slot] = apid;
            this->m_beaconSeq[slot] = 0;
            this->m_beaconApidCount++;
        }
    }
    const U16 seq = this->m_beaconSeq[slot];
    this->m_beaconSeq[slot] = static_cast<U16>((seq + 1) & SPP_SEQ_MASK);

    // TM primary header; the frame counts and FECF are set by stampTmFrame()
    putU16(&f[0], static_cast<U16>((this->m_tmScid << TM_SCID_OFFSET) | (this->m_tmVcid << TM_VCID_OFFSET)));
    f[TM_MC_COUNT_OFFSET] = 0;
    f[TM_VC_COUNT_OFFSET] = 0;
    putU16(&f[4], TM_DATA_FIELD_STATUS);

    // Beacon space packet: the whole F Prime packet (descriptor included) as
    // data, as SpacePacketFramer does
    FwSizeType pos = TM_HEADER_SIZE;
    putU16(&f[pos], apid);  // version 0, type TM, no secondary header
    putU16(&f[pos + 2], static_cast<U16>(SPP_SEQ_UNSEGMENTED | seq));
    putU16(&f[pos + 4], static_cast<U16>(packetLen - 1));
    pos += SPP_HEADER_SIZE;
    std::memcpy(&f[pos], this->m_beaconPacket, packetLen);
    pos += packetLen;

    // Rest of the data field: one idle packet (TM standard 4.2.2.5), as TmFramer
    const FwSizeType end = frameSize - TM_FECF_SIZE;
    const FwSizeType idleLen = end - pos;
    putU16(&f[pos], SPP_IDLE_APID);
    putU16(&f[pos + 2], SPP_SEQ_UNSEGMENTED);
    putU16(&f[pos + 4], static_cast<U16>(idleLen - SPP_HEADER_SIZE - 1));
    std::memset(&f[pos + SPP_HEADER_SIZE], SPP_IDLE_DATA, idleLen - SPP_HEADER_SIZE);
    f[end] = 0;
    f[end + 1] = 0;
}

void LR2021Manager ::beaconDropped(FwIndexType target) {
    this->m_beaconDropCount++;
    this->tlmWrite_BeaconDropCount(this->m_beaconDropCount);
    this->log_WARNING_LO_BeaconDropped(static_cast<U8>(target));
}

void LR2021Manager ::beaconIn_handler(FwIndexType portNum, Fw::Buffer& fwBuffer) {
    const FwIndexType target = this->m_txRoute[BEACON_ROUTE];
    const FwSizeType len = fwBuffer.isValid() ? fwBuffer.getSize() : 0;
    const U8* data = fwBuffer.getData();
    bool ok = this->m_tmFraming && (data != nullptr) && (len > DESCRIPTOR_SIZE) &&
              (len <= static_cast<FwSizeType>(this->m_tmFrameSize - TM_BEACON_OVERHEAD));
    if (ok) {
        const U16 apid = static_cast<U16>((data[0] << 8) | data[1]);
        ok = (apid < SPP_IDLE_APID);
    }
    if (ok) {
        if (this->m_beaconPending) {
            this->beaconDropped(target);  // the previous one never got its turn
        }
        std::memcpy(this->m_beaconPacket, data, len);
        this->m_beaconPacketLen = static_cast<U16>(len);
        this->m_beaconPending = true;
    } else {
        this->beaconDropped(target);
    }

    // The packet is copied: hand the buffer straight back
    if (this->isConnected_beaconReturnOut_OutputPort(0)) {
        this->beaconReturnOut_out(0, fwBuffer);
    } else if (fwBuffer.isValid()) {
        this->deallocate_out(0, fwBuffer);
    }

    this->serviceBeacon();
}

void LR2021Manager ::serviceBeacon() {
    if (!this->m_beaconPending || this->m_beaconInFlight) {
        return;
    }
    const FwIndexType idx = this->m_txRoute[BEACON_ROUTE];

    if (idx == DISABLED_RADIO) {
        this->m_beaconPending = false;  // route disabled: discard silently
        return;
    }

    if (idx == UART_RADIO) {
        // Synchronous send to the byte-stream driver, like dataIn's UART path
        this->m_beaconPending = false;
        this->buildBeaconFrame();
        this->stampTmFrame(this->m_beaconFrame, this->m_tmFrameSize);
        Drv::ByteStreamStatus st = Drv::ByteStreamStatus::OTHER_ERROR;
        if (this->isConnected_drvSendOut_OutputPort(0)) {
            Fw::Buffer frame(this->m_beaconFrame, this->m_tmFrameSize);
            st = Drv::ByteStreamStatus::SEND_RETRY;
            for (FwIndexType i = 0; (st == Drv::ByteStreamStatus::SEND_RETRY) && (i < UART_RETRY_LIMIT); i++) {
                st = this->drvSendOut_out(0, frame);
            }
        }
        if (st == Drv::ByteStreamStatus::OP_OK) {
            this->m_beaconTxCount++;
            this->tlmWrite_BeaconTxCount(this->m_beaconTxCount);
        } else {
            this->beaconDropped(idx);
        }
        return;
    }

    if ((idx == SDR_RADIO) || (idx == HOST_RADIO)) {
        // m_beaconFrame is out on sdrOut / hostOut until sdrReturnIn / hostReturnIn hands it back
        const bool host = (idx == HOST_RADIO);
        this->m_beaconPending = false;
        if (host ? !this->isConnected_hostOut_OutputPort(0) : !this->isConnected_sdrOut_OutputPort(0)) {
            this->beaconDropped(idx);
            return;
        }
        this->buildBeaconFrame();
        this->stampTmFrame(this->m_beaconFrame, this->m_tmFrameSize);
        this->m_beaconInFlight = true;
        Fw::Buffer frame(this->m_beaconFrame, this->m_tmFrameSize);
        if (host) {
            this->hostOut_out(0, frame);
        } else {
            this->sdrOut_out(0, frame);
        }
        return;
    }

    // Radio target: wait (keep it pending) until the radio is in a packet mode,
    // not owned by a BER test, and its slot is free. A framer frame held back
    // by the previous beacon goes first.
    RadioSlot& r = this->m_radio[idx];
    if (((r.mode != RadioMode::FSK) && (r.mode != RadioMode::FLRC)) || r.txTesting ||
        (this->m_ber.active && ((idx == this->m_ber.txRadio) || (idx == this->m_ber.rxRadio))) ||
        r.workingBuffer.isValid() || r.txInFlight || r.pendingFramer.isValid()) {
        return;
    }
    this->m_beaconPending = false;
    this->buildBeaconFrame();
    this->stampTmFrame(this->m_beaconFrame, this->m_tmFrameSize);
    r.workingBuffer = Fw::Buffer(this->m_beaconFrame, this->m_tmFrameSize);
    r.workingContext = ComCfg::FrameContext();
    r.beaconFrame = true;
    r.relayFrame = false;
    r.txAttempts = 0;
    this->m_beaconInFlight = true;
    if (!this->txSend(r)) {
        this->txDrop(r);  // txComplete() clears the beacon state and counts the drop
    }
}

}  // namespace LR2021
