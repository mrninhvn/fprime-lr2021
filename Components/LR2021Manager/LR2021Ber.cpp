// ======================================================================
// \title  LR2021Ber.cpp
// \author ninhdh4
// \brief  Bit error rate (BER) test for the LR2021Manager component.
//
// One radio module transmits a fixed PRBS9 pattern; a second module
// receives it and the software counts payload bit errors against the known
// pattern. The over-the-air path is raw (no CCSDS RS / CLTU / randomization,
// CRC off) so the measurement is the true channel BER. The whole thing is a
// non-blocking state machine driven by run(): berDrive() paces the TX and
// finalizes; berRxIntercept() is called from the FSK / FLRC RX_DONE service
// to tally each received packet.
// ======================================================================

#include "Components/LR2021Manager/LR2021Manager.hpp"
#include "Components/LR2021Manager/LR2021Cfg.hpp"

extern "C" {
#include "lr20xx_radio_fifo.h"
}

namespace LR2021 {

void LR2021Manager ::berFillPattern(U8* buf, U16 len) {
    // PRBS9 (x^9 + x^5 + 1), all-ones seed: a maximal-length sequence with
    // good bit transitions (avoids the long constant runs that hurt clock
    // recovery). Deterministic, so the receiver regenerates the same bytes.
    U16 lfsr = 0x1FF;
    for (U16 i = 0; i < len; i++) {
        U8 b = 0;
        for (U8 k = 0; k < 8; k++) {
            const U8 bit = static_cast<U8>(((lfsr >> 8) ^ (lfsr >> 4)) & 0x1u);
            lfsr = static_cast<U16>(((lfsr << 1) | bit) & 0x1FFu);
            b = static_cast<U8>((b << 1) | bit);
        }
        buf[i] = b;
    }
}

bool LR2021Manager ::berStart(FwIndexType txRadio,
                              FwIndexType rxRadio,
                              RadioMode mode,
                              U32 freq_hz,
                              I8 power_dbm,
                              U32 num_packets,
                              U16 payload_len,
                              U32 interval_ms) {
    if ((txRadio == rxRadio) || (txRadio >= NUM_RADIOS) || (rxRadio >= NUM_RADIOS) ||
        (payload_len == 0) || (payload_len > BER_MAX_PAYLOAD)) {
        return false;
    }
    if ((mode != RadioMode::FSK) && (mode != RadioMode::FLRC)) {
        return false;
    }

    // Bring both radios up in the requested mode / band. setMode() leaves each
    // in normal continuous RX (role-based for FSK, raw for FLRC).
    if (!this->setMode(txRadio, mode, freq_hz, power_dbm)) {
        return false;
    }
    if (!this->setMode(rxRadio, mode, freq_hz, power_dbm)) {
        return false;
    }

    // FLRC RX is already a raw byte pipe; the FSK RX must be switched off the
    // CCSDS role channel onto the dedicated raw BER syncword / fixed length.
    if (mode == RadioMode::FSK) {
        if (!this->fskBerRx(this->m_radio[rxRadio], payload_len)) {
            return false;
        }
    }

    this->berFillPattern(this->m_ber.pattern, payload_len);

    this->m_ber.active = true;
    this->m_ber.phase = BerPhase::SENDING;
    this->m_ber.txRadio = txRadio;
    this->m_ber.rxRadio = rxRadio;
    this->m_ber.mode = mode;
    this->m_ber.freqHz = freq_hz;
    this->m_ber.powerDbm = power_dbm;
    this->m_ber.totalPackets = num_packets;
    this->m_ber.payloadLen = payload_len;
    this->m_ber.intervalMs = interval_ms;
    this->m_ber.sentCount = 0;
    this->m_ber.recvCount = 0;
    this->m_ber.bitErrors = 0;
    this->m_ber.totalBits = 0;
    this->m_ber.nextTxTime = this->getTime();  // first packet may go immediately
    return true;
}

bool LR2021Manager ::berTxOne(RadioSlot& r) {
    switch (this->m_ber.mode) {
        case RadioMode::FLRC:
            // FLRC TX is already raw (payload sent verbatim).
            return this->flrcTx(r, this->m_ber.pattern, this->m_ber.payloadLen);
        case RadioMode::FSK:
            return this->fskBerTx(r, this->m_ber.pattern, this->m_ber.payloadLen);
        default:
            return false;
    }
}

void LR2021Manager ::berCompare(const U8* recv, U16 len) {
    const U16 n = this->m_ber.payloadLen;
    const U16 cmp = (len < n) ? len : n;
    U32 errs = 0;
    for (U16 i = 0; i < cmp; i++) {
        errs += static_cast<U32>(__builtin_popcount(static_cast<unsigned>(recv[i] ^ this->m_ber.pattern[i])));
    }
    // A short packet (bits never delivered) counts its missing bytes as full
    // errors; a long packet's extra bytes are ignored (not part of the pattern).
    if (len < n) {
        errs += static_cast<U32>(n - len) * 8u;
    }
    this->m_ber.bitErrors += errs;
    this->m_ber.totalBits += static_cast<U32>(n) * 8u;
}

bool LR2021Manager ::berRxIntercept(RadioSlot& r, U16 pkt_len) {
    if (!this->m_ber.active || (r.idx != this->m_ber.rxRadio)) {
        return false;
    }
    if (pkt_len > BER_MAX_PAYLOAD) {
        pkt_len = BER_MAX_PAYLOAD;
    }

    U8 payload[BER_MAX_PAYLOAD] = {0};
    if ((pkt_len > 0) && (lr20xx_radio_fifo_read_rx(&r, payload, pkt_len) == LR20XX_STATUS_OK)) {
        this->berCompare(payload, pkt_len);
    }
    this->m_ber.recvCount++;

    // Re-arm RX on the BER channel for the next packet (FLRC RX is raw already).
    if (this->m_ber.mode == RadioMode::FSK) {
        (void)this->fskBerRx(r, this->m_ber.payloadLen);
    } else {
        (void)this->flrcRx(r, 0);
    }
    return true;
}

void LR2021Manager ::berDrive() {
    if (!this->m_ber.active) {
        return;
    }
    const Fw::Time now = this->getTime();

    if (this->m_ber.phase == BerPhase::SENDING) {
        if (this->m_ber.sentCount >= this->m_ber.totalPackets) {
            // Everything sent: give the last packets time to arrive, then tally.
            this->m_ber.phase = BerPhase::DRAINING;
            Fw::Time end = now;
            end.add(BER_DRAIN_MS / 1000, (BER_DRAIN_MS % 1000) * 1000);
            this->m_ber.drainEnd = end;
            return;
        }
        RadioSlot& tx = this->m_radio[this->m_ber.txRadio];
        // Pace: one packet in flight at a time, no sooner than the interval.
        if (!tx.txInFlight && (now >= this->m_ber.nextTxTime)) {
            if (this->berTxOne(tx)) {
                this->m_ber.sentCount++;
                Fw::Time next = now;
                next.add(this->m_ber.intervalMs / 1000, (this->m_ber.intervalMs % 1000) * 1000);
                this->m_ber.nextTxTime = next;
            }
            // On a TX failure leave the counters untouched and retry next tick.
        }
    } else {  // DRAINING
        if (now >= this->m_ber.drainEnd) {
            this->berFinish();
        }
    }
}

void LR2021Manager ::berFinish() {
    const U32 sent = this->m_ber.sentCount;
    const U32 recv = this->m_ber.recvCount;
    const U32 lost = (sent > recv) ? (sent - recv) : 0;
    const U32 bitErrors = this->m_ber.bitErrors;
    const U32 totalBits = this->m_ber.totalBits;
    U32 berPpm = 0;
    if (totalBits > 0) {
        berPpm = static_cast<U32>((static_cast<U64>(bitErrors) * 1000000ULL) / totalBits);
    }

    this->m_ber.active = false;

    this->tlmWrite_BerBitErrors(bitErrors);
    this->tlmWrite_BerBitsTotal(totalBits);
    this->tlmWrite_BerRatePpm(berPpm);
    this->tlmWrite_BerPacketsRecv(recv);
    this->tlmWrite_BerPacketsLost(lost);
    this->log_ACTIVITY_HI_BerTestDone(static_cast<U8>(this->m_ber.txRadio),
                                      static_cast<U8>(this->m_ber.rxRadio), sent, recv, lost,
                                      bitErrors, totalBits, berPpm);

    // Restore both radios to normal continuous RX in their mode (the RX radio
    // was parked on the raw BER channel; the TX radio is already back in RX
    // after its last TX_DONE, but setMode gives a clean, known state).
    (void)this->setMode(this->m_ber.rxRadio, this->m_ber.mode, this->m_ber.freqHz, this->m_ber.powerDbm);
    (void)this->setMode(this->m_ber.txRadio, this->m_ber.mode, this->m_ber.freqHz, this->m_ber.powerDbm);
}

}  // namespace LR2021
