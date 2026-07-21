// ======================================================================
// \title  LR2021Fsk.cpp
// \author ninhdh4
// \brief  FSK / GFSK operations for the LR2021Manager component, built on
//         the lr20xx_driver C API. Mirrors the FLRC flow (LR2021Flrc.cpp).
//
// Flow:
//   fskInit()    - packet type, RF freq, PA / RX path, modulation and
//                  packet params, syncword
//   fskTx()      - update payload length, fill TX FIFO, start TX
//   fskRx()      - clear RX FIFO, start RX (timeout or continuous)
//   fskService() - poll IRQ status from the rate group; handle TX_DONE /
//                  RX_DONE / error IRQs
// ======================================================================

#include "Components/LR2021Manager/LR2021Manager.hpp"
#include "Components/LR2021Manager/LR2021Cfg.hpp"
#include "Components/LR2021Manager/LR2021Rs.hpp"

#include <cstring>

extern "C" {
#include "lr20xx_system.h"
#include "lr20xx_radio_common.h"
#include "lr20xx_radio_fifo.h"
#include "lr20xx_radio_fsk.h"
}

namespace LR2021 {

namespace {

// Default FSK configuration. Adjust in LR2021Cfg.hpp (or extend FSK_INIT).
constexpr lr20xx_radio_fsk_mod_params_t FSK_MOD_PARAMS = {
    .bitrate_unit = LR20XX_RADIO_FSK_MOD_PARAMS_BR_IN_BPS,
    .bitrate      = FSK_BITRATE_BPS,
    .pulse_shape  = FSK_PULSE_SHAPE,
    .bw           = FSK_RX_BW,
    .fdev_in_hz   = FSK_FDEV_HZ,
};

// True when packets carry no length header (fixed-length CCSDS channel).
constexpr bool FSK_IMPLICIT_LEN = (FSK_HEADER_MODE == LR20XX_RADIO_FSK_HEADER_IMPLICIT);

// TM channel packet length on the air: the fixed frame plus RS parity when
// coding is enabled, otherwise the maximum accepted variable length.
constexpr uint16_t FSK_TM_PKT_LEN =
    FSK_IMPLICIT_LEN ? (FSK_FIXED_PAYLOAD_LEN + (FSK_TM_RS ? Rs::RS_PARITY : 0))
                     : LR2021Manager::FSK_MAX_PAYLOAD;
// Maximum TM frame bytes accepted from the framer per packet.
constexpr uint16_t FSK_TM_DATA_CAP = FSK_TM_RS ? FSK_FIXED_PAYLOAD_LEN : FSK_TM_PKT_LEN;
static_assert(FSK_TM_PKT_LEN <= LR2021Manager::FSK_MAX_PAYLOAD, "TM packet length out of range");
static_assert(!FSK_TM_RS || FSK_IMPLICIT_LEN, "RS coding requires the implicit header mode");
static_assert(!FSK_TM_RS || (FSK_FIXED_PAYLOAD_LEN == Rs::RS_DATA),
              "With RS coding the TM frame must be the RS(255,223) data field (223 bytes)");

// TC channel: maximum TC frame bytes carried by one CLTU.
constexpr uint16_t FSK_CLTU_DATA_CAP = FSK_CLTU_MAX_BLOCKS * CLTU_BLOCK_DATA;
static_assert(FSK_CLTU_PKT_LEN <= LR2021Manager::FSK_MAX_PAYLOAD, "CLTU packet length out of range");

// ----------------------------------------------------------------------
// CCSDS pseudo-randomizer (131.0-B / 231.0-B)
// ----------------------------------------------------------------------

// XOR the data with the standard pseudo-random sequence: LFSR
// x^8 + x^7 + x^5 + x^3 + 1, seeded all-ones at each frame. Verified
// against the published sequence start FF 48 0E C0 9A 0D 70 BC.
// Self-inverse: the same call randomizes and derandomizes.
void ccsdsRandomize(U8* data, U16 len) {
    U8 s = 0xFF;
    for (U16 i = 0; i < len; i++) {
        U8 rnd = 0;
        for (U8 b = 0; b < 8; b++) {
            rnd = static_cast<U8>((rnd << 1) | (s & 1));
            const U8 fb = static_cast<U8>((s ^ (s >> 3) ^ (s >> 5) ^ (s >> 7)) & 1);
            s = static_cast<U8>((s >> 1) | (fb << 7));
        }
        data[i] ^= rnd;
    }
}

// ----------------------------------------------------------------------
// CLTU / BCH(63,56) codec (CCSDS 231.0-B)
// ----------------------------------------------------------------------

// Compute the parity octet of one codeblock: the 7 data bytes are divided
// by g(x) = x^7 + x^6 + x^2 + 1; the 7 remainder bits are complemented and
// placed in bits 7..1, with the filler bit '0' in bit 0.
U8 bchParityOctet(const U8* block) {
    U8 sr = 0;
    for (U16 i = 0; i < CLTU_BLOCK_DATA; i++) {
        for (I8 bit = 7; bit >= 0; bit--) {
            const U8 fb = ((sr >> 6) ^ (block[i] >> bit)) & 1;
            sr = static_cast<U8>((sr << 1) & 0x7F);
            if (fb) {
                sr ^= 0x45;  // x^6 + x^2 + 1
            }
        }
    }
    return static_cast<U8>((~sr << 1) & 0xFE);
}

// Encode a TC frame into the fixed-length TC packet: BCH codeblocks
// (last one filled with CLTU_FILL_BYTE), tail sequence, then idle padding
// up to FSK_CLTU_PKT_LEN. The CLTU start sequence 0xEB90 is not part of
// the payload: it is the radio syncword. Returns 0 when the frame is too
// long, FSK_CLTU_PKT_LEN otherwise.
U16 cltuEncode(const U8* data, U16 len, U8* out) {
    const U16 nblocks = static_cast<U16>((len + CLTU_BLOCK_DATA - 1) / CLTU_BLOCK_DATA);
    if (nblocks > FSK_CLTU_MAX_BLOCKS) {
        return 0;
    }
    const U16 stream_len = static_cast<U16>(nblocks * CLTU_BLOCK_DATA);

    // Assemble the data octet stream (frame + fill), optionally randomized
    // continuously across codeblocks (parity octets are not randomized).
    U8 stream[FSK_CLTU_DATA_CAP];
    std::memcpy(stream, data, len);
    std::memset(&stream[len], CLTU_FILL_BYTE, static_cast<size_t>(stream_len - len));
    if (FSK_TC_RANDOMIZE) {
        ccsdsRandomize(stream, stream_len);
    }

    U16 off = 0;
    // Sacrificial idle pad: the radio corrupts the first byte after the
    // syncword, so codeblock 0 must not start there.
    for (; off < CLTU_LEAD_PAD; off++) {
        out[off] = CLTU_FILL_BYTE;
    }
    for (U16 b = 0; b < nblocks; b++) {
        std::memcpy(&out[off], &stream[b * CLTU_BLOCK_DATA], CLTU_BLOCK_DATA);
        out[off + CLTU_BLOCK_DATA] = bchParityOctet(&out[off]);
        off += CLTU_BLOCK_SIZE;
    }
    std::memcpy(&out[off], CLTU_TAIL_SEQUENCE, CLTU_BLOCK_SIZE);
    off += CLTU_BLOCK_SIZE;
    // Idle padding: the receiver captures a full fixed-length packet.
    std::memset(&out[off], CLTU_FILL_BYTE, static_cast<size_t>(FSK_CLTU_PKT_LEN - off));
    return FSK_CLTU_PKT_LEN;
}

// Decode the codeblocks of a received TC packet, stopping at the tail
// sequence or at the first parity failure (TED codeblock rejection).
// Returns the number of data bytes recovered (TC frame + fill).
U16 cltuDecode(const U8* in, U16 in_len, U8* out, U16 out_cap) {
    // Hunt for the first codeblock: the leading byte(s) after the syncword
    // are unreliable (radio corruption) and the sender prepends an idle pad,
    // so skip forward until a block with valid parity (or the tail) appears.
    U16 off = 0;
    while ((off < CLTU_MAX_LEAD_SKIP) && ((off + CLTU_BLOCK_SIZE) <= in_len)) {
        const U8* cand = &in[off];
        if ((std::memcmp(cand, CLTU_TAIL_SEQUENCE, CLTU_BLOCK_SIZE) == 0) ||
            (bchParityOctet(cand) == cand[CLTU_BLOCK_DATA])) {
            break;
        }
        off++;
    }

    U16 out_len = 0;
    while ((off + CLTU_BLOCK_SIZE) <= in_len) {
        const U8* block = &in[off];
        if (std::memcmp(block, CLTU_TAIL_SEQUENCE, CLTU_BLOCK_SIZE) == 0) {
            break;
        }
        if (bchParityOctet(block) != block[CLTU_BLOCK_DATA]) {
            break;
        }
        if ((out_len + CLTU_BLOCK_DATA) > out_cap) {
            break;
        }
        std::memcpy(&out[out_len], block, CLTU_BLOCK_DATA);
        out_len += CLTU_BLOCK_DATA;
        off += CLTU_BLOCK_SIZE;
    }
    if (FSK_TC_RANDOMIZE && (out_len > 0)) {
        ccsdsRandomize(out, out_len);
    }
    return out_len;
}

// Write the syncword of the requested channel (the chip has one register,
// shared by TX and RX, so it is set before every operation). The TC channel
// is asymmetric: TX prepends one idle octet inside the syncword to dodge
// the LR2021 TX first-byte corruption; RX matches the bare 0xEB90 so a real
// ground station is received as-is.
lr20xx_status_t fskApplySyncword(LR2021Manager* mgr, bool tc_channel, bool is_tx) {
    if (!tc_channel) {
        return lr20xx_radio_fsk_set_syncword(mgr, FSK_TM_SYNCWORD, FSK_TM_SYNCWORD_BITS,
                                             LR20XX_RADIO_FSK_SYNCWORD_MSBF);
    }
    return is_tx ? lr20xx_radio_fsk_set_syncword(mgr, FSK_TC_TX_SYNCWORD, FSK_TC_TX_SYNCWORD_BITS,
                                                 LR20XX_RADIO_FSK_SYNCWORD_MSBF)
                 : lr20xx_radio_fsk_set_syncword(mgr, FSK_TC_RX_SYNCWORD, FSK_TC_RX_SYNCWORD_BITS,
                                                 LR20XX_RADIO_FSK_SYNCWORD_MSBF);
}

// Build the packet params for a given payload length. With a variable-length
// header mode the length acts as the TX payload length / RX maximum length;
// with an implicit header it is the exact packet length.
// long_preamble: required on the TC receive channel — without it the chip
// cannot receive packets whose preamble exceeds ~2000 bits, and the PLOP
// CMM-2 acquisition sequence of a real ground station is typically much
// longer (seconds of idle). Minimum preamble with it enabled is detector
// length + 8 bits + 10 us, well under our 32-bit TX preamble.
lr20xx_radio_fsk_pkt_params_t fskPktParams(uint16_t pld_len, bool long_preamble) {
    lr20xx_radio_fsk_pkt_params_t params = {};
    params.pbl_length_in_bit     = FSK_PREAMBLE_BITS;
    params.preamble_detector     = FSK_PREAMBLE_DETECTOR;
    params.long_preamble_enabled = long_preamble;
    params.address_filtering     = LR20XX_RADIO_FSK_ADDRESS_FILTERING_DISABLED;
    params.header_mode           = FSK_HEADER_MODE;
    params.payload_length_unit   = LR20XX_RADIO_FSK_PAYLOAD_LENGTH_IN_BYTE;
    params.payload_length        = pld_len;
    params.crc                   = FSK_CRC;
    params.whitening             = FSK_WHITENING;
    return params;
}
}  // namespace

bool LR2021Manager ::fskTune(U32 freq_hz) {
    // Skip the SPI write when the chip is already on this frequency: with a
    // single-frequency link (TX freq == RX freq) the radio is programmed once
    // in fskInit and never retunes, so the TX<->RX turnaround has zero extra
    // cost. A split TX/RX plan retunes only when crossing between channels.
    if (freq_hz == this->m_progFreqHz) {
        return true;
    }
    lr20xx_status_t status = lr20xx_radio_common_set_rf_freq(this, freq_hz);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_rf_freq failed (%d)", status);
        return false;
    }
    this->m_progFreqHz = freq_hz;
    return true;
}

bool LR2021Manager ::fskInit(U32 freq_hz, I8 power_dbm) {
    lr20xx_status_t status;

    status = lr20xx_system_set_standby_mode(this, LR20XX_SYSTEM_STANDBY_MODE_RC);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_standby failed (%d)", status);
        return false;
    }

    status = lr20xx_radio_common_set_pkt_type(this, LR20XX_RADIO_COMMON_PKT_TYPE_FSK);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_pkt_type failed (%d)", status);
        return false;
    }

    // Base frequency for this mode; RX rests here and TX may retune to a
    // separate channel (see setRxFreq / setTxFreq / fskTune). Program the RX
    // (rest) frequency now so the radio comes up on the right channel.
    this->m_freqHz = freq_hz;
    const U32 rx_freq = this->m_rxFreqHz ? this->m_rxFreqHz : this->m_freqHz;
    status = lr20xx_radio_common_set_rf_freq(this, rx_freq);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_rf_freq failed (%d)", status);
        return false;
    }
    this->m_progFreqHz = rx_freq;

    // Select the RX path and PA matching the requested band:
    // sub-GHz -> LF path/PA, 2.4 GHz -> HF path/PA.
    const bool is_hf = (freq_hz >= 1000000000U);
    status = lr20xx_radio_common_set_rx_path(
        this, is_hf ? LR20XX_RADIO_COMMON_RX_PATH_HF : LR20XX_RADIO_COMMON_RX_PATH_LF,
        LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_NONE);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_rx_path failed (%d)", status);
        return false;
    }

    lr20xx_radio_common_pa_cfg_t pa_cfg = {};
    pa_cfg.pa_sel = is_hf ? LR20XX_RADIO_COMMON_PA_SEL_HF : LR20XX_RADIO_COMMON_PA_SEL_LF;
    pa_cfg.pa_lf_mode = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM;
    pa_cfg.pa_lf_duty_cycle = 6;  // datasheet default duty/slice values
    pa_cfg.pa_lf_slices = 7;
    pa_cfg.pa_hf_duty_cycle = 16;
    status = lr20xx_radio_common_set_pa_cfg(this, &pa_cfg);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_pa_cfg failed (%d)", status);
        return false;
    }

    // set_tx_params takes 0.5 dBm steps.
    status = lr20xx_radio_common_set_tx_params(this, static_cast<int8_t>(power_dbm * 2),
                                               LR20XX_RADIO_COMMON_RAMP_96_US);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_tx_params failed (%d)", status);
        return false;
    }

    status = lr20xx_radio_fsk_set_modulation_params(this, &FSK_MOD_PARAMS);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_modulation_params failed (%d)", status);
        return false;
    }

    // Configure for the receive channel of this role (the resting state);
    // fskTx() / fskRx() re-apply params and syncword per operation.
    const bool rx_is_tc = (this->m_ccsdsRole == CcsdsRole::SPACECRAFT);
    const lr20xx_radio_fsk_pkt_params_t pkt_params =
        fskPktParams(rx_is_tc ? FSK_CLTU_PKT_LEN : FSK_TM_PKT_LEN, rx_is_tc);
    status = lr20xx_radio_fsk_set_packet_params(this, &pkt_params);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_packet_params failed (%d)", status);
        return false;
    }

    status = fskApplySyncword(this, rx_is_tc, false);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_syncword failed (%d)", status);
        return false;
    }

    // Clear any stale IRQs before operations start.
    lr20xx_system_irq_mask_t irq = 0;
    (void)lr20xx_system_get_and_clear_irq_status(this, &irq);

    DEBUG("FSK init OK: %u Hz, %d dBm", static_cast<unsigned>(freq_hz), power_dbm);
    this->m_mode = RadioMode::FSK;
    this->m_rxContinuous = false;
    this->m_txInFlight = false;

    if (this->isConnected_ready_OutputPort(0)) {
        this->ready_out(0);
    }
    return true;
}

bool LR2021Manager ::fskTx(const U8* data, U16 len) {
    // SPACECRAFT transmits TM frames; GROUND transmits TC frames as CLTUs.
    const bool tx_is_tc = (this->m_ccsdsRole == CcsdsRole::GROUND);
    const U16 max_data = tx_is_tc ? FSK_CLTU_DATA_CAP : FSK_TM_DATA_CAP;
    if ((this->m_mode != RadioMode::FSK) || (data == nullptr) || (len == 0) ||
        (len > max_data)) {
        return false;
    }

    // Refuse while a TX is in flight (cleared by TX_DONE / error in
    // fskService). Lets callers retry from a periodic loop safely.
    if (this->m_txInFlight) {
        return false;
    }

    U8 payload[FSK_MAX_PAYLOAD] = {0};
    U16 tx_len;
    if (tx_is_tc) {
        // TC uplink: BCH codeblocks + tail sequence + idle padding.
        tx_len = cltuEncode(data, len, payload);
        if (tx_len == 0) {
            return false;
        }
    } else {
        // TM downlink: the frame fills the RS data field exactly (zero-pad
        // if shorter); RS parity is appended, then everything after the
        // ASM is pseudo-randomized per CCSDS 131.0-B.
        std::memcpy(payload, data, len);
        if (FSK_TM_RS) {
            Rs::encode(payload, &payload[Rs::RS_DATA]);
        }
        tx_len = FSK_IMPLICIT_LEN ? FSK_TM_PKT_LEN : len;
        if (FSK_TM_RANDOMIZE) {
            ccsdsRandomize(payload, tx_len);
        }
    }

    lr20xx_status_t status = fskApplySyncword(this, tx_is_tc, true);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("TX set_syncword failed (%d)", status);
        return false;
    }

    const lr20xx_radio_fsk_pkt_params_t pkt_params = fskPktParams(tx_len, false);
    status = lr20xx_radio_fsk_set_packet_params(this, &pkt_params);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("TX set_packet_params failed (%d)", status);
        return false;
    }

    status = lr20xx_radio_fifo_clear_tx(this);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("fifo_clear_tx failed (%d)", status);
        return false;
    }

    status = lr20xx_radio_fifo_write_tx(this, payload, tx_len);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("fifo_write_tx failed (%d)", status);
        return false;
    }

    // Retune to the TX (downlink) channel; no-op when TX shares the RX freq.
    const U32 tx_freq = this->m_txFreqHz ? this->m_txFreqHz : this->m_freqHz;
    if (!this->fskTune(tx_freq)) {
        return false;
    }

    status = lr20xx_radio_common_set_tx(this, FSK_TX_TIMEOUT_MS);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_tx failed (%d)", status);
        return false;
    }

    this->m_txInFlight = true;
    // DEBUG("TX started, %u bytes", len);

    // Sample the antenna coupler RF power detectors while the PA is on.
    this->rfPowerMeasureTx();
    return true;
}

bool LR2021Manager ::fskRx(U32 timeout_ms) {
    if (this->m_mode != RadioMode::FSK) {
        return false;
    }

    // Restore the receive-channel configuration of this role (a TX may have
    // switched the syncword / packet length to the transmit channel).
    // SPACECRAFT receives TC CLTUs; GROUND receives TM frames.
    const bool rx_is_tc = (this->m_ccsdsRole == CcsdsRole::SPACECRAFT);
    lr20xx_status_t status = fskApplySyncword(this, rx_is_tc, false);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("RX set_syncword failed (%d)", status);
        return false;
    }

    const lr20xx_radio_fsk_pkt_params_t pkt_params =
        fskPktParams(rx_is_tc ? FSK_CLTU_PKT_LEN : FSK_TM_PKT_LEN, rx_is_tc);
    status = lr20xx_radio_fsk_set_packet_params(this, &pkt_params);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("RX set_packet_params failed (%d)", status);
        return false;
    }

    status = lr20xx_radio_fifo_clear_rx(this);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("fifo_clear_rx failed (%d)", status);
        return false;
    }

    // Retune to the RX (uplink / rest) channel; no-op when RX shares the freq.
    const U32 rx_freq = this->m_rxFreqHz ? this->m_rxFreqHz : this->m_freqHz;
    if (!this->fskTune(rx_freq)) {
        return false;
    }

    if (timeout_ms == 0) {
        status = lr20xx_radio_common_set_rx_with_timeout_in_rtc_step(this, FSK_RX_CONTINUOUS);
    } else {
        status = lr20xx_radio_common_set_rx(this, timeout_ms);
    }
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_rx failed (%d)", status);
        return false;
    }

    this->m_rxContinuous = (timeout_ms == 0);
    // DEBUG("RX started (%s)", this->m_rxContinuous ? "continuous" : "timeout");
    return true;
}

void LR2021Manager ::fskService() {
    if (this->m_mode != RadioMode::FSK) {
        return;
    }

    // DIO7 is configured as the IRQ output: skip the SPI status poll while
    // the line is low (irqPending() returns true when the port is unwired,
    // falling back to pure SPI polling).
    if (!this->irqPending()) {
        return;
    }

    lr20xx_system_irq_mask_t irq = 0;
    if (lr20xx_system_get_and_clear_irq_status(this, &irq) != LR20XX_STATUS_OK) {
        return;
    }
    if (irq == LR20XX_SYSTEM_IRQ_NONE) {
        return;
    }

    if ((irq & LR20XX_SYSTEM_IRQ_TX_DONE) != 0) {
        this->m_txInFlight = false;
        this->m_fskTxCount++;
        this->tlmWrite_FskTxCount(this->m_fskTxCount);
        // this->log_ACTIVITY_HI_FskTxDone();
        DEBUG("TX done, %llu bytes", this->m_workingBuffer.getSize());
        if (this->isConnected_asyncSendReturnIn_OutputPort(0)) {
            Fw::Buffer buffer = m_workingBuffer;
            m_workingBuffer = Fw::Buffer();
            this->asyncSendReturnIn_out(0, buffer, Drv::ByteStreamStatus::OP_OK);
        }
        this->fskRx(0);
    }

    if ((irq & LR20XX_SYSTEM_IRQ_RX_DONE) != 0) {
        uint16_t pkt_len = 0;
        (void)lr20xx_radio_common_get_rx_packet_length(this, &pkt_len);
        if (pkt_len > FSK_MAX_PAYLOAD) {
            pkt_len = FSK_MAX_PAYLOAD;
        }

        lr20xx_radio_fsk_packet_status_t pkt_status = {};
        (void)lr20xx_radio_fsk_get_packet_status(this, &pkt_status);

        U8 payload[FSK_MAX_PAYLOAD] = {0};
        U8 decoded[FSK_CLTU_DATA_CAP];
        const U8* out_data = payload;
        U16 out_len = 0;
        if ((pkt_len > 0) && (lr20xx_radio_fifo_read_rx(this, payload, pkt_len) == LR20XX_STATUS_OK)) {
            if (this->m_ccsdsRole == CcsdsRole::SPACECRAFT) {
                this->logHex("CLTU raw", payload, 4);
                // TC uplink capture: decode the CLTU codeblocks; the recovered
                // bytes (TC frame + fill) go to the FrameAccumulator, which
                // extracts the frame and discards the fill.
                out_len = cltuDecode(payload, pkt_len, decoded, sizeof decoded);
                out_data = decoded;
                if (out_len == 0) {
                    DEBUG("CLTU decode failed (%u bytes)", pkt_len);
                    this->logHex("CLTU head", payload, 16);
                }
            } else {
                // TM downlink capture: undo the CCSDS pseudo-randomization
                // and strip the RS parity (no onboard correction; the frame
                // CRC rejects corrupted frames downstream).
                if (FSK_TM_RANDOMIZE) {
                    ccsdsRandomize(payload, pkt_len);
                }
                out_len = pkt_len;
                if (FSK_TM_RS && (pkt_len == FSK_TM_PKT_LEN)) {
                    out_len = Rs::RS_DATA;
                }
            }
        }
        if (out_len > 0) {
            this->logHex("FSK RX", out_data, (out_len > 32) ? 32 : out_len);
            Fw::Buffer recv_buffer = this->allocate_out(0, out_len);
            if (recv_buffer.getData()) {
                memcpy(recv_buffer.getData(), out_data, out_len);
                recv_buffer.setSize(out_len);
                this->recv_out(0, recv_buffer, Drv::ByteStreamStatus::OP_OK);
            }
        }

        this->m_fskRxCount++;
        this->tlmWrite_FskRxCount(this->m_fskRxCount);
        this->tlmWrite_FskRssi(pkt_status.rssi_sync_in_dbm);
        this->log_ACTIVITY_HI_FskRxPacket(out_len, pkt_status.rssi_sync_in_dbm);
        this->fskRx(0);
    }

    const U32 error_mask =
        LR20XX_SYSTEM_IRQ_TIMEOUT | LR20XX_SYSTEM_IRQ_CRC_ERROR | LR20XX_SYSTEM_IRQ_LEN_ERROR;
    if ((irq & error_mask) != 0) {
        // A TX timeout also ends any in-flight transmission.
        this->m_txInFlight = false;
        this->log_WARNING_HI_FskError(static_cast<U32>(irq));
    }
}

}  // namespace LR2021
