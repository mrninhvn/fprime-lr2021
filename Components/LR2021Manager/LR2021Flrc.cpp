// ======================================================================
// \title  LR2021Flrc.cpp
// \author ninhdh4
// \brief  FLRC (Fast Long Range Communication, 2.4 GHz) operations for the
//         LR2021Manager component, built on the lr20xx_driver C API.
//
// Flow (all per radio slot):
//   flrcInit()    - packet type, RF freq, HF PA / RX path, modulation and
//                   packet params, syncword #1
//   flrcTx()      - update payload length, fill TX FIFO, start TX
//   flrcRx()      - clear RX FIFO, start RX (timeout or continuous)
//   flrcService() - poll IRQ status from the rate group; handle TX_DONE /
//                   RX_DONE / error IRQs
// ======================================================================

#include "Components/LR2021Manager/LR2021Manager.hpp"
#include "Components/LR2021Manager/LR2021Cfg.hpp"

#include <Svc/Ccsds/Utils/CRC16.hpp>
#include <cstring>
#ifdef __ZEPHYR__
#include <zephyr/kernel.h>
#endif

extern "C" {
#include "lr20xx_system.h"
#include "lr20xx_radio_common.h"
#include "lr20xx_radio_fifo.h"
#include "lr20xx_radio_flrc.h"
#include "lr20xx_regmem.h"
}

namespace LR2021 {

namespace {

// Default FLRC configuration. Adjust here (or extend FLRC_INIT) as needed.
constexpr lr20xx_radio_flrc_mod_params_t FLRC_MOD_PARAMS = {
    .br_bw = FLRC_RAW_BIT_RATE,
    .cr    = FLRC_CR,
    .shape = FLRC_PULSE_SHAPE,
};

// Build the packet params for a given payload length.
lr20xx_radio_flrc_pkt_params_t flrcPktParams(uint16_t pld_len) {
    lr20xx_radio_flrc_pkt_params_t params = {};
    params.preamble_len     = FLRC_PREAMBLE_BITS;
    params.sync_word_len    = FLRC_SYNCWORD_LEN;
    params.tx_syncword      = FLRC_TX_SYNCWORD;
    params.match_sync_word  = FLRC_MATCH_SYNCWORD;
    params.header_type      = FLRC_PKT_LEN_TYPE;
    params.pld_len_in_bytes = pld_len;
    params.crc_type         = FLRC_CRC;
    return params;
}

#if LR2021_FLRC_BURST
// Genmod ShapeCfg DblRate (bit 8): TX interpolation by 16 instead of 8, only
// allowed below 2 Mb/s. Semtech's FLRC burst TX sets it after the FLRC setup
// (which rewrites ShapeCfg) and clears it at >= 2 Mb/s.
constexpr uint32_t GENMOD_SHAPECFG_REG = 0x00F30904;
constexpr uint32_t GENMOD_SHAPECFG_DBLRATE = 1UL << 8;
#endif

// Smallest CCSDS transfer frame worth checking: TC primary header (5) +
// one data octet + FECF (2). A TM frame is larger still.
constexpr U16 CCSDS_MIN_FRAME = 8;
constexpr U16 CCSDS_FECF_SIZE = 2;

// The FLRC link runs with the radio CRC off and variable length, so noise
// that matches the 32-bit syncword is delivered as a random-length packet
// (it showed up as NoBuffsAvailable for sizes beyond the comms buffers). The
// UHF FSK path never forwards such noise because its CLTU BCH / fixed-length
// decode rejects it; this is the FLRC equivalent: accept only a CCSDS TC or TM
// transfer frame (version '00') whose Frame Error Control Field, the CRC-16
// that F Prime's TC and TM framers append over the rest of the frame, matches.
// The CRC covers the whole packet, so it also rejects a frame with a wrong
// length; random noise passes with probability ~1/262144.
bool ccsdsFrameValid(const U8* data, U16 len) {
    if ((len < CCSDS_MIN_FRAME) || ((data[0] >> 6) != 0)) {
        return false;
    }
    const U16 fecf = static_cast<U16>((data[len - 2] << 8) | data[len - 1]);
    return Svc::Ccsds::Utils::CRC16::compute(data, static_cast<U32>(len - CCSDS_FECF_SIZE)) == fecf;
}
}  // namespace

bool LR2021Manager ::flrcInit(RadioSlot& r, U32 freq_hz, I8 power_dbm) {
    lr20xx_status_t status;

    status = lr20xx_system_set_standby_mode(&r, LR20XX_SYSTEM_STANDBY_MODE_RC);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_standby failed (%d)", status);
        return false;
    }

    status = lr20xx_radio_common_set_pkt_type(&r, LR20XX_RADIO_COMMON_PKT_TYPE_FLRC);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_pkt_type failed (%d)", status);
        return false;
    }

    // Park in STANDBY_XOSC after each TX/RX so a TCXO module is never
    // power-cycled across the turnaround (see the note in fskInit; harmless
    // on crystal modules). FS (LR2021_FLRC_FALLBACK_FS) also keeps the
    // synthesizer locked, so the next TX of a burst starts sooner.
    status = lr20xx_radio_common_set_rx_tx_fallback_mode(
        &r, LR2021_FLRC_FALLBACK_FS ? LR20XX_RADIO_FALLBACK_FS : LR20XX_RADIO_FALLBACK_STDBY_XOSC);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_fallback_mode failed (%d)", status);
        return false;
    }

    // Base frequency for this mode; RX rests on the RX channel and TX may
    // retune to a separate one (setRxFreq / setTxFreq), as in fskInit.
    r.freqHz = freq_hz;
    const U32 rx_freq = r.rxFreqHz ? r.rxFreqHz : r.freqHz;
    status = lr20xx_radio_common_set_rf_freq(&r, rx_freq);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_rf_freq failed (%d)", status);
        return false;
    }
    r.progFreqHz = rx_freq;

    // Select the RX path and PA matching the requested band:
    // sub-GHz -> LF path/PA, 2.4 GHz -> HF path/PA.
    const bool is_hf = (freq_hz >= 1000000000U);
    status = lr20xx_radio_common_set_rx_path(
        &r, is_hf ? LR20XX_RADIO_COMMON_RX_PATH_HF : LR20XX_RADIO_COMMON_RX_PATH_LF,
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
    status = lr20xx_radio_common_set_pa_cfg(&r, &pa_cfg);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_pa_cfg failed (%d)", status);
        return false;
    }

    // set_tx_params takes 0.5 dBm steps.
    status = lr20xx_radio_common_set_tx_params(&r, static_cast<int8_t>(power_dbm * 2), LR2021_FLRC_PA_RAMP);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_tx_params failed (%d)", status);
        return false;
    }

    status = lr20xx_radio_flrc_set_modulation_params(&r, &FLRC_MOD_PARAMS);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_modulation_params failed (%d)", status);
        return false;
    }

#if LR2021_FLRC_BURST
    status = lr20xx_regmem_write_regmem32_mask(
        &r, GENMOD_SHAPECFG_REG, GENMOD_SHAPECFG_DBLRATE,
        (FLRC_RAW_BITRATE_BPS < 2000000u) ? GENMOD_SHAPECFG_DBLRATE : 0u);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("DblRate write failed (%d)", status);
        return false;
    }
#endif

    const lr20xx_radio_flrc_pkt_params_t pkt_params = flrcPktParams(FLRC_MAX_PAYLOAD);
    status = lr20xx_radio_flrc_set_pkt_params(&r, &pkt_params);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_pkt_params failed (%d)", status);
        return false;
    }

    status = lr20xx_radio_flrc_set_syncword(&r, 1, FLRC_SYNCWORD);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_syncword failed (%d)", status);
        return false;
    }

    // Clear any stale IRQs before operations start.
    lr20xx_system_irq_mask_t irq = 0;
    (void)lr20xx_system_get_and_clear_irq_status(&r, &irq);

    DEBUG("radio %d FLRC init OK: %u Hz, %d dBm", static_cast<int>(r.idx),
          static_cast<unsigned>(freq_hz), power_dbm);
    r.mode = RadioMode::FLRC;
    r.rxContinuous = false;
    r.txInFlight = false;
    // Downlink readiness (initial comStatus) is signalled once by setMode().
    return true;
}

bool LR2021Manager ::flrcTx(RadioSlot& r, const U8* data, U16 len, bool lbt) {
    if ((r.mode != RadioMode::FLRC) || (data == nullptr) || (len == 0) || (len > FLRC_MAX_PAYLOAD)) {
        return false;
    }

    // Refuse while a TX is in flight (cleared by TX_DONE / error in
    // flrcService). Lets callers retry from a periodic loop safely.
    if (r.txInFlight) {
        return false;
    }

    // Radio requires at least 6 payload bytes: zero-pad short payloads.
    U8 payload[FLRC_MAX_PAYLOAD] = {0};
    std::memcpy(payload, data, len);
    const U16 tx_len = (len < FLRC_MIN_PAYLOAD) ? FLRC_MIN_PAYLOAD : len;

    const lr20xx_radio_flrc_pkt_params_t pkt_params = flrcPktParams(tx_len);
    lr20xx_status_t status = lr20xx_radio_flrc_set_pkt_params(&r, &pkt_params);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("TX set_pkt_params failed (%d)", status);
        return false;
    }

    status = lr20xx_radio_fifo_clear_tx(&r);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("fifo_clear_tx failed (%d)", status);
        return false;
    }

    status = lr20xx_radio_fifo_write_tx(&r, payload, tx_len);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("fifo_write_tx failed (%d)", status);
        return false;
    }

    // Split TX/RX plan: retune to the TX channel. The chip sits in continuous
    // RX here, so enter STDBY_XOSC first (SetRfFrequency is a standby command;
    // XOSC keeps a TCXO running). No-op when TX shares the RX frequency.
    // fskTune is mode-independent (cached SetRfFrequency).
    const U32 tx_freq = r.txFreqHz ? r.txFreqHz : r.freqHz;
    if (tx_freq != r.progFreqHz) {
        status = lr20xx_system_set_standby_mode(&r, LR20XX_SYSTEM_STANDBY_MODE_XOSC);
        if (status != LR20XX_STATUS_OK) {
            DEBUG("TX set_standby failed (%d)", status);
            return false;
        }
        if (!this->fskTune(r, tx_freq)) {
            return false;
        }
    }
#if LR2021_FLRC_FALLBACK_FS
    // Lock the synthesizer before keying (Semtech FLRC burst: "needed in
    // 2.4GHz at low datarate to avoid crc errors").
    status = lr20xx_system_set_fs_mode(&r);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("TX set_fs failed (%d)", status);
        return false;
    }
#endif

    // Key up, directly or through the CAD-LBT gate.
    if (!this->txKey(r, FLRC_TX_TIMEOUT_MS, lbt)) {
        return false;
    }
    // DEBUG("radio %d TX started, %u bytes", static_cast<int>(r.idx), tx_len);
    return true;
}

bool LR2021Manager ::flrcRx(RadioSlot& r, U32 timeout_ms) {
    if (r.mode != RadioMode::FLRC) {
        return false;
    }

#if LR2021_FLRC_FALLBACK_FS
    // The FS fallback leaves the synthesizer running: back to STDBY_XOSC for
    // the temperature read and SetRfFrequency (a standby command).
    if (lr20xx_system_set_standby_mode(&r, LR20XX_SYSTEM_STANDBY_MODE_XOSC) != LR20XX_STATUS_OK) {
        DEBUG("RX set_standby failed");
        return false;
    }
#endif
    // On the post-TX turnaround the radio has fallen back to STDBY_XOSC (the
    // only state where the Measure Unit ADC converts), so sample die temperature
    // here before re-arming RX. At initial bring-up the chip is in STDBY_RC and
    // readDieTemp simply reads 0 and skips publishing.
    this->publishTempPoly(r.idx);

    // Restore max payload length so any packet size is accepted (VAR_LEN).
    const lr20xx_radio_flrc_pkt_params_t pkt_params = flrcPktParams(FLRC_MAX_PAYLOAD);
    lr20xx_status_t status = lr20xx_radio_flrc_set_pkt_params(&r, &pkt_params);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("RX set_pkt_params failed (%d)", status);
        return false;
    }

    status = lr20xx_radio_fifo_clear_rx(&r);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("fifo_clear_rx failed (%d)", status);
        return false;
    }

    // Back to the RX (rest) channel. The chip is in standby here (TX_DONE
    // falls back to STDBY_XOSC; flrcInit leaves STDBY_RC). No-op when the
    // TX and RX frequencies are the same.
    const U32 rx_freq = r.rxFreqHz ? r.rxFreqHz : r.freqHz;
    if (!this->fskTune(r, rx_freq)) {
        return false;
    }

    if (timeout_ms == 0) {
        status = lr20xx_radio_common_set_rx_with_timeout_in_rtc_step(&r, FLRC_RX_CONTINUOUS);
    } else {
        status = lr20xx_radio_common_set_rx(&r, timeout_ms);
    }
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_rx failed (%d)", status);
        return false;
    }

    r.rxContinuous = (timeout_ms == 0);
    return true;
}

void LR2021Manager ::flrcService(RadioSlot& r) {
    if (r.mode != RadioMode::FLRC) {
        return;
    }

    // The radio's IRQ line gates the SPI status poll: skip it while the
    // line is low (irqPending() returns true when the port is unwired,
    // falling back to pure SPI polling).
    if (!this->irqPending(r.idx)) {
        return;
    }

    lr20xx_system_irq_mask_t irq = 0;
    if (lr20xx_system_get_and_clear_irq_status(&r, &irq) != LR20XX_STATUS_OK) {
        return;
    }
    if (irq == LR20XX_SYSTEM_IRQ_NONE) {
        return;
    }

    // Link-aware carrier sense: note peer preamble / syncword / RX end.
    this->lbtTrackIrq(r, irq);

    // CAD-LBT result: a busy channel means the chip never keyed up; listen
    // again while the frame backs off.
    if (this->lbtCadDone(r, irq)) {
        this->flrcRx(r, 0);
        return;
    }

    if ((irq & LR20XX_SYSTEM_IRQ_TX_DONE) != 0) {
        r.txInFlight = false;
        this->m_txCount++;
        this->tlmWrite_FlrcTxCount(this->m_txCount);
        // this->log_ACTIVITY_HI_FlrcTxDone(static_cast<U8>(r.idx));
        // BER test packets are sent without a working buffer.
        if (r.workingBuffer.isValid()) {
            DEBUG("radio %d TX done, %llu bytes", static_cast<int>(r.idx), r.workingBuffer.getSize());
        }
        // Burst: the next frame is waiting, key it right away.
        if (!this->burstKeyStaged(r)) {
            // Return the frame's buffer (downlink: plus the next comStatus
            // credit; relay: freed; burst frame: already returned).
            this->txComplete(r, Fw::Success::SUCCESS);
            this->flrcRx(r, 0);
            // Listen-only gap before our next TX, so the peer can get a word in.
            this->lbtTurnaround(r);
        }
    }

    if ((irq & LR20XX_SYSTEM_IRQ_RX_DONE) != 0) {
        uint16_t pkt_len = 0;
        (void)lr20xx_radio_common_get_rx_packet_length(&r, &pkt_len);
        if (pkt_len > FLRC_MAX_PAYLOAD) {
            pkt_len = FLRC_MAX_PAYLOAD;
        }

        // BER test: when this is the test's RX radio, tally the raw packet and
        // re-arm; skip the normal forward to the com stack.
        if (this->berRxIntercept(r, pkt_len)) {
            return;
        }

        lr20xx_radio_flrc_pkt_status_t pkt_status = {};
        (void)lr20xx_radio_flrc_get_pkt_status(&r, &pkt_status);

        U8 payload[FLRC_MAX_PAYLOAD] = {0};
        const bool read = (pkt_len > 0) && (lr20xx_radio_fifo_read_rx(&r, payload, pkt_len) == LR20XX_STATUS_OK);
        if (read && !ccsdsFrameValid(payload, pkt_len)) {
            // Noise on the syncword or a corrupted frame: drop it before a
            // buffer is allocated, and keep it out of the RX count / RSSI.
            this->flrcRx(r, 0);
            return;
        }
        if (read) {
            this->logHex("FLRC RX", payload, (pkt_len > 32) ? 32 : pkt_len);
            // Forward to the configured RX sink: dataOut (frame accumulator) in
            // flight, or straight out the UART driver on a ground relay.
            this->forwardRxPacket(r.idx, payload, pkt_len);
        }

        this->m_rxCount++;
        this->tlmWrite_FlrcRxCount(this->m_rxCount);
        this->tlmWrite_FlrcRssi(pkt_status.rssi_avg_in_dbm);
        this->tlmWrite_BeaconFlrcRssi(static_cast<I8>(pkt_status.rssi_avg_in_dbm));
        this->sendRssiPoly(Svc::PolyDbCfg::PolyDbEntry::POLYDB_ENTRY_OBC_FLRC_RSSI, pkt_status.rssi_avg_in_dbm);
        // Per-packet event disabled: at S-band file rates (~100 frames/s) it floods the event
        // stream and the console; FlrcRxCount / FlrcRssi carry the same information.
        // this->log_ACTIVITY_HI_FlrcRxPacket(static_cast<U8>(r.idx), pkt_len, pkt_status.rssi_avg_in_dbm);
        this->flrcRx(r, 0);
    }

    const U32 error_mask =
        LR20XX_SYSTEM_IRQ_TIMEOUT | LR20XX_SYSTEM_IRQ_CRC_ERROR | LR20XX_SYSTEM_IRQ_LEN_ERROR;
    if ((irq & error_mask) != 0) {
        this->log_WARNING_HI_FlrcError(static_cast<U8>(r.idx), static_cast<U32>(irq));
        // A TX timeout ends the in-flight transmission: drop its frame so the
        // slot (and the downlink credit) is not held forever, then listen.
        if (r.txInFlight) {
            this->burstAbort(r);
            r.txInFlight = false;
            r.cadListening = false;
            this->txDrop(r);
            this->flrcRx(r, 0);
        }
    }
}

// ----------------------------------------------------------------------
// FLRC burst (LR2021_FLRC_BURST)
//
// After the first frame of a burst is keyed, its buffer and the next credit
// go back at once, so the framer stack prepares the next frame during this
// one's airtime. That frame is written straight into the chip TX FIFO behind
// the one on the air (credit held: one frame staged, none ever dropped) and
// on TX_DONE it is keyed with set_tx alone: no FIFO write, RX re-arm or
// retune in between. The radio goes back to RX when no frame is waiting.
// Other radios (UHF) keep one frame per credit.
//
// Same scheme as Semtech's FLRC burst TX (usp v1.2.1
// smtc_rac_flrc_burst.c, used by FLRP-BURST): patch RAM + 1024-byte FIFO
// (radioInit), next packet written during the current TX, key without
// set_pkt_params unless the length changes (never here: a burst keeps one
// length), FS fallback with set_fs before the first key, DblRate below
// 2 Mb/s. Semtech keys on TX_TIMESTAMP (last bit); this keys on TX_DONE.
//
// History (FW 0x18, driver 1.3.4, no patch RAM, 255-byte FIFO, 2026-10-30;
// details in docs/flrc-burst.md): two frames did not fit in the FIFO; a
// FIFO write during TX left the next packet without preamble / syncword /
// header, so frames were staged in RAM and written at TX_DONE; that needed
// the 128 us PA ramp (LR2021_FLRC_PA_RAMP) and >= ~450 us from TX_DONE to
// set_tx (LR2021_FLRC_BURST_KEY_DELAY_US), and still left ~40 us of each
// staged frame's head corrupt at 2.08 Mb/s.
// ----------------------------------------------------------------------

void LR2021Manager ::burstStart(RadioSlot& r) {
#if LR2021_FLRC_BURST
    if ((r.mode != RadioMode::FLRC) || !r.workingBuffer.isValid() || r.relayFrame || r.beaconFrame ||
        (r.workingBuffer.getSize() < FLRC_MIN_PAYLOAD) || (r.workingBuffer.getSize() > BURST_FRAME_MAX)) {
        return;
    }
    // The frame is in the chip now: hand the buffer back with the next credit.
    Fw::Buffer buffer = r.workingBuffer;
    ComCfg::FrameContext context = r.workingContext;
    r.workingBuffer = Fw::Buffer();
    r.workingContext = ComCfg::FrameContext();
    r.burstInFlight = true;
    r.burstLen = buffer.getSize();
    if (this->isConnected_dataReturnOut_OutputPort(0)) {
        this->dataReturnOut_out(0, buffer, context);
    }
    if (this->isConnected_comStatusOut_OutputPort(0)) {
        Fw::Success success = Fw::Success::SUCCESS;
        this->comStatusOut_out(0, success);
    }
    this->m_comOpen = true;
#else
    (void)r;
#endif
}

bool LR2021Manager ::burstStage(RadioSlot& r, Fw::Buffer& data, const ComCfg::FrameContext& context) {
#if LR2021_FLRC_BURST
    if (!r.burstInFlight || !r.txInFlight) {
        return false;
    }
    // Not stageable (length differs from the burst's, or a frame is staged
    // already): hold it until the slot is free, like a frame held back by a
    // beacon. Its buffer and credit stay out meanwhile.
    if (r.burstStaged || r.burstCreditHeld || !data.isValid() || (data.getSize() != r.burstLen)) {
        if (r.pendingFramer.isValid()) {
            return false;  // cannot happen with one credit; the caller drops it
        }
        r.pendingFramer = data;
        r.pendingFramerContext = context;
        return true;
    }
    // Write it into the TX FIFO behind the frame on the air and return the
    // framer buffer; the credit is held until the frame is keyed.
    this->stampTmFrame(data.getData(), data.getSize());
    const bool written =
        (lr20xx_radio_fifo_write_tx(&r, data.getData(), static_cast<uint16_t>(data.getSize())) == LR20XX_STATUS_OK);
    if (this->isConnected_dataReturnOut_OutputPort(0)) {
        this->dataReturnOut_out(0, data, context);
    }
    r.burstCreditHeld = true;
    if (!written) {
        // Not keyed: the burst ends on TX_DONE, which grants the credit, and
        // the next flrcTx clears the FIFO (clearing it now would cut the
        // frame on the air).
        DEBUG("burst fifo_write_tx failed");
        this->log_WARNING_HI_TxFrameDropped(static_cast<U8>(r.idx));
        return true;
    }
    r.burstStaged = true;
    return true;
#else
    (void)r;
    (void)data;
    (void)context;
    return false;
#endif
}

bool LR2021Manager ::burstKeyStaged(RadioSlot& r) {
#if LR2021_FLRC_BURST
    if (!r.burstStaged) {
        // Burst over: back to RX (the caller re-arms it). A credit is still
        // held only after a failed FIFO write.
        r.burstInFlight = false;
        this->burstGrantCredit(r);
        return false;
    }
#if defined(__ZEPHYR__) && (LR2021_FLRC_BURST_KEY_DELAY_US > 0)
    // Minimum interframe (see LR2021_FLRC_BURST_KEY_DELAY_US), from handling
    // TX_DONE. The staged frame is in the FIFO already: nothing else to do.
    const uint32_t doneCyc = k_cycle_get_32();
    while (k_cyc_to_us_floor32(k_cycle_get_32() - doneCyc) < LR2021_FLRC_BURST_KEY_DELAY_US) {
    }
#endif
    if (lr20xx_radio_common_set_tx(&r, FLRC_TX_TIMEOUT_MS) != LR20XX_STATUS_OK) {
        DEBUG("burst key failed");
        this->burstAbort(r);  // drops the staged frame, grants its credit
        return false;
    }
    r.burstStaged = false;
    r.txInFlight = true;
    this->burstGrantCredit(r);
    return true;
#else
    (void)r;
    return false;
#endif
}

void LR2021Manager ::burstGrantCredit(RadioSlot& r) {
    if (!r.burstCreditHeld) {
        return;
    }
    r.burstCreditHeld = false;
    if (this->isConnected_comStatusOut_OutputPort(0)) {
        Fw::Success success = Fw::Success::SUCCESS;
        this->comStatusOut_out(0, success);
    }
    this->m_comOpen = true;
}

void LR2021Manager ::burstAbort(RadioSlot& r) {
#if LR2021_FLRC_BURST
    if (r.burstInFlight && r.txInFlight) {
        this->log_WARNING_HI_TxFrameDropped(static_cast<U8>(r.idx));  // burst frame cut off on the air
    }
    if (r.burstStaged) {
        this->log_WARNING_HI_TxFrameDropped(static_cast<U8>(r.idx));
    }
    r.burstInFlight = false;
    r.burstStaged = false;
    this->burstGrantCredit(r);
#else
    (void)r;
#endif
}

}  // namespace LR2021
