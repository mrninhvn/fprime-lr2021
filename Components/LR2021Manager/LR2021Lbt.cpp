// ======================================================================
// \title  LR2021Lbt.cpp
// \author ninhdh4
// \brief  Listen-before-talk (half-duplex channel access) for the
//         LR2021Manager component. Rationale and defaults: the LBT section
//         of LR2021Cfg.hpp.
//
// Flow of one frame (dataIn / relayIn -> working buffer of the slot):
//   txSend()      - turnaround window after our last TX still open -> hold
//                   until it ends; link-aware carrier sense: a peer frame
//                   arriving (or a received packet not yet read) ->
//                   lbtDefer(); otherwise fskTx / flrcTx, ending in txKey()
//   txKey()       - CAD gate off: SetTx. On: SetCadParams(exit mode TX) +
//                   SetCad, so the chip keys up by itself only if the TX
//                   channel stays below the threshold for the listen window
//   lbtCadDone()  - service loop, CAD_DONE: CAD_DETECTED -> lbtDefer() (the
//                   caller re-arms RX)
//   lbtRun()      - every run() tick: retry a held frame once its backoff
//                   ends; idle-channel RSSI for the noise-floor telemetry
// ======================================================================

#include "Components/LR2021Manager/LR2021Manager.hpp"
#include "Components/LR2021Manager/LR2021Cfg.hpp"

extern "C" {
#include "lr20xx_radio_common.h"
#include "lr20xx_system.h"
}

namespace LR2021 {

namespace {

// Largest LR2021 CAD listen window: the 24-bit cad_timeout in 32 MHz ticks.
constexpr U32 CAD_TIMEOUT_MAX_TICKS = 0xFFFFFF;
constexpr U32 CAD_TICKS_PER_US = 32;

// IRQs that end a reception (successfully or not): the peer frame is over.
constexpr U32 RX_END_IRQ_MASK = LR20XX_SYSTEM_IRQ_RX_DONE | LR20XX_SYSTEM_IRQ_CRC_ERROR |
                                LR20XX_SYSTEM_IRQ_LEN_ERROR | LR20XX_SYSTEM_IRQ_ADDR_ERROR |
                                LR20XX_SYSTEM_IRQ_TIMEOUT;

Fw::Time timeAfterUs(const Fw::Time& now, U32 us) {
    Fw::Time t = now;
    t.add(us / 1000000U, us % 1000000U);
    return t;
}

// True once \p deadline has passed. An unset or incomparable deadline (time
// base changed, e.g. after a ground SET_TIME) counts as passed, and so does
// one more than \p max_wait_us ahead (wall clock stepped backwards): a clock
// jump can at worst cause one early retry, never wedge a held frame.
bool deadlinePassed(const Fw::Time& now, const Fw::Time& deadline, U32 max_wait_us) {
    if (!(now < deadline)) {
        return true;
    }
    const Fw::Time left = Fw::Time::sub(deadline, now);
    const U64 left_us = static_cast<U64>(left.getSeconds()) * 1000000ULL + left.getUSeconds();
    return left_us > max_wait_us;
}

// Link-aware hold lengths for the mode of \p r.
U32 frameHoldUs(const LR2021Manager::RadioSlot& r) {
    return (r.mode == LR2021Manager::RadioMode::FLRC) ? LBT_FLRC_FRAME_HOLD_US : LBT_FSK_FRAME_HOLD_US;
}

U32 preambleHoldUs(const LR2021Manager::RadioSlot& r) {
    return (r.mode == LR2021Manager::RadioMode::FLRC) ? LBT_FLRC_PREAMBLE_HOLD_US : LBT_FSK_PREAMBLE_HOLD_US;
}

// Longest legitimate wait for a held frame: a backoff or a turnaround window.
U32 holdMaxUs(const LR2021Manager::RadioSlot& r) {
    const U32 ms = (r.lbt.turnaroundMs > r.lbt.backoffMaxMs) ? r.lbt.turnaroundMs : r.lbt.backoffMaxMs;
    return ms * 1000U;
}

}  // namespace

// ----------------------------------------------------------------------
// Configuration
// ----------------------------------------------------------------------

LR2021Manager::LbtCfg LR2021Manager ::lbtDefaults(RadioMode mode) {
    LbtCfg cfg;
    if (mode == RadioMode::FLRC) {
        cfg.cadEnabled = LBT_FLRC_CAD_ENABLED;
        cfg.thresholdDbm = LBT_FLRC_THRESHOLD_DBM;
        cfg.listenUs = LBT_FLRC_LISTEN_US;
        cfg.backoffMinMs = LBT_FLRC_BACKOFF_MIN_MS;
        cfg.backoffMaxMs = LBT_FLRC_BACKOFF_MAX_MS;
        cfg.turnaroundMs = LBT_FLRC_TURNAROUND_MS;
    } else {
        cfg.cadEnabled = LBT_FSK_CAD_ENABLED;
        cfg.thresholdDbm = LBT_FSK_THRESHOLD_DBM;
        cfg.listenUs = LBT_FSK_LISTEN_US;
        cfg.backoffMinMs = LBT_FSK_BACKOFF_MIN_MS;
        cfg.backoffMaxMs = LBT_FSK_BACKOFF_MAX_MS;
        cfg.turnaroundMs = LBT_FSK_TURNAROUND_MS;
    }
    cfg.maxAttempts = LBT_MAX_ATTEMPTS;
    return cfg;
}

void LR2021Manager ::setLbt(FwIndexType idx, const LbtCfg& cfg) {
    FW_ASSERT((idx >= 0) && (idx < NUM_RADIOS), static_cast<FwAssertArgType>(idx));
    FW_ASSERT(cfg.backoffMinMs <= cfg.backoffMaxMs, cfg.backoffMinMs, cfg.backoffMaxMs);
    FW_ASSERT(cfg.maxAttempts >= 1);
    this->m_radio[idx].lbt = cfg;
    this->m_radio[idx].lbtCustom = true;
}

// ----------------------------------------------------------------------
// Channel access
// ----------------------------------------------------------------------

bool LR2021Manager ::txKey(RadioSlot& r, U32 tx_timeout_ms, bool lbt) {
    lr20xx_status_t status;
    if (!(lbt && r.lbt.cadEnabled)) {
        status = lr20xx_radio_common_set_tx(&r, tx_timeout_ms);
        if (status != LR20XX_STATUS_OK) {
            DEBUG("set_tx failed (%d)", status);
            return false;
        }
        r.txInFlight = true;
        // Sample the antenna coupler RF power detectors while the PA is on.
        this->rfPowerMeasureTx();
        return true;
    }

    // Hardware LBT. The CAD is a standby command (the radio rests in RX);
    // XOSC standby keeps a TCXO running, as on every other turnaround.
    status = lr20xx_system_set_standby_mode(&r, LR20XX_SYSTEM_STANDBY_MODE_XOSC);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("LBT set_standby failed (%d)", status);
        return false;
    }

    // cad_timeout: listen window in 32 MHz ticks. threshold: in -dBm.
    // tx_rx_timeout: the TX timeout once the channel is clear, in RTC steps
    // (same unit as SetTx).
    const U64 ticks = static_cast<U64>(r.lbt.listenUs) * CAD_TICKS_PER_US;
    lr20xx_radio_common_cad_params_t cad = {};
    cad.timeout = (ticks > CAD_TIMEOUT_MAX_TICKS) ? CAD_TIMEOUT_MAX_TICKS : static_cast<uint32_t>(ticks);
    cad.threshold = static_cast<uint8_t>(-r.lbt.thresholdDbm);
    cad.exit_mode = LR20XX_RADIO_COMMON_CAD_EXIT_MODE_TX;
    cad.tx_rx_timeout = lr20xx_radio_common_convert_time_in_ms_to_rtc_step(tx_timeout_ms);
    status = lr20xx_radio_common_set_cad_params(&r, &cad);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("LBT set_cad_params failed (%d)", status);
        return false;
    }
    status = lr20xx_radio_common_set_cad(&r);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("LBT set_cad failed (%d)", status);
        return false;
    }
    // CAD_DONE (lbtCadDone) tells whether the chip went on to transmit.
    r.cadListening = true;
    r.txInFlight = true;
    return true;
}

bool LR2021Manager ::txSend(RadioSlot& r) {
    FW_ASSERT(r.workingBuffer.isValid());
    if ((r.mode != RadioMode::FSK) && (r.mode != RadioMode::FLRC)) {
        return false;  // no packet engine (unconfigured, CW, TX test)
    }
    // Turnaround window after our last TX: keep listening so the peer can
    // start its frame. A scheduled gap, not a busy channel: no attempt counted.
    const Fw::Time now = this->getTime();
    if (!deadlinePassed(now, r.txQuietUntil, static_cast<U32>(r.lbt.turnaroundMs) * 1000U)) {
        r.txHeld = true;
        r.txRetryTime = r.txQuietUntil;
        return true;
    }
    const bool force = (r.txAttempts >= r.lbt.maxAttempts);
    if (force) {
        // The channel never cleared: send anyway (a saturated band or a
        // threshold below the noise floor must not stall the link).
        this->m_lbtForcedCount++;
        this->tlmWrite_LbtForcedCount(this->m_lbtForcedCount);
        this->log_WARNING_LO_LbtForced(static_cast<U8>(r.idx), r.txAttempts);
    } else if (this->lbtPeerBusy(r)) {
        this->lbtDefer(r);
        return true;
    }

    const U8* data = r.workingBuffer.getData();
    const U16 len = static_cast<U16>(r.workingBuffer.getSize());
    switch (r.mode) {
        case RadioMode::FLRC:
            return this->flrcTx(r, data, len, !force);
        case RadioMode::FSK:
            return this->fskTx(r, data, len, !force);
        default:
            return false;
    }
}

void LR2021Manager ::lbtDefer(RadioSlot& r) {
    const Fw::Time now = this->getTime();
    if (r.txAttempts < 0xFF) {
        r.txAttempts++;
    }
    r.txInFlight = false;
    r.cadListening = false;
    r.txHeld = true;

    // Random backoff in [min, max] ms (xorshift32). The clock's microseconds
    // are mixed in so two nodes that boot identically never draw the same
    // backoff sequence and collide again on every retry.
    U32 x = this->m_lbtRng ^ now.getUSeconds() ^ (static_cast<U32>(r.idx) << 24);
    if (x == 0) {
        x = 0x9E3779B9u;
    }
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    this->m_lbtRng = x;
    const U32 span = static_cast<U32>(r.lbt.backoffMaxMs - r.lbt.backoffMinMs) + 1U;
    const U32 backoff_ms = r.lbt.backoffMinMs + (x % span);
    r.txRetryTime = timeAfterUs(now, backoff_ms * 1000U);

    this->m_lbtBusyCount++;
    this->tlmWrite_LbtBusyCount(this->m_lbtBusyCount);
    DEBUG("radio %d channel busy, attempt %u, retry in %u ms", static_cast<int>(r.idx),
          static_cast<unsigned>(r.txAttempts), static_cast<unsigned>(backoff_ms));
}

void LR2021Manager ::lbtTurnaround(RadioSlot& r) {
    r.txQuietUntil = timeAfterUs(this->getTime(), static_cast<U32>(r.lbt.turnaroundMs) * 1000U);
}

bool LR2021Manager ::lbtPeerBusy(RadioSlot& r) {
    // IRQ flags latched since the last service poll, read without clearing
    // them (the service loop still handles them).
    lr20xx_system_stat1_t stat1;
    lr20xx_system_stat2_t stat2;
    lr20xx_system_irq_mask_t irq = LR20XX_SYSTEM_IRQ_NONE;
    if (lr20xx_system_get_status(&r, &stat1, &stat2, &irq) == LR20XX_STATUS_OK) {
        // A packet received but not read yet: transmitting now would clear the
        // RX FIFO on the TX->RX turnaround. Let the service loop take it first.
        if ((irq & LR20XX_SYSTEM_IRQ_RX_DONE) != 0) {
            return true;
        }
        this->lbtTrackIrq(r, static_cast<U32>(irq));
    }
    return !deadlinePassed(this->getTime(), r.peerBusyUntil, frameHoldUs(r));
}

void LR2021Manager ::lbtTrackIrq(RadioSlot& r, U32 irq) {
    if ((irq & RX_END_IRQ_MASK) != 0) {
        // The frame is over: release the hold (unset time = passed).
        r.peerBusyUntil = Fw::Time();
        return;
    }
    const Fw::Time now = this->getTime();
    if ((irq & LR20XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID) != 0) {
        // Our link's syncword: a peer frame is on the air until RX_DONE.
        r.peerBusyUntil = timeAfterUs(now, frameHoldUs(r));
    } else if ((irq & LR20XX_SYSTEM_IRQ_PREAMBLE_DETECTED) != 0) {
        // Preamble only: hold for the preamble + syncword airtime, never
        // shortening a syncword hold already running.
        const Fw::Time until = timeAfterUs(now, preambleHoldUs(r));
        if (!(r.peerBusyUntil > until)) {
            r.peerBusyUntil = until;
        }
    }
}

bool LR2021Manager ::lbtCadDone(RadioSlot& r, U32 irq) {
    if (!r.cadListening || ((irq & LR20XX_SYSTEM_IRQ_CAD_DONE) == 0)) {
        return false;
    }
    r.cadListening = false;
    if ((irq & LR20XX_SYSTEM_IRQ_CAD_DETECTED) != 0) {
        // Busy: the chip did not key up and is back in its fallback standby.
        this->lbtDefer(r);
        return true;
    }
    // Clear: the chip went straight into TX. Sample the RF detectors if the
    // (short) packet is not already over.
    if ((irq & LR20XX_SYSTEM_IRQ_TX_DONE) == 0) {
        this->rfPowerMeasureTx();
    }
    return false;
}

void LR2021Manager ::lbtRun(RadioSlot& r) {
    const Fw::Time now = this->getTime();

    // Retry a held frame once its backoff has elapsed.
    if (r.txHeld && !r.txInFlight && deadlinePassed(now, r.txRetryTime, holdMaxUs(r))) {
        r.txHeld = false;
        if (!this->txSend(r)) {
            this->txDrop(r);
        }
    }

    // Noise floor: the lowest instantaneous RSSI while idling in RX, skipping
    // our own TX, peer frames and the BER test (whose radios are not idle).
    if (((r.mode != RadioMode::FSK) && (r.mode != RadioMode::FLRC)) || !r.rxContinuous || r.txInFlight ||
        this->m_ber.active) {
        return;
    }
    if (!deadlinePassed(now, r.noiseNextSample, LBT_NOISE_SAMPLE_MS * 1000U)) {
        return;
    }
    r.noiseNextSample = timeAfterUs(now, LBT_NOISE_SAMPLE_MS * 1000U);
    if (!deadlinePassed(now, r.peerBusyUntil, frameHoldUs(r))) {
        return;
    }
    int16_t rssi = 0;
    if (lr20xx_radio_common_get_rssi_inst(&r, &rssi, nullptr) != LR20XX_STATUS_OK) {
        return;
    }
    if (!r.noiseValid || (rssi < r.noiseMinDbm)) {
        r.noiseMinDbm = rssi;
        r.noiseValid = true;
    }
    if (deadlinePassed(now, r.noiseNextReport, LBT_NOISE_REPORT_MS * 1000U)) {
        r.noiseNextReport = timeAfterUs(now, LBT_NOISE_REPORT_MS * 1000U);
        if (r.mode == RadioMode::FLRC) {
            this->tlmWrite_FlrcNoiseFloor(r.noiseMinDbm);
        } else {
            this->tlmWrite_FskNoiseFloor(r.noiseMinDbm);
        }
        r.noiseValid = false;
    }
}

void LR2021Manager ::txDrop(RadioSlot& r) {
    if (!r.workingBuffer.isValid()) {
        return;  // e.g. a BER test packet: nothing to return
    }
    DEBUG("radio %d TX frame dropped", static_cast<int>(r.idx));
    this->log_WARNING_HI_TxFrameDropped(static_cast<U8>(r.idx));
    this->txComplete(r, Fw::Success::FAILURE);
}

// ----------------------------------------------------------------------
// Commands
// ----------------------------------------------------------------------

void LR2021Manager ::RadioLbtConfig_cmdHandler(FwOpcodeType opCode,
                                               U32 cmdSeq,
                                               U8 radio,
                                               Fw::Enabled cad,
                                               I16 threshold_dbm,
                                               U32 listen_us,
                                               U16 backoff_min_ms,
                                               U16 backoff_max_ms,
                                               U8 max_attempts,
                                               U16 turnaround_ms) {
    if (radio >= NUM_RADIOS) {
        this->log_WARNING_LO_BadRadioIndex(radio);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
        return;
    }
    const bool valid = (threshold_dbm <= 0) && (threshold_dbm >= -255) && (listen_us >= 1) &&
                       (listen_us <= CAD_TIMEOUT_MAX_TICKS / CAD_TICKS_PER_US) &&
                       (backoff_min_ms <= backoff_max_ms) && (max_attempts >= 1);
    if (!valid) {
        this->log_WARNING_LO_LbtConfigInvalid(radio);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
        return;
    }
    LbtCfg cfg;
    cfg.cadEnabled = (cad == Fw::Enabled::ENABLED);
    cfg.thresholdDbm = threshold_dbm;
    cfg.listenUs = listen_us;
    cfg.backoffMinMs = backoff_min_ms;
    cfg.backoffMaxMs = backoff_max_ms;
    cfg.maxAttempts = max_attempts;
    cfg.turnaroundMs = turnaround_ms;
    this->setLbt(radio, cfg);
    this->log_ACTIVITY_HI_LbtConfigSet(radio, cad, threshold_dbm, listen_us, backoff_min_ms, backoff_max_ms,
                                       max_attempts, turnaround_ms);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void LR2021Manager ::RadioLbtDefault_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U8 radio) {
    if (radio >= NUM_RADIOS) {
        this->log_WARNING_LO_BadRadioIndex(radio);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
        return;
    }
    RadioSlot& r = this->m_radio[radio];
    r.lbtCustom = false;
    r.lbt = lbtDefaults(r.mode);
    this->log_ACTIVITY_HI_LbtConfigSet(radio, r.lbt.cadEnabled ? Fw::Enabled::ENABLED : Fw::Enabled::DISABLED,
                                       r.lbt.thresholdDbm, r.lbt.listenUs, r.lbt.backoffMinMs,
                                       r.lbt.backoffMaxMs, r.lbt.maxAttempts, r.lbt.turnaroundMs);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

}  // namespace LR2021
