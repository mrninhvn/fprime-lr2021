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

#include <cstring>

extern "C" {
#include "lr20xx_system.h"
#include "lr20xx_radio_common.h"
#include "lr20xx_radio_fifo.h"
#include "lr20xx_radio_flrc.h"
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
    // on crystal modules).
    status = lr20xx_radio_common_set_rx_tx_fallback_mode(&r, LR20XX_RADIO_FALLBACK_STDBY_XOSC);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_fallback_mode failed (%d)", status);
        return false;
    }

    status = lr20xx_radio_common_set_rf_freq(&r, freq_hz);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_rf_freq failed (%d)", status);
        return false;
    }
    r.progFreqHz = freq_hz;

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
    status = lr20xx_radio_common_set_tx_params(&r, static_cast<int8_t>(power_dbm * 2),
                                               LR20XX_RADIO_COMMON_RAMP_96_US);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_tx_params failed (%d)", status);
        return false;
    }

    status = lr20xx_radio_flrc_set_modulation_params(&r, &FLRC_MOD_PARAMS);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_modulation_params failed (%d)", status);
        return false;
    }

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

bool LR2021Manager ::flrcTx(RadioSlot& r, const U8* data, U16 len) {
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

    status = lr20xx_radio_common_set_tx(&r, FLRC_TX_TIMEOUT_MS);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_tx failed (%d)", status);
        return false;
    }

    r.txInFlight = true;
    // DEBUG("radio %d TX started, %u bytes", static_cast<int>(r.idx), tx_len);

    // Sample the antenna coupler RF power detectors while the PA is on.
    this->rfPowerMeasureTx();
    return true;
}

bool LR2021Manager ::flrcRx(RadioSlot& r, U32 timeout_ms) {
    if (r.mode != RadioMode::FLRC) {
        return false;
    }

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

    if ((irq & LR20XX_SYSTEM_IRQ_TX_DONE) != 0) {
        r.txInFlight = false;
        this->m_txCount++;
        this->tlmWrite_FlrcTxCount(this->m_txCount);
        // this->log_ACTIVITY_HI_FlrcTxDone(static_cast<U8>(r.idx));
        // Only the framer/downlink path fills the working buffer; a relay TX
        // (relayIn) leaves it empty and logs its own size in relayUplink, so
        // don't print a misleading "0 bytes" here for that case.
        if (r.workingBuffer.isValid()) {
            DEBUG("radio %d TX done, %llu bytes", static_cast<int>(r.idx), r.workingBuffer.getSize());
        }
        // Only a TX that owns an F Prime buffer returns one (and grants the
        // next comStatus credit).
        this->txComplete(r, Fw::Success::SUCCESS);
        this->flrcRx(r, 0);
    }

    if ((irq & LR20XX_SYSTEM_IRQ_RX_DONE) != 0) {
        uint16_t pkt_len = 0;
        (void)lr20xx_radio_common_get_rx_packet_length(&r, &pkt_len);
        if (pkt_len > FLRC_MAX_PAYLOAD) {
            pkt_len = FLRC_MAX_PAYLOAD;
        }

        lr20xx_radio_flrc_pkt_status_t pkt_status = {};
        (void)lr20xx_radio_flrc_get_pkt_status(&r, &pkt_status);

        U8 payload[FLRC_MAX_PAYLOAD] = {0};
        if ((pkt_len > 0) && (lr20xx_radio_fifo_read_rx(&r, payload, pkt_len) == LR20XX_STATUS_OK)) {
            this->logHex("FLRC RX", payload, (pkt_len > 32) ? 32 : pkt_len);
            // Forward to the configured RX sink: dataOut (frame accumulator) in
            // flight, or straight out the UART driver on a ground relay.
            this->forwardRxPacket(payload, pkt_len);
        }

        this->m_rxCount++;
        this->tlmWrite_FlrcRxCount(this->m_rxCount);
        this->tlmWrite_FlrcRssi(pkt_status.rssi_avg_in_dbm);
        this->sendRssiPoly(Svc::PolyDbCfg::PolyDbEntry::POLYDB_ENTRY_OBC_FLRC_RSSI, pkt_status.rssi_avg_in_dbm);
        this->log_ACTIVITY_HI_FlrcRxPacket(static_cast<U8>(r.idx), pkt_len, pkt_status.rssi_avg_in_dbm);
        this->flrcRx(r, 0);
    }

    const U32 error_mask =
        LR20XX_SYSTEM_IRQ_TIMEOUT | LR20XX_SYSTEM_IRQ_CRC_ERROR | LR20XX_SYSTEM_IRQ_LEN_ERROR;
    if ((irq & error_mask) != 0) {
        // A TX timeout also ends any in-flight transmission.
        r.txInFlight = false;
        this->log_WARNING_HI_FlrcError(static_cast<U8>(r.idx), static_cast<U32>(irq));
    }
}

}  // namespace LR2021
