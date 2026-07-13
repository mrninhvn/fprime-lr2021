// ======================================================================
// \title  LR2021Flrc.cpp
// \author ninhdh4
// \brief  FLRC (Fast Long Range Communication, 2.4 GHz) operations for the
//         LR2021Manager component, built on the lr20xx_driver C API.
//
// Flow:
//   flrcInit()    - packet type, RF freq, HF PA / RX path, modulation and
//                   packet params, syncword #1
//   flrcTx()      - update payload length, fill TX FIFO, start TX
//   flrcRx()      - clear RX FIFO, start RX (timeout or continuous)
//   flrcService() - poll IRQ status from the rate group; handle TX_DONE /
//                   RX_DONE / error IRQs (no DIO IRQ line is wired)
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

bool LR2021Manager ::flrcInit(U32 freq_hz, I8 power_dbm) {
    lr20xx_status_t status;

    status = lr20xx_system_set_standby_mode(this, LR20XX_SYSTEM_STANDBY_MODE_RC);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_standby failed (%d)", status);
        return false;
    }

    status = lr20xx_radio_common_set_pkt_type(this, LR20XX_RADIO_COMMON_PKT_TYPE_FLRC);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_pkt_type failed (%d)", status);
        return false;
    }

    status = lr20xx_radio_common_set_rf_freq(this, freq_hz);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_rf_freq failed (%d)", status);
        return false;
    }

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

    status = lr20xx_radio_flrc_set_modulation_params(this, &FLRC_MOD_PARAMS);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_modulation_params failed (%d)", status);
        return false;
    }

    const lr20xx_radio_flrc_pkt_params_t pkt_params = flrcPktParams(FLRC_MAX_PAYLOAD);
    status = lr20xx_radio_flrc_set_pkt_params(this, &pkt_params);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_pkt_params failed (%d)", status);
        return false;
    }

    status = lr20xx_radio_flrc_set_syncword(this, 1, FLRC_SYNCWORD);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_syncword failed (%d)", status);
        return false;
    }

    // Clear any stale IRQs before operations start.
    lr20xx_system_irq_mask_t irq = 0;
    (void)lr20xx_system_get_and_clear_irq_status(this, &irq);

    DEBUG("FLRC init OK: %u Hz, %d dBm", static_cast<unsigned>(freq_hz), power_dbm);
    this->m_mode = RadioMode::FLRC;
    this->m_rxContinuous = false;
    this->m_txInFlight = false;

    if (this->isConnected_ready_OutputPort(0)) {
        this->ready_out(0);
    }
    return true;
}

bool LR2021Manager ::flrcTx(const U8* data, U16 len) {
    if ((this->m_mode != RadioMode::FLRC) || (data == nullptr) || (len == 0) || (len > FLRC_MAX_PAYLOAD)) {
        return false;
    }

    // Refuse while a TX is in flight (cleared by TX_DONE / error in
    // flrcService). Lets callers retry from a periodic loop safely.
    if (this->m_txInFlight) {
        return false;
    }

    // Radio requires at least 6 payload bytes: zero-pad short payloads.
    U8 payload[FLRC_MAX_PAYLOAD] = {0};
    std::memcpy(payload, data, len);
    const U16 tx_len = (len < FLRC_MIN_PAYLOAD) ? FLRC_MIN_PAYLOAD : len;

    const lr20xx_radio_flrc_pkt_params_t pkt_params = flrcPktParams(tx_len);
    lr20xx_status_t status = lr20xx_radio_flrc_set_pkt_params(this, &pkt_params);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("TX set_pkt_params failed (%d)", status);
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

    status = lr20xx_radio_common_set_tx(this, FLRC_TX_TIMEOUT_MS);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_tx failed (%d)", status);
        return false;
    }

    this->m_txInFlight = true;
    // DEBUG("TX started, %u bytes", tx_len);
    return true;
}

bool LR2021Manager ::flrcRx(U32 timeout_ms) {
    if (this->m_mode != RadioMode::FLRC) {
        return false;
    }

    // Restore max payload length so any packet size is accepted (VAR_LEN).
    const lr20xx_radio_flrc_pkt_params_t pkt_params = flrcPktParams(FLRC_MAX_PAYLOAD);
    lr20xx_status_t status = lr20xx_radio_flrc_set_pkt_params(this, &pkt_params);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("RX set_pkt_params failed (%d)", status);
        return false;
    }

    status = lr20xx_radio_fifo_clear_rx(this);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("fifo_clear_rx failed (%d)", status);
        return false;
    }

    if (timeout_ms == 0) {
        status = lr20xx_radio_common_set_rx_with_timeout_in_rtc_step(this, FLRC_RX_CONTINUOUS);
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

void LR2021Manager ::flrcService() {
    if (this->m_mode != RadioMode::FLRC) {
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
        this->m_txCount++;
        this->tlmWrite_FlrcTxCount(this->m_txCount);
        // this->log_ACTIVITY_HI_FlrcTxDone();
        DEBUG("TX done, %llu bytes", this->m_workingBuffer.getSize());
        if (this->isConnected_asyncSendReturnIn_OutputPort(0)) {
            Fw::Buffer buffer = m_workingBuffer;
            m_workingBuffer = Fw::Buffer();
            this->asyncSendReturnIn_out(0, buffer, Drv::ByteStreamStatus::OP_OK);
        }
        this->flrcRx(0);
    }

    if ((irq & LR20XX_SYSTEM_IRQ_RX_DONE) != 0) {
        uint16_t pkt_len = 0;
        (void)lr20xx_radio_common_get_rx_packet_length(this, &pkt_len);
        if (pkt_len > FLRC_MAX_PAYLOAD) {
            pkt_len = FLRC_MAX_PAYLOAD;
        }

        lr20xx_radio_flrc_pkt_status_t pkt_status = {};
        (void)lr20xx_radio_flrc_get_pkt_status(this, &pkt_status);

        U8 payload[FLRC_MAX_PAYLOAD] = {0};
        if ((pkt_len > 0) && (lr20xx_radio_fifo_read_rx(this, payload, pkt_len) == LR20XX_STATUS_OK)) {
            this->logHex("FLRC RX", payload, (pkt_len > 32) ? 32 : pkt_len);
            Fw::Buffer recv_buffer = this->allocate_out(0, pkt_len);
            if (recv_buffer.getData()) {
                memcpy(recv_buffer.getData(), payload, pkt_len);
                recv_buffer.setSize(pkt_len);
                this->recv_out(0, recv_buffer, Drv::ByteStreamStatus::OP_OK);
            }
        }

        this->m_rxCount++;
        this->tlmWrite_FlrcRxCount(this->m_rxCount);
        this->tlmWrite_FlrcRssi(pkt_status.rssi_avg_in_dbm);
        this->log_ACTIVITY_HI_FlrcRxPacket(pkt_len, pkt_status.rssi_avg_in_dbm);
        this->flrcRx(0);
    }

    const U32 error_mask =
        LR20XX_SYSTEM_IRQ_TIMEOUT | LR20XX_SYSTEM_IRQ_CRC_ERROR | LR20XX_SYSTEM_IRQ_LEN_ERROR;
    if ((irq & error_mask) != 0) {
        // A TX timeout also ends any in-flight transmission.
        this->m_txInFlight = false;
        this->log_WARNING_HI_FlrcError(static_cast<U32>(irq));
    }
}

}  // namespace LR2021
