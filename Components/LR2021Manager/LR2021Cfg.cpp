// ======================================================================
// \title  LR2021Cfg.cpp
// \author ninhdh4
// \brief  LR2021 configuration operations for the LR2021Manager component,
//         built on the lr20xx_driver C API.
//
// Flow:
//   radioInit(idx) - reset, patch RAM, regulator, clocks, calibration, DIO /
//                    RF-switch map for the module type populated on that
//                    radio slot, 1024-byte FIFOs (order of USP v1.2.1
//                    ral_lr20xx_init)
// ======================================================================

#include "Components/LR2021Manager/LR2021Manager.hpp"
#include "Components/LR2021Manager/LR2021Cfg.hpp"

extern "C" {
#include "lr20xx_patch.h"
#include "lr20xx_pram_load.h"
#include "lr20xx_system.h"
#include "lr20xx_radio_common.h"
#include "lr20xx_radio_fifo.h"
}

namespace LR2021 {

bool LR2021Manager ::radioInit(FwIndexType idx) {
    FW_ASSERT((idx >= 0) && (idx < NUM_RADIOS), static_cast<FwAssertArgType>(idx));
    RadioSlot& r = this->m_radio[idx];

    // Enable this module's load switch (no-op when the port is unconnected).
    if (this->isConnected_powerGpioWrite_OutputPort(idx)) {
        this->powerGpioWrite_out(idx, Fw::Logic::HIGH);
    }

    lr20xx_status_t status;

    status = lr20xx_system_reset(&r);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("radio %d reset failed (%d)", static_cast<int>(idx), status);
        return false;
    }

    // Patch RAM. Since lr20xx_driver v2.0 the chip limitations (among them
    // the DC-DC sub-GHz sensitivity fix the driver used to apply as a
    // workaround) are fixed by the PRAM, which must be loaded after every
    // reset (and after a sleep without retention, not used here).
    status = lr20xx_pram_load_pram_lr2021(&r);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("radio %d PRAM load failed (%d)", static_cast<int>(idx), status);
        return false;
    }
    lr20xx_patch_version_t pram = {false, 0, 0};
    if (lr20xx_patch_get_version(&r, &pram) == LR20XX_STATUS_OK) {
        Fw::LogStringArg msg;
        if (pram.is_pram_loaded) {
            msg.format("radio %d PRAM type %X v%X", static_cast<int>(idx), pram.pram_type, pram.pram_version);
        } else {
            msg.format("radio %d PRAM not loaded", static_cast<int>(idx));
        }
        this->log_DIAGNOSTIC_LR2021(msg);
    }

    // System configuration mirroring the validated USP overlay for these
    // radio modules: DC-DC regulator, internal 32 kHz RC, module-specific
    // HF clock source.
    status = lr20xx_system_set_reg_mode(&r, LR20XX_SYSTEM_REG_MODE_DCDC);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_reg_mode failed (%d)", status);
        return false;
    }

    status = lr20xx_system_cfg_lfclk(&r, LR20XX_SYSTEM_LFCLK_RC);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("cfg_lfclk failed (%d)", status);
        return false;
    }

    // HF clock source, module-specific.
    // NiceRF LoRa2021F33: chip-supplied TCXO at 3.3 V. The third argument is
    // the start-up gating window in RTC steps (1 step = 1/32768 s; the driver
    // header wrongly calls it a 32 MHz step). 2000 steps ~= 61 ms,
    // bench-proven.
    // RY42F (RYLR428): plain crystal, no TCXO - leave the reset-default
    // XOSC configuration untouched.
    if (r.moduleType == ModuleType::NICERF) {
        status = lr20xx_system_set_tcxo_mode(&r, LR20XX_SYSTEM_TCXO_CTRL_3_3V, 2000);
        if (status != LR20XX_STATUS_OK) {
            DEBUG("set_tcxo_mode failed (%d)", status);
            return false;
        }
    }

    // Clear the reset-latched error state before exercising the HF clock. A
    // HF_XOSC_START (0x1) is expected on the TCXO module: at reset, before
    // TCXO mode was configured, the chip tried and failed to start its
    // default oscillator.
    lr20xx_system_errors_t errors = 0;
    if ((lr20xx_system_get_errors(&r, &errors) == LR20XX_STATUS_OK) && (errors != 0)) {
        DEBUG("radio %d errors after clock cfg: 0x%X (cleared)", static_cast<int>(idx),
              static_cast<unsigned>(errors));
        (void)lr20xx_system_clear_errors(&r);
    }

    // Calibrate the Measure Unit ADC. Without this one-time boot calibration
    // the ADC-backed measurements (GetTemp / GetVbat) return 0. Per the driver
    // docstring this "should be executed at boot" and, for the MU block,
    // "initial calibration is enough". On the TCXO module the RC clocks, PLL,
    // AAF and PA offset are recalibrated too, once the TCXO runs (USP v1.2.1
    // fix "calibrate after configuring tcxo", mask 0x6F). Calibrate returns
    // the chip to STDBY_RC.
    const lr20xx_system_calibration_mask_t calib_mask =
        (r.moduleType == ModuleType::NICERF)
            ? static_cast<lr20xx_system_calibration_mask_t>(
                  LR20XX_SYSTEM_CALIB_LF_RC_MASK | LR20XX_SYSTEM_CALIB_HF_RC_MASK | LR20XX_SYSTEM_CALIB_PLL_MASK |
                  LR20XX_SYSTEM_CALIB_AAF_MASK | LR20XX_SYSTEM_CALIB_MU_MASK | LR20XX_SYSTEM_CALIB_PA_OFF_MASK)
            : static_cast<lr20xx_system_calibration_mask_t>(LR20XX_SYSTEM_CALIB_MU_MASK);
    status = lr20xx_system_calibrate(&r, calib_mask);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("calibrate (0x%X) failed (%d)", calib_mask, status);
        return false;
    }
    // The calibration holds BUSY while the TCXO starts (~61 ms window) and
    // the blocks calibrate; give it the same margin as a cold XOSC entry.
    if (!this->waitOnBusy(idx, 2500000)) {
        DEBUG("radio %d BUSY stuck after calibrate", static_cast<int>(idx));
        return false;
    }
    errors = 0;
    if ((lr20xx_system_get_errors(&r, &errors) == LR20XX_STATUS_OK) && (errors != 0)) {
        DEBUG("radio %d errors after calibrate: 0x%X (cleared)", static_cast<int>(idx),
              static_cast<unsigned>(errors));
        (void)lr20xx_system_clear_errors(&r);
    }

    // DIO routing is MODULE-specific (each module wires its RF switch tree
    // and IRQ pad differently). Getting this wrong gives "TX started" with
    // no RF (antenna switch never closes) and no TX_DONE seen by the MCU
    // (IRQ on the wrong pad), which wedges txInFlight after the first TX.
    //
    // NiceRF LoRa2021F33 (per the validated nicerf_2021f33 shield dtsi):
    // IRQ on DIO9; dio5=HF_RX, dio6=LF_TX, dio8=HF_TX; DIO7 must be driven
    // constantly HIGH (module control line); no LF_RX switch line (the LF RX
    // path is the switch tree's resting position).
    // RY42F (RYLR428): IRQ on DIO7; RF switches dio8=LF_RX, dio9=LF_TX,
    // dio10=HF_RX, dio11=HF_TX.
    struct RfSwitchCfg {
        lr20xx_system_dio_t dio;
        lr20xx_system_dio_rf_switch_cfg_t cfg;
    };
    const bool is_nicerf = (r.moduleType == ModuleType::NICERF);
    const lr20xx_system_dio_t irq_dio = is_nicerf ? LR20XX_SYSTEM_DIO_9 : LR20XX_SYSTEM_DIO_7;
    const RfSwitchCfg nicerf_switches[] = {
        {LR20XX_SYSTEM_DIO_5, LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_RX_HF},
        {LR20XX_SYSTEM_DIO_6, LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_TX_LF},
        {LR20XX_SYSTEM_DIO_8, LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_TX_HF},
    };
    const RfSwitchCfg ry42f_switches[] = {
        {LR20XX_SYSTEM_DIO_8, LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_RX_LF},
        {LR20XX_SYSTEM_DIO_9, LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_TX_LF},
        {LR20XX_SYSTEM_DIO_10, LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_RX_HF},
        {LR20XX_SYSTEM_DIO_11, LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_TX_HF},
    };
    const RfSwitchCfg* rf_switches = is_nicerf ? nicerf_switches : ry42f_switches;
    const FwSizeType rf_switch_count =
        is_nicerf ? FW_NUM_ARRAY_ELEMENTS(nicerf_switches) : FW_NUM_ARRAY_ELEMENTS(ry42f_switches);

    status = lr20xx_system_set_dio_function(&r, irq_dio, LR20XX_SYSTEM_DIO_FUNC_IRQ,
                                            LR20XX_SYSTEM_DIO_DRIVE_PULL_UP);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("irq dio set_dio_function failed (%d)", status);
        return false;
    }
    status = lr20xx_system_set_dio_irq_cfg(&r, irq_dio, LR20XX_SYSTEM_IRQ_ALL_MASK);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("irq dio set_dio_irq_cfg failed (%d)", status);
        return false;
    }

    if (is_nicerf) {
        // NiceRF: DIO7 held high (module control line, per the shield dtsi).
        status = lr20xx_system_set_dio_function(&r, LR20XX_SYSTEM_DIO_7, LR20XX_SYSTEM_DIO_FUNC_GPIO_HIGH,
                                                LR20XX_SYSTEM_DIO_DRIVE_PULL_UP);
        if (status != LR20XX_STATUS_OK) {
            DEBUG("dio7 gpio_high failed (%d)", status);
            return false;
        }
    }

    // RF switch control. Without this the antenna is never connected.
    for (FwSizeType i = 0; i < rf_switch_count; i++) {
        const RfSwitchCfg& sw = rf_switches[i];
        status = lr20xx_system_set_dio_function(&r, sw.dio, LR20XX_SYSTEM_DIO_FUNC_RF_SWITCH,
                                                LR20XX_SYSTEM_DIO_DRIVE_AUTO);
        if (status != LR20XX_STATUS_OK) {
            DEBUG("dio%d set_dio_function failed (%d)", sw.dio, status);
            return false;
        }
        status = lr20xx_system_set_dio_rf_switch_cfg(&r, sw.dio, sw.cfg);
        if (status != LR20XX_STATUS_OK) {
            DEBUG("dio%d set_dio_rf_switch_cfg failed (%d)", sw.dio, status);
            return false;
        }
    }

    // NOTE: no explicit front-end calibration here. It proved unnecessary on
    // this hardware once the DIO map and the STDBY_XOSC fallback were right
    // (bench-verified with the block removed); the chip handles the needed
    // calibration when RF operations start. Revisit during RX sensitivity
    // characterisation.

    // 1024-byte TX / RX FIFOs (1020 / 1000 B usable) instead of the default
    // 256-byte ones, as USP v1.2.1 ral_lr20xx_init does: two register writes
    // each, valid with the patch RAM loaded above. Kept until reset or sleep
    // (no sleep here, so no retention slots). The FLRC burst needs it to
    // hold the next frame behind the one on the air.
    status = lr20xx_radio_fifo_configure_1024_byte_tx_fifo(&r);
    if (status == LR20XX_STATUS_OK) {
        status = lr20xx_radio_fifo_configure_1024_byte_rx_fifo(&r);
    }
    if (status != LR20XX_STATUS_OK) {
        DEBUG("radio %d 1024-byte FIFO cfg failed (%d)", static_cast<int>(idx), status);
        return false;
    }

    DEBUG("radio %d init OK", static_cast<int>(idx));
    return true;
}

}  // namespace LR2021
