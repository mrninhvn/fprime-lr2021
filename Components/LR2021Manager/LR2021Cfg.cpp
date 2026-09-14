// ======================================================================
// \title  LR2021Cfg.cpp
// \author ninhdh4
// \brief  LR2021 configuration operations for the LR2021Manager component,
//         built on the lr20xx_driver C API.
//
// Flow:
//   radioInit(idx) - reset, regulator, clocks, DIO / RF-switch map for the
//                    module type populated on that radio slot
// ======================================================================

#include "Components/LR2021Manager/LR2021Manager.hpp"
#include "Components/LR2021Manager/LR2021Cfg.hpp"

extern "C" {
#include "lr20xx_system.h"
#include "lr20xx_radio_common.h"
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
    // "initial calibration is enough". Calibrate returns the chip to STDBY_RC.
    status = lr20xx_system_calibrate(&r, LR20XX_SYSTEM_CALIB_MU_MASK);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("calibrate (MU ADC) failed (%d)", status);
        return false;
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

    DEBUG("radio %d init OK", static_cast<int>(idx));
    return true;
}

}  // namespace LR2021
