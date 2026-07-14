// ======================================================================
// \title  LR2021Cfg.cpp
// \author ninhdh4
// \brief  LR2021 configuration operations for the LR2021Manager component,
//         built on the lr20xx_driver C API.
//
// Flow:
//   chipInit()    - HF PA / RX path
// ======================================================================

#include "Components/LR2021Manager/LR2021Manager.hpp"
#include "Components/LR2021Manager/LR2021Cfg.hpp"

extern "C" {
#include "lr20xx_system.h"
}

namespace LR2021 {

bool LR2021Manager ::radioInit() {
    if (this->isConnected_powerGpioWrite_OutputPort(0)) {
        this->powerGpioWrite_out(0, Fw::Logic::HIGH); 
    }

    lr20xx_status_t status;

    status = lr20xx_system_reset(this);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("reset failed (%d)", status);
        return false;
    }

    // System configuration mirroring the validated USP overlay for this
    // radio module: DC-DC regulator, internal 32 kHz RC, TCXO at 1.8 V.
    status = lr20xx_system_set_reg_mode(this, LR20XX_SYSTEM_REG_MODE_DCDC);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_reg_mode failed (%d)", status);
        return false;
    }

    status = lr20xx_system_cfg_lfclk(this, LR20XX_SYSTEM_LFCLK_RC);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("cfg_lfclk failed (%d)", status);
        return false;
    }

    status = lr20xx_system_set_tcxo_mode(this, LR20XX_SYSTEM_TCXO_CTRL_1_8V, 0);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("set_tcxo_mode failed (%d)", status);
        return false;
    }

    // Route radio IRQs to DIO7 (wired to the MCU's IRQ input line).
    status = lr20xx_system_set_dio_function(this, LR20XX_SYSTEM_DIO_7, LR20XX_SYSTEM_DIO_FUNC_IRQ,
                                            LR20XX_SYSTEM_DIO_DRIVE_NONE);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("dio7 set_dio_function failed (%d)", status);
        return false;
    }
    status = lr20xx_system_set_dio_irq_cfg(this, LR20XX_SYSTEM_DIO_7, LR20XX_SYSTEM_IRQ_ALL_MASK);
    if (status != LR20XX_STATUS_OK) {
        DEBUG("dio7 set_dio_irq_cfg failed (%d)", status);
        return false;
    }

    // RF switch control: DIO8/9 steer the LF path, DIO10/11 the HF path
    // (per the validated USP overlay: dio8=LF_RX, dio9=LF_TX, dio10=HF_RX,
    // dio11=HF_TX). Without this the antenna is never connected.
    const struct {
        lr20xx_system_dio_t dio;
        lr20xx_system_dio_rf_switch_cfg_t cfg;
    } rf_switches[] = {
        {LR20XX_SYSTEM_DIO_8, LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_RX_LF},
        {LR20XX_SYSTEM_DIO_9, LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_TX_LF},
        {LR20XX_SYSTEM_DIO_10, LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_RX_HF},
        {LR20XX_SYSTEM_DIO_11, LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_TX_HF},
    };
    for (const auto& sw : rf_switches) {
        status = lr20xx_system_set_dio_function(this, sw.dio, LR20XX_SYSTEM_DIO_FUNC_RF_SWITCH,
                                                LR20XX_SYSTEM_DIO_DRIVE_AUTO);
        if (status != LR20XX_STATUS_OK) {
            DEBUG("dio%d set_dio_function failed (%d)", sw.dio, status);
            return false;
        }
        status = lr20xx_system_set_dio_rf_switch_cfg(this, sw.dio, sw.cfg);
        if (status != LR20XX_STATUS_OK) {
            DEBUG("dio%d set_dio_rf_switch_cfg failed (%d)", sw.dio, status);
            return false;
        }
    }

    DEBUG("OK");
    return true;
}

}  // namespace LR2021
