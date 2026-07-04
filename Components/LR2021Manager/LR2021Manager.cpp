// ======================================================================
// \title  LR2021Manager.cpp
// \author v.ninhdh4
// \brief  cpp file for LR2021Manager component implementation class
// ======================================================================

#include "Components/LR2021Manager/LR2021Manager.hpp"

#include <Fw/Types/Assert.hpp>
#include <cstdio>

extern "C" {
#include "lr20xx_system.h"
}

namespace LR2021 {

// Global instance pointer, consumed by the C HAL bridge (LR2021Hal.cpp)
LR2021Manager* g_lr2021_manager = nullptr;

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

LR2021Manager ::LR2021Manager(const char* const compName) : LR2021ManagerComponentBase(compName) {
    g_lr2021_manager = this;
}

LR2021Manager ::~LR2021Manager() {}

// ----------------------------------------------------------------------
// Helpers used by the HAL bridge
// ----------------------------------------------------------------------

bool LR2021Manager ::spiTransfer(const U8* tx, U8* rx, U16 len) {
    FW_ASSERT(tx != nullptr);
    FW_ASSERT(rx != nullptr);
    FW_ASSERT(len > 0, static_cast<FwAssertArgType>(len));

    if (!this->isConnected_spiWriteRead_OutputPort(0)) {
        return false;
    }

    Fw::Buffer writeBuffer(const_cast<U8*>(tx), len);
    Fw::Buffer readBuffer(rx, len);
    Drv::SpiStatus status = this->spiWriteRead_out(0, writeBuffer, readBuffer);
    if (status != Drv::SpiStatus::SPI_OK) {
        Fw::LogStringArg msg;
        msg.format("SPI transfer failed (status=%d, len=%u)", static_cast<int>(status.e), len);
        this->log_DIAGNOSTIC_LR2021(msg);
        return false;
    }
    return true;
}

void LR2021Manager ::halReset() {
    if (!this->isConnected_resetGpioWrite_OutputPort(0)) {
        return;
    }
    // NRESET is active low and the GPIO is declared GPIO_ACTIVE_LOW, so the
    // F Prime logic level is the *logical* reset state (the driver inverts it):
    //   Fw::Logic::HIGH -> reset asserted (line driven low)
    //   Fw::Logic::LOW  -> reset released (line driven high)
    this->resetGpioWrite_out(0, Fw::Logic::HIGH);  // assert reset
    this->delayMs(5);
    this->resetGpioWrite_out(0, Fw::Logic::LOW);   // release reset
    this->delayMs(10);
    // Wait until the radio finishes its start-up sequence.
    (void)this->waitOnBusy();
}

bool LR2021Manager ::halWakeup() {
    // A NSS falling edge wakes the radio; issue a short dummy transfer to
    // generate one, then wait for BUSY to fall.
    U8 dummy = 0;
    (void)this->spiTransfer(&dummy, &dummy, 1);
    return this->waitOnBusy();
}

bool LR2021Manager ::waitOnBusy(U32 timeout_us) {
    // Without a BUSY line connected we cannot poll; assume the radio is ready.
    if (!this->isConnected_busyGpioRead_OutputPort(0)) {
        return true;
    }

    const U32 step_us = 100;
    U32 elapsed_us = 0;
    Fw::Logic state = Fw::Logic::HIGH;
    do {
        Drv::GpioStatus status = this->busyGpioRead_out(0, state);
        if (status != Drv::GpioStatus::OP_OK) {
            return false;
        }
        if (state == Fw::Logic::LOW) {
            return true;
        }
        this->delayUs(step_us);
        elapsed_us += step_us;
    } while (elapsed_us < timeout_us);

    // Timed out: the radio never lowered BUSY. Usually means it is held in
    // reset, unpowered, or the BUSY line is miswired.
    Fw::LogStringArg msg;
    msg.format("BUSY stuck high; radio not ready (timeout %u us)", timeout_us);
    this->log_DIAGNOSTIC_LR2021(msg);
    return false;
}

void LR2021Manager ::chipVersion() {
    lr20xx_system_version_t version = {0, 0};
    lr20xx_status_t status = lr20xx_system_get_version(this, &version);
    if (status == LR20XX_STATUS_OK) {
        Fw::LogStringArg msg;
        msg.format("Chip Version %X.%X", version.major, version.minor);
        this->log_DIAGNOSTIC_LR2021(msg);
    }
}

void LR2021Manager ::logDebug(const Fw::LogStringArg& msg) {
    this->log_DIAGNOSTIC_LR2021(msg);
}

void LR2021Manager ::logHex(const char* tag, const U8* data, U16 len) {
    char buf[128];
    int off = snprintf(buf, sizeof(buf), "%s:", tag);
    for (U16 i = 0; (i < len) && (off > 0) && (off < static_cast<int>(sizeof(buf) - 4)); i++) {
        off += snprintf(buf + off, static_cast<size_t>(sizeof(buf) - off), " %02X", data[i]);
    }
    Fw::LogStringArg msg(buf);
    this->log_DIAGNOSTIC_LR2021(msg);
}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

void LR2021Manager ::run_handler(FwIndexType portNum, U32 context) {
    // Reserved for periodic IRQ / RX servicing. No-op for now.
}

// ----------------------------------------------------------------------
// Handler implementations for commands
// ----------------------------------------------------------------------

void LR2021Manager ::RESET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    lr20xx_status_t status = lr20xx_system_reset(this);
    if (status == LR20XX_STATUS_OK) {
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
    } else {
        this->log_WARNING_HI_HalError(static_cast<I32>(status));
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
    }
}

}  // namespace LR2021
