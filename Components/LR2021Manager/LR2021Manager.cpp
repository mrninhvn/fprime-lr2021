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

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

// The C HAL bridge (LR2021Hal.cpp) reaches the component through the
// driver's context pointer: every lr20xx_* call passes &m_radio[i], and the
// slot carries the back-pointer plus the radio index.
LR2021Manager ::LR2021Manager(const char* const compName) : LR2021ManagerComponentBase(compName) {
    for (FwIndexType i = 0; i < NUM_RADIOS; i++) {
        this->m_radio[i].mgr = this;
        this->m_radio[i].idx = i;
    }
}

LR2021Manager ::~LR2021Manager() {}

// ----------------------------------------------------------------------
// Helpers used by the HAL bridge. The hardware ports are arrays indexed by
// the radio index, so dispatch is a direct port-number selection.
// ----------------------------------------------------------------------

bool LR2021Manager ::spiTransfer(FwIndexType idx, const U8* tx, U8* rx, U16 len) {
    FW_ASSERT(tx != nullptr);
    FW_ASSERT(rx != nullptr);
    FW_ASSERT(len > 0, static_cast<FwAssertArgType>(len));
    FW_ASSERT((idx >= 0) && (idx < NUM_RADIOS), static_cast<FwAssertArgType>(idx));

    if (!this->isConnected_spiWriteRead_OutputPort(idx)) {
        return false;
    }

    Fw::Buffer writeBuffer(const_cast<U8*>(tx), len);
    Fw::Buffer readBuffer(rx, len);
    Drv::SpiStatus status = this->spiWriteRead_out(idx, writeBuffer, readBuffer);
    if (status != Drv::SpiStatus::SPI_OK) {
        Fw::LogStringArg msg;
        msg.format("SPI transfer failed (radio=%d, status=%d, len=%u)", static_cast<int>(idx),
                   static_cast<int>(status.e), len);
        this->log_DIAGNOSTIC_LR2021(msg);
        return false;
    }
    return true;
}

void LR2021Manager ::halReset(FwIndexType idx) {
    FW_ASSERT((idx >= 0) && (idx < NUM_RADIOS), static_cast<FwAssertArgType>(idx));
    if (!this->isConnected_resetGpioWrite_OutputPort(idx)) {
        return;
    }
    // NRESET is active low and the GPIO is declared GPIO_ACTIVE_LOW, so the
    // F Prime logic level is the *logical* reset state (the driver inverts it):
    //   Fw::Logic::HIGH -> reset asserted (line driven low)
    //   Fw::Logic::LOW  -> reset released (line driven high)
    this->resetGpioWrite_out(idx, Fw::Logic::HIGH);  // assert reset
    this->delayMs(5);
    this->resetGpioWrite_out(idx, Fw::Logic::LOW);   // release reset
    this->delayMs(10);
    // Wait until the radio finishes its start-up sequence.
    (void)this->waitOnBusy(idx);
}

bool LR2021Manager ::halWakeup(FwIndexType idx) {
    // A NSS falling edge wakes the radio; issue a short dummy transfer to
    // generate one, then wait for BUSY to fall.
    U8 dummy = 0;
    (void)this->spiTransfer(idx, &dummy, &dummy, 1);
    return this->waitOnBusy(idx);
}

bool LR2021Manager ::waitOnBusy(FwIndexType idx, U32 timeout_us) {
    FW_ASSERT((idx >= 0) && (idx < NUM_RADIOS), static_cast<FwAssertArgType>(idx));
    // Without a BUSY line connected we cannot poll; assume the radio is ready.
    if (!this->isConnected_busyGpioRead_OutputPort(idx)) {
        return true;
    }

    const U32 step_us = 100;
    U32 elapsed_us = 0;
    Fw::Logic state = Fw::Logic::HIGH;
    do {
        Drv::GpioStatus status = this->busyGpioRead_out(idx, state);
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
    msg.format("radio %d BUSY stuck high; not ready (timeout %u us)", static_cast<int>(idx), timeout_us);
    this->log_DIAGNOSTIC_LR2021(msg);
    return false;
}

bool LR2021Manager ::irqPending(FwIndexType idx) {
    FW_ASSERT((idx >= 0) && (idx < NUM_RADIOS), static_cast<FwAssertArgType>(idx));
    // Without an IRQ line connected the caller must poll over SPI.
    if (!this->isConnected_irqGpioRead_OutputPort(idx)) {
        return true;
    }
    Fw::Logic state = Fw::Logic::LOW;
    if (this->irqGpioRead_out(idx, state) != Drv::GpioStatus::OP_OK) {
        return true;  // read failed: fall back to the SPI poll
    }
    return state == Fw::Logic::HIGH;
}

void LR2021Manager ::chipVersion(FwIndexType idx) {
    FW_ASSERT((idx >= 0) && (idx < NUM_RADIOS), static_cast<FwAssertArgType>(idx));
    lr20xx_system_version_t version = {0, 0};
    lr20xx_status_t status = lr20xx_system_get_version(&this->m_radio[idx], &version);
    if (status == LR20XX_STATUS_OK) {
        Fw::LogStringArg msg;
        msg.format("radio %d chip version %X.%X", static_cast<int>(idx), version.major, version.minor);
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
// Radio bring-up / selection
// ----------------------------------------------------------------------

void LR2021Manager ::setModuleType(FwIndexType idx, ModuleType type) {
    FW_ASSERT((idx >= 0) && (idx < NUM_RADIOS), static_cast<FwAssertArgType>(idx));
    this->m_radio[idx].moduleType = type;
}

void LR2021Manager ::setCcsdsRole(FwIndexType idx, CcsdsRole role) {
    FW_ASSERT((idx >= 0) && (idx < NUM_RADIOS), static_cast<FwAssertArgType>(idx));
    this->m_radio[idx].ccsdsRole = role;
}

void LR2021Manager ::setTxFreq(FwIndexType idx, U32 tx_freq_hz) {
    FW_ASSERT((idx >= 0) && (idx < NUM_RADIOS), static_cast<FwAssertArgType>(idx));
    this->m_radio[idx].txFreqHz = tx_freq_hz;
}

void LR2021Manager ::setRxFreq(FwIndexType idx, U32 rx_freq_hz) {
    FW_ASSERT((idx >= 0) && (idx < NUM_RADIOS), static_cast<FwAssertArgType>(idx));
    this->m_radio[idx].rxFreqHz = rx_freq_hz;
}

void LR2021Manager ::setTxRoute(FwIndexType comQueueIndex, FwIndexType radioIdx) {
    FW_ASSERT((comQueueIndex >= 0) && (comQueueIndex < TX_ROUTE_TABLE_SIZE),
              static_cast<FwAssertArgType>(comQueueIndex));
    // radioIdx in [0, NUM_RADIOS) selects a radio; UART_RADIO (== NUM_RADIOS)
    // selects the byte-stream (UART) target.
    FW_ASSERT((radioIdx >= 0) && (radioIdx <= UART_RADIO), static_cast<FwAssertArgType>(radioIdx));
    this->m_txRoute[comQueueIndex] = radioIdx;
}

FwIndexType LR2021Manager ::routeTx(const ComCfg::FrameContext& context) const {
    const FwIndexType queueIndex = context.get_comQueueIndex();
    if ((queueIndex >= 0) && (queueIndex < TX_ROUTE_TABLE_SIZE)) {
        return this->m_txRoute[queueIndex];
    }
    return 0;
}

void LR2021Manager ::txComplete(RadioSlot& r, Fw::Success status) {
    if (r.workingBuffer.isValid()) {
        Fw::Buffer buffer = r.workingBuffer;
        ComCfg::FrameContext context = r.workingContext;
        r.workingBuffer = Fw::Buffer();
        r.workingContext = ComCfg::FrameContext();
        if (this->isConnected_dataReturnOut_OutputPort(0)) {
            this->dataReturnOut_out(0, buffer, context);
        }
        if (this->isConnected_comStatusOut_OutputPort(0)) {
            this->comStatusOut_out(0, status);
        }
        // A failed frame closes the com flow: the framer stack stops after a
        // FAILURE status, so the next successful setMode() must re-open it.
        this->m_comOpen = (status == Fw::Success::SUCCESS);
    }
}

bool LR2021Manager ::setMode(FwIndexType idx, RadioMode mode, U32 freq_hz, I8 power_dbm) {
    FW_ASSERT((idx >= 0) && (idx < NUM_RADIOS), static_cast<FwAssertArgType>(idx));
    RadioSlot& r = this->m_radio[idx];

    // A mode switch aborts any in-flight TX: drop the frame, returning its
    // buffer (if it owns one) to the framer.
    if (r.txInFlight && r.workingBuffer.isValid()) {
        this->txComplete(r, Fw::Success::FAILURE);
    }
    r.txInFlight = false;

    bool ok = false;
    switch (mode) {
        case RadioMode::FLRC:
            ok = this->flrcInit(r, freq_hz, power_dbm) && this->flrcRx(r, 0);
            break;
        case RadioMode::FSK:
            ok = this->fskInit(r, freq_hz, power_dbm) && this->fskRx(r, 0);
            break;
        default:
            break;
    }
    if (ok) {
        r.freqHz = freq_hz;
        r.powerDbm = power_dbm;
        // Open (or re-open after a failure) the downlink flow. Emitted only
        // once: the framer stack must see a single initial comStatus, each
        // further one is granted per completed TX in txComplete().
        if (!this->m_comOpen && this->isConnected_comStatusOut_OutputPort(0)) {
            Fw::Success success = Fw::Success::SUCCESS;
            this->comStatusOut_out(0, success);
            this->m_comOpen = true;
        }
    }
    return ok;
}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

void LR2021Manager ::run_handler(FwIndexType portNum, U32 context) {
    // Poll radio IRQs (TX done / RX done / errors) on every radio.
    for (FwIndexType i = 0; i < NUM_RADIOS; i++) {
        RadioSlot& r = this->m_radio[i];
        switch (r.mode) {
            case RadioMode::FLRC:
                this->flrcService(r);
                break;
            case RadioMode::FSK:
                this->fskService(r);
                break;
            default:
                break;
        }
    }
}

void LR2021Manager ::dataIn_handler(FwIndexType portNum,
                                    Fw::Buffer& data,
                                    const ComCfg::FrameContext& context) {
    // Downlink routing: the frame context's comQueueIndex selects the target
    // through the route table (telemetry / events -> UHF, file -> S-band, or
    // the byte-stream UART target).
    const FwIndexType idx = this->routeTx(context);

    // UART target: synchronous send straight to the byte-stream driver, then
    // return the buffer and grant the next comStatus credit (like ComStub's
    // synchronous path). No radio slot / working buffer involved.
    if (idx == UART_RADIO) {
        Fw::Success comSuccess = Fw::Success::FAILURE;
        if (data.isValid() && this->isConnected_drvSendOut_OutputPort(0)) {
            Drv::ByteStreamStatus st = Drv::ByteStreamStatus::SEND_RETRY;
            for (FwIndexType i = 0; (st == Drv::ByteStreamStatus::SEND_RETRY) && (i < UART_RETRY_LIMIT); i++) {
                st = this->drvSendOut_out(0, data);
            }
            comSuccess = (st == Drv::ByteStreamStatus::OP_OK) ? Fw::Success::SUCCESS : Fw::Success::FAILURE;
        }
        if (this->isConnected_dataReturnOut_OutputPort(0)) {
            this->dataReturnOut_out(0, data, context);
        }
        if (this->isConnected_comStatusOut_OutputPort(0)) {
            this->comStatusOut_out(0, comSuccess);
        }
        this->m_comOpen = (comSuccess == Fw::Success::SUCCESS);
        return;
    }

    RadioSlot& r = this->m_radio[idx];

    // The framer stack sends one frame per comStatus credit, so the routed
    // radio is normally idle here; any local failure drops the frame.
    bool ok = false;
    if (data.isValid() && (data.getSize() > 0) && !r.txInFlight) {
        r.workingBuffer = data;
        r.workingContext = context;
        switch (r.mode) {
            case RadioMode::FLRC:
                ok = this->flrcTx(r, r.workingBuffer.getData(), r.workingBuffer.getSize());
                break;
            case RadioMode::FSK:
                ok = this->fskTx(r, r.workingBuffer.getData(), r.workingBuffer.getSize());
                break;
            default:
                break;
        }
        if (!ok) {
            r.workingBuffer = Fw::Buffer();
            r.workingContext = ComCfg::FrameContext();
        }
    }
    if (!ok) {
        DEBUG("radio %d TX frame dropped", static_cast<int>(idx));
        this->log_WARNING_HI_TxFrameDropped(static_cast<U8>(idx));
        if (this->isConnected_dataReturnOut_OutputPort(0)) {
            this->dataReturnOut_out(0, data, context);
        }
        if (this->isConnected_comStatusOut_OutputPort(0)) {
            Fw::Success failure = Fw::Success::FAILURE;
            this->comStatusOut_out(0, failure);
        }
        // Closed until a successful setMode() re-opens the flow.
        this->m_comOpen = false;
    }
}

void LR2021Manager ::dataReturnIn_handler(FwIndexType portNum,
                                          Fw::Buffer& data,
                                          const ComCfg::FrameContext& context) {
    // Frees both radio-RX buffers and UART-RX buffers: all come from the same
    // buffer manager (our allocate port and the byte-stream driver's allocate
    // port point at it), so a plain deallocate returns them correctly.
    this->deallocate_out(0, data);
}

void LR2021Manager ::drvConnected_handler(FwIndexType portNum) {
    // Intentionally empty: the single initial comStatus credit is emitted by
    // setMode(). See the declaration comment.
}

void LR2021Manager ::drvReceiveIn_handler(FwIndexType portNum,
                                          Fw::Buffer& recvBuffer,
                                          const Drv::ByteStreamStatus& recvStatus) {
    if (recvStatus != Drv::ByteStreamStatus::OP_OK) {
        // Receive failed: free the driver's buffer, nothing to forward.
        this->deallocate_out(0, recvBuffer);
        return;
    }
    // Forward uplink bytes to the frame accumulator with an empty context
    // (the byte-stream carries raw framed bytes, like a radio RX).
    ComCfg::FrameContext emptyContext;
    this->dataOut_out(0, recvBuffer, emptyContext);
}

// ----------------------------------------------------------------------
// Handler implementations for commands
// ----------------------------------------------------------------------

void LR2021Manager ::RESET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U8 radio) {
    if (radio >= NUM_RADIOS) {
        this->log_WARNING_LO_BadRadioIndex(radio);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
        return;
    }
    // radioInit() resets the chip and re-applies the chip-level config
    // (regulator, clocks, DIO routing / RF switches).
    RadioSlot& r = this->m_radio[radio];
    bool ok = this->radioInit(radio);
    if (ok && (r.mode != RadioMode::NONE)) {
        // Restore the previously active mode so the link comes back up.
        ok = this->setMode(radio, r.mode, r.freqHz, r.powerDbm);
    }
    this->cmdResponse_out(opCode, cmdSeq, ok ? Fw::CmdResponse::OK : Fw::CmdResponse::EXECUTION_ERROR);
}

void LR2021Manager ::SET_MODE_cmdHandler(FwOpcodeType opCode,
                                         U32 cmdSeq,
                                         U8 radio,
                                         LR2021Manager_Mode mode,
                                         U32 freq_hz,
                                         I8 power_dbm) {
    if (radio >= NUM_RADIOS) {
        this->log_WARNING_LO_BadRadioIndex(radio);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
        return;
    }
    RadioMode target = RadioMode::NONE;
    switch (mode.e) {
        case LR2021Manager_Mode::FLRC:
            target = RadioMode::FLRC;
            break;
        case LR2021Manager_Mode::FSK:
            target = RadioMode::FSK;
            break;
        default:
            break;
    }

    const bool ok = this->setMode(radio, target, freq_hz, power_dbm);
    if (ok) {
        this->log_ACTIVITY_HI_ModeSet(radio, mode);
    }
    this->cmdResponse_out(opCode, cmdSeq, ok ? Fw::CmdResponse::OK : Fw::CmdResponse::EXECUTION_ERROR);
}

}  // namespace LR2021
