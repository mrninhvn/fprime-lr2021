// ======================================================================
// \title  LR2021Manager.cpp
// \author v.ninhdh4
// \brief  cpp file for LR2021Manager component implementation class
// ======================================================================

#include "Components/LR2021Manager/LR2021Manager.hpp"

#include <Fw/Types/Assert.hpp>
#include <cstdio>
#include <cstring>

extern "C" {
#include "lr20xx_radio_common.h"
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

void LR2021Manager ::setRxSink(RxSink sink) {
    this->m_rxSink = sink;
}

void LR2021Manager ::forwardRxPacket(const U8* data, U16 len) {
    if ((data == nullptr) || (len == 0)) {
        return;
    }
    Fw::Buffer recv_buffer = this->allocate_out(0, len);
    if (recv_buffer.getData() == nullptr) {
        // Allocator gave back an empty buffer: return it and drop the packet.
        this->deallocate_out(0, recv_buffer);
        return;
    }
    std::memcpy(recv_buffer.getData(), data, len);
    recv_buffer.setSize(len);

    if (this->m_rxSink == RxSink::UART) {
        // Ground relay: push the raw packet bytes straight out the byte-stream
        // (UART) driver. The send copies synchronously, so we keep ownership of
        // the buffer the whole time and free it right after.
        if (this->isConnected_drvSendOut_OutputPort(0)) {
            Drv::ByteStreamStatus st = Drv::ByteStreamStatus::SEND_RETRY;
            for (FwIndexType i = 0; (st == Drv::ByteStreamStatus::SEND_RETRY) && (i < UART_RETRY_LIMIT); i++) {
                st = this->drvSendOut_out(0, recv_buffer);
            }
        }
        this->deallocate_out(0, recv_buffer);
    } else {
        // Flight: hand the packet to the Svc.Com dataOut port (frame
        // accumulator). Ownership passes downstream and comes back on
        // dataReturnIn. If unconnected, free it here so it is not leaked.
        if (this->isConnected_dataOut_OutputPort(0)) {
            ComCfg::FrameContext emptyContext;
            this->dataOut_out(0, recv_buffer, emptyContext);
        } else {
            this->deallocate_out(0, recv_buffer);
        }
    }
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
    // buffer (if it owns one) to the framer. It also cancels any pending
    // timed TX test on this slot (setMode is how run() restores RX after a
    // test, and an explicit command should override a running test too).
    if (r.txInFlight && r.workingBuffer.isValid()) {
        this->txComplete(r, Fw::Success::FAILURE);
    }
    r.txInFlight = false;
    r.txTesting = false;

    bool ok = false;
    switch (mode) {
        case RadioMode::FLRC:
            ok = this->flrcInit(r, freq_hz, power_dbm) && this->flrcRx(r, 0);
            break;
        case RadioMode::FSK:
            ok = this->fskInit(r, freq_hz, power_dbm) && this->fskRx(r, 0);
            break;
        case RadioMode::CW:
            // Reuse fskInit for band/PA-path selection, frequency and TX
            // power (pkt_type/modulation params are irrelevant to a raw
            // carrier), then key the carrier with set_tx_test_mode().
            // NOTE 1: set_tx_test_mode() itself starts transmitting (it is the
            //   whole of ral_lr20xx_set_tx_cw()); do NOT follow it with
            //   set_tx() -- a normal TX entry after it cancels the test
            //   carrier (verified on HW: adding set_tx() stopped CW).
            // NOTE 2: fskInit() leaves the chip in STANDBY_RC, which powers the
            //   (NiceRF) TCXO down; set_tx_test_mode() does not ramp it up on
            //   its own, so the carrier never radiates. Enter STANDBY_XOSC
            //   first so the HF clock is running (matches the proven sequence).
            ok = this->fskInit(r, freq_hz, power_dbm);
            if (ok) {
                const lr20xx_status_t st = lr20xx_system_set_standby_mode(&r, LR20XX_SYSTEM_STANDBY_MODE_XOSC);
                ok = (st == LR20XX_STATUS_OK);
            }
            if (ok) {
                const lr20xx_status_t st =
                    lr20xx_radio_common_set_tx_test_mode(&r, LR20XX_RADIO_COMMON_TX_TEST_MODE_CONTINUOUS_WAVE);
                ok = (st == LR20XX_STATUS_OK);
            }
            if (ok) {
                // fskInit() stamps r.mode = FSK; override so run()/dataIn's
                // per-mode switches fall through their default (no service,
                // no TX) while the carrier is up.
                r.mode = RadioMode::CW;
            }
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

bool LR2021Manager ::txCw(FwIndexType idx, U32 freq_hz, I8 power_dbm) {
    // Thin wrapper: all the actual CW keying (band/PA setup via fskInit,
    // STANDBY_XOSC, then set_tx_test_mode) lives in setMode()'s RadioMode::CW
    // case, so there is exactly one path that can key a carrier and exactly
    // one that stamps r.mode = CW.
    return this->setMode(idx, RadioMode::CW, freq_hz, power_dbm);
}

bool LR2021Manager ::txTest(FwIndexType idx, RadioMode mode, U32 freq_hz, I8 power_dbm, U32 duration_s) {
    FW_ASSERT((idx >= 0) && (idx < NUM_RADIOS), static_cast<FwAssertArgType>(idx));
    if ((mode != RadioMode::FSK) && (mode != RadioMode::FLRC)) {
        return false;
    }
    RadioSlot& r = this->m_radio[idx];

    // A test switch aborts any in-flight TX, same as setMode().
    if (r.txInFlight && r.workingBuffer.isValid()) {
        this->txComplete(r, Fw::Success::FAILURE);
    }
    r.txInFlight = false;

    // Band/PA-path, frequency and TX power via the requested packet engine's
    // own init (packet/modulation params are irrelevant to a raw PRBS9
    // pattern, but the init also selects the right RX/PA path for the band).
    bool ok = (mode == RadioMode::FLRC) ? this->flrcInit(r, freq_hz, power_dbm)
                                        : this->fskInit(r, freq_hz, power_dbm);
    if (ok) {
        // fskInit/flrcInit leave the chip in STANDBY_RC (TCXO off); enter
        // STANDBY_XOSC so the HF clock runs, or the test carrier never
        // radiates (same requirement as the CW case in setMode()).
        const lr20xx_status_t st = lr20xx_system_set_standby_mode(&r, LR20XX_SYSTEM_STANDBY_MODE_XOSC);
        ok = (st == LR20XX_STATUS_OK);
    }
    if (ok) {
        // set_tx_test_mode() itself keys the modulated carrier (same as
        // set_tx_cw for the CW mode); it must NOT be followed by set_tx(),
        // which would cancel the test pattern (verified on HW).
        const lr20xx_status_t st =
            lr20xx_radio_common_set_tx_test_mode(&r, LR20XX_RADIO_COMMON_TX_TEST_MODE_PRBS9);
        ok = (st == LR20XX_STATUS_OK);
    }
    if (!ok) {
        DEBUG("radio %d TX test failed to key", static_cast<int>(idx));
        // Best-effort: still try to bring the radio back to a known-good
        // (RX) state rather than leaving it in whatever partial state the
        // failed sequence left it in.
        (void)this->setMode(idx, mode, freq_hz, power_dbm);
        return false;
    }

    DEBUG("radio %d PRBS9 TX test: %u Hz, %d dBm, %u s", static_cast<int>(idx),
          static_cast<unsigned>(freq_hz), static_cast<int>(power_dbm), static_cast<unsigned>(duration_s));

    // Test mode has no hardware timeout, and we must NOT block the thread to
    // time it (that stalls the message queue -> async dataIn frames pile up
    // -> queue-full assert -> FATAL). Instead schedule a non-blocking stop:
    // record the deadline and the mode/freq/power to restore, and let run()
    // stop the carrier (via setMode) once the deadline passes. Park r.mode at
    // NONE meanwhile so run()'s service switch and dataIn's TX switch both
    // no-op/drop for this slot while the carrier is up (the other radio keeps
    // running normally).
    r.txTesting = true;
    r.txTestMode = mode;
    r.txTestFreqHz = freq_hz;
    r.txTestPowerDbm = power_dbm;
    Fw::Time end = this->getTime();
    end.add(duration_s, 0);
    r.txTestEnd = end;
    r.mode = RadioMode::NONE;
    return true;
}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

void LR2021Manager ::run_handler(FwIndexType portNum, U32 context) {
    // Poll radio IRQs (TX done / RX done / errors) on every radio.
    for (FwIndexType i = 0; i < NUM_RADIOS; i++) {
        RadioSlot& r = this->m_radio[i];

        // Non-blocking timed TX test: while a test carrier is up on this slot
        // (r.mode parked at NONE), stop servicing it and just watch the clock.
        // When the deadline passes, restore normal continuous RX (setMode also
        // clears r.txTesting), which stops the carrier.
        if (r.txTesting) {
            if (this->getTime() >= r.txTestEnd) {
                const bool ok = this->setMode(i, r.txTestMode, r.txTestFreqHz, r.txTestPowerDbm);
                if (ok) {
                    this->log_ACTIVITY_HI_TxTestDone(static_cast<U8>(i));
                } else {
                    this->log_WARNING_HI_TxTestError(static_cast<U8>(i));
                }
            }
            continue;
        }

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

    // Drive the non-blocking BER test (if one is running): the per-slot
    // service above already cleared txInFlight / tallied any RX_DONE this
    // tick, so berDrive() can send the next packet or finalize now.
    this->berDrive();
}

// ----------------------------------------------------------------------
// PolyDb
// ----------------------------------------------------------------------

bool LR2021Manager ::readDieTemp(FwIndexType idx, I8& tempC) {
    RadioSlot& r = this->m_radio[idx];

    // The Measure Unit ADC only converts in STDBY_XOSC (it returns a stale 0
    // during TX/RX), and must have been calibrated at boot (radioInit calls
    // lr20xx_system_calibrate with LR20XX_SYSTEM_CALIB_MU_MASK). Callers read
    // this on the TX->RX turnaround, while the radio is briefly in STDBY_XOSC.
    uint16_t raw = 0;
    const lr20xx_status_t status = lr20xx_system_get_temp(
        &r, LR20XX_SYSTEM_VALUE_FORMAT_RAW, LR20XX_SYSTEM_MEAS_RES_12_BITS, LR20XX_SYSTEM_TEMP_SRC_VBE, &raw);
    if (status != LR20XX_STATUS_OK) {
        return false;
    }
    // DEBUG("readDieTemp raw=%u", raw);

    // A raw of 0 means no valid conversion happened (MU ADC not in STDBY_XOSC):
    // reject it so we never publish a bogus temperature into PolyDb.
    if (raw == 0) {
        return false;
    }

    // Vana (typ. 1.35 V), Vbe25 (typ. 0.7295 V), VbeSlope (typ. -1.7 mV/degC).
    // See lr20xx_system_get_temp()'s docstring for the derivation. NOTE: the
    // driver's get_temp right-shifts the 13-bit measurement by 3 (unlike
    // get_vbat, which does not), so the docstring's /8192 becomes /1024 here.
    // Verified against get_vbat: raw 542 -> ~34 C, sane for an operating die.
    constexpr F32 VANA_V = 1.35f;
    constexpr F32 VBE25_V = 0.7295f;
    constexpr F32 VBE_SLOPE_MV_PER_C = -1.7f;
    F32 tempF = (static_cast<F32>(raw) / 1024.0f * VANA_V - VBE25_V) * (1000.0f / VBE_SLOPE_MV_PER_C) + 25.0f;
    // Clamp to the I8 telemetry range so an out-of-range reading can't wrap.
    if (tempF > 127.0f) {
        tempF = 127.0f;
    } else if (tempF < -128.0f) {
        tempF = -128.0f;
    }
    tempC = static_cast<I8>(tempF + (tempF >= 0.0f ? 0.5f : -0.5f));
    return true;
}

void LR2021Manager ::publishTempPoly(FwIndexType idx) {
    FW_ASSERT((idx >= 0) && (idx < NUM_RADIOS), static_cast<FwAssertArgType>(idx));
    if (!this->isConnected_setPoly_OutputPort(0)) {
        return;
    }

    // Read the die temperature. The caller must invoke this only while the
    // radio is in STDBY_XOSC (the MU ADC does not convert during TX/RX): that
    // is the case on the TX->RX turnaround, inside fskRx()/flrcRx() after the
    // set_standby(XOSC) step and before set_rx.
    I8 tempC = 0;
    if (!this->readDieTemp(idx, tempC)) {
        return;
    }

    // Each module reports to its own PolyDb entry.
    const Svc::PolyDbCfg::PolyDbEntry entry =
        (this->m_radio[idx].moduleType == ModuleType::NICERF)
            ? Svc::PolyDbCfg::PolyDbEntry::POLYDB_ENTRY_OBC_NICERF_Temperature
            : Svc::PolyDbCfg::PolyDbEntry::POLYDB_ENTRY_OBC_REYAX_Temperature;

    Svc::MeasurementStatus polyStatus = Svc::MeasurementStatus::OK;
    Fw::Time polyTime = this->getTime();
    Fw::PolyType polyValue = tempC;
    this->setPoly_out(0, entry, polyStatus, polyTime, polyValue);
    DEBUG("radio %d die temp %d C", static_cast<int>(idx), static_cast<int>(tempC));
}

void LR2021Manager ::sendRssiPoly(Svc::PolyDbCfg::PolyDbEntry entry, I16 rssiDbm) {
    if (!this->isConnected_setPoly_OutputPort(0)) {
        return;
    }

    Svc::MeasurementStatus polyStatus = Svc::MeasurementStatus::OK;
    Fw::Time polyTime = this->getTime();
    Fw::PolyType polyValue = static_cast<int8_t>(rssiDbm);
    this->setPoly_out(0, entry, polyStatus, polyTime, polyValue);
}

void LR2021Manager ::dataIn_handler(FwIndexType portNum,
                                    Fw::Buffer& data,
                                    const ComCfg::FrameContext& context) {
    // Downlink routing: the frame context's comQueueIndex selects the target
    // through the route table (telemetry / events -> UHF, file -> S-band, or
    // the byte-stream UART target).
    const FwIndexType idx = this->routeTx(context);

    // A running BER test owns both of its radios: drop any downlink frame
    // routed to one of them so the raw test traffic is not corrupted.
    if (this->m_ber.active && ((idx == this->m_ber.txRadio) || (idx == this->m_ber.rxRadio))) {
        this->log_WARNING_HI_TxFrameDropped(static_cast<U8>(idx));
        if (this->isConnected_dataReturnOut_OutputPort(0)) {
            this->dataReturnOut_out(0, data, context);
        }
        if (this->isConnected_comStatusOut_OutputPort(0)) {
            Fw::Success failure = Fw::Success::FAILURE;
            this->comStatusOut_out(0, failure);
        }
        this->m_comOpen = false;
        return;
    }

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
    // Forward uplink bytes to the frame accumulator with an empty context (the
    // byte-stream carries raw framed bytes, like a radio RX). The accumulator
    // and APID router downstream decide local-vs-radio routing.
    ComCfg::FrameContext emptyContext;
    this->dataOut_out(0, recvBuffer, emptyContext);
}

void LR2021Manager ::relayIn_handler(FwIndexType portNum, Fw::Buffer& data, const ComCfg::FrameContext& context) {
    // A complete frame routed to the radio by the APID router: transmit it
    // verbatim (fire-and-forget) and free the buffer. flrcTx/fskTx copy the
    // payload into the radio FIFO synchronously, so the buffer can be returned
    // to the (shared) buffer manager immediately.
    this->relayUplink(data);
    this->deallocate_out(0, data);
}

void LR2021Manager ::relayUplink(Fw::Buffer& data) {
    // Full route: all ground uplink goes to radio 0 for now.
    // TODO: parse the CCSDS primary-header APID and route to the radio
    // configured for that APID (per-APID route table), so multiple uplink
    // targets can share the host UART.
    const FwIndexType idx = 0;
    RadioSlot& r = this->m_radio[idx];

    // Drop the relay while a BER test owns this radio (it is mid raw test TX/RX).
    if (this->m_ber.active && ((idx == this->m_ber.txRadio) || (idx == this->m_ber.rxRadio))) {
        DEBUG("radio %d uplink relay dropped (BER test active)", static_cast<int>(idx));
        return;
    }

    // No working buffer / comStatus crediting for a relay: flrcTx/fskTx copy
    // the payload into the radio FIFO synchronously, so the source buffer is
    // freed by the caller right after. TX_DONE in the service loop then finds
    // an empty working buffer and returns the radio to RX.
    const U16 len = data.isValid() ? static_cast<U16>(data.getSize()) : 0;
    bool ok = false;
    if ((len > 0) && !r.txInFlight) {
        switch (r.mode) {
            case RadioMode::FLRC:
                ok = this->flrcTx(r, data.getData(), len);
                break;
            case RadioMode::FSK:
                ok = this->fskTx(r, data.getData(), len);
                break;
            default:
                break;
        }
    }
    if (ok) {
        // The relay path does not use the working buffer, so flrcService's
        // "TX done" prints 0 bytes; log the actual relayed size here instead.
        DEBUG("radio %d relay TX %u bytes", static_cast<int>(idx), static_cast<unsigned>(len));
    } else {
        DEBUG("radio %d uplink relay dropped (%u bytes, txInFlight=%d)", static_cast<int>(idx),
              static_cast<unsigned>(len), static_cast<int>(r.txInFlight));
        this->log_WARNING_HI_TxFrameDropped(static_cast<U8>(idx));
    }
}

// ----------------------------------------------------------------------
// Handler implementations for commands
// ----------------------------------------------------------------------

void LR2021Manager ::RadioReset_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U8 radio) {
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

void LR2021Manager ::RadioSetMode_cmdHandler(FwOpcodeType opCode,
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
        case LR2021Manager_Mode::CW:
            target = RadioMode::CW;
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

void LR2021Manager ::RadioPower_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U8 radio, Fw::On power) {
    if (radio >= NUM_RADIOS) {
        this->log_WARNING_LO_BadRadioIndex(radio);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
        return;
    }
    if (!this->isConnected_powerGpioWrite_OutputPort(radio)) {
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
        return;
    }
    this->powerGpioWrite_out(radio, (power == Fw::On::ON) ? Fw::Logic::HIGH : Fw::Logic::LOW);
    this->log_ACTIVITY_HI_PowerSet(radio, power);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void LR2021Manager ::RadioSetModulation_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U8 radio, LR2021Manager_Mode mode) {
    if (radio >= NUM_RADIOS) {
        this->log_WARNING_LO_BadRadioIndex(radio);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
        return;
    }
    RadioSlot& r = this->m_radio[radio];
    if (r.mode == RadioMode::NONE) {
        this->log_WARNING_LO_RadioNotConfigured(radio);
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
        case LR2021Manager_Mode::CW:
            target = RadioMode::CW;
            break;
        default:
            break;
    }
    const bool ok = this->setMode(radio, target, r.freqHz, r.powerDbm);
    if (ok) {
        this->log_ACTIVITY_HI_ModeSet(radio, mode);
    }
    this->cmdResponse_out(opCode, cmdSeq, ok ? Fw::CmdResponse::OK : Fw::CmdResponse::EXECUTION_ERROR);
}

void LR2021Manager ::RadioSetFreq_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U8 radio, U32 freq_hz) {
    if (radio >= NUM_RADIOS) {
        this->log_WARNING_LO_BadRadioIndex(radio);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
        return;
    }
    RadioSlot& r = this->m_radio[radio];
    if (r.mode == RadioMode::NONE) {
        this->log_WARNING_LO_RadioNotConfigured(radio);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
        return;
    }
    const bool ok = this->setMode(radio, r.mode, freq_hz, r.powerDbm);
    this->cmdResponse_out(opCode, cmdSeq, ok ? Fw::CmdResponse::OK : Fw::CmdResponse::EXECUTION_ERROR);
}

void LR2021Manager ::RadioSetPower_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U8 radio, I8 power_dbm) {
    if (radio >= NUM_RADIOS) {
        this->log_WARNING_LO_BadRadioIndex(radio);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
        return;
    }
    RadioSlot& r = this->m_radio[radio];
    if (r.mode == RadioMode::NONE) {
        this->log_WARNING_LO_RadioNotConfigured(radio);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
        return;
    }
    const bool ok = this->setMode(radio, r.mode, r.freqHz, power_dbm);
    this->cmdResponse_out(opCode, cmdSeq, ok ? Fw::CmdResponse::OK : Fw::CmdResponse::EXECUTION_ERROR);
}

void LR2021Manager ::RadioTxRoute_cmdHandler(FwOpcodeType opCode,
                                             U32 cmdSeq,
                                             LR2021Manager_RouteQueue source,
                                             U8 target) {
    if (target > UART_RADIO) {
        this->log_WARNING_LO_BadRouteTarget(target);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
        return;
    }
    // RouteQueue values are defined to match the comQueueIndex the topology
    // assigns each ComQueue queue (EVT=0, TLM=1, FILE=2); see the .fpp comment.
    const FwIndexType queueIndex = static_cast<FwIndexType>(source.e);
    this->setTxRoute(queueIndex, target);
    this->log_ACTIVITY_HI_RouteSet(source, target);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void LR2021Manager ::RadioTxTest_cmdHandler(FwOpcodeType opCode,
                                            U32 cmdSeq,
                                            U8 radio,
                                            LR2021Manager_Mode mode,
                                            U32 freq_hz,
                                            I8 power_dbm,
                                            U32 duration_s) {
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
    if (target == RadioMode::NONE) {
        // CW (unmodulated) is not a valid packet engine for a PRBS9 test.
        this->log_WARNING_HI_TxTestError(radio);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
        return;
    }
    // Non-blocking: txTest() keys the test carrier and schedules its own stop
    // in run() at the deadline (blocking here would overflow the async message
    // queue -> queue-full FATAL). The command returns as soon as the carrier
    // is keyed; TxTestDone is emitted later by run() when RX is restored.
    this->log_ACTIVITY_HI_TxTestStarted(radio, mode, duration_s);
    const bool ok = this->txTest(radio, target, freq_hz, power_dbm, duration_s);
    if (!ok) {
        this->log_WARNING_HI_TxTestError(radio);
    }
    this->cmdResponse_out(opCode, cmdSeq, ok ? Fw::CmdResponse::OK : Fw::CmdResponse::EXECUTION_ERROR);
}

void LR2021Manager ::RadioBerTest_cmdHandler(FwOpcodeType opCode,
                                             U32 cmdSeq,
                                             U8 tx_radio,
                                             U8 rx_radio,
                                             LR2021Manager_Mode mode,
                                             U32 freq_hz,
                                             I8 power_dbm,
                                             U32 num_packets,
                                             U16 payload_len,
                                             U32 interval_ms) {
    if ((tx_radio >= NUM_RADIOS) || (rx_radio >= NUM_RADIOS)) {
        this->log_WARNING_LO_BadRadioIndex((tx_radio >= NUM_RADIOS) ? tx_radio : rx_radio);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
        return;
    }

    // Only the packet engines can carry a raw pattern; CW is unmodulated.
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

    // The minimum FLRC payload the radio accepts is FLRC_MIN_PAYLOAD; FSK
    // accepts any non-zero length.
    const U16 min_len = (target == RadioMode::FLRC) ? FLRC_MIN_PAYLOAD : 1;
    const bool valid = (target != RadioMode::NONE) && (tx_radio != rx_radio) &&
                       (num_packets >= 1) && (num_packets <= BER_MAX_PACKETS) &&
                       (payload_len >= min_len) && (payload_len <= BER_MAX_PAYLOAD);
    if (!valid || this->m_ber.active) {
        // Invalid args, or a test is already running (reject rather than
        // clobber the one in progress).
        this->log_WARNING_HI_BerTestError(tx_radio, rx_radio);
        this->cmdResponse_out(opCode, cmdSeq,
                              this->m_ber.active ? Fw::CmdResponse::BUSY : Fw::CmdResponse::VALIDATION_ERROR);
        return;
    }

    // Non-blocking: berStart arms both radios and the run() state machine
    // paces the TX / tallies the RX; BerTestDone is emitted by berFinish().
    this->log_ACTIVITY_HI_BerTestStarted(tx_radio, rx_radio, mode, num_packets);
    const bool ok = this->berStart(tx_radio, rx_radio, target, freq_hz, power_dbm, num_packets,
                                   payload_len, interval_ms);
    if (!ok) {
        this->m_ber.active = false;
        this->log_WARNING_HI_BerTestError(tx_radio, rx_radio);
    }
    this->cmdResponse_out(opCode, cmdSeq, ok ? Fw::CmdResponse::OK : Fw::CmdResponse::EXECUTION_ERROR);
}

}  // namespace LR2021
