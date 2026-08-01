// ======================================================================
// \title  LR2021Hal.cpp
// \author v.ninhdh4
// \brief  Implementation of the Semtech lr20xx_hal.h porting layer.
//
// These C-linkage functions are called by the lr20xx_driver C sources. They
// forward every radio access to the LR2021Manager F Prime component, which in
// turn drives the Zephyr SPI bus and the reset / busy GPIOs.
//
// The driver's opaque `context` pointer is a LR2021Manager::RadioSlot
// (every lr20xx_* call in the component passes &m_radio[i]); the slot
// carries the manager back-pointer plus the radio index, which selects the
// SPI chip select / GPIO port set of that module on the shared bus.
//
// SPI transport notes (see lr20xx_hal.h for the full contract):
//   * The Zephyr SPI port performs one full-duplex transfer per call, with a
//     single NSS assert/deassert and equal-length TX/RX buffers.
//   * When reading, the MOSI line must carry only zero (NOP) bytes, which we
//     guarantee by zero-filling the TX buffer.
// ======================================================================

#include "Components/LR2021Manager/LR2021Manager.hpp"

#include <cstring>

extern "C" {
#include "lr20xx_hal.h"
}

namespace {

using RadioSlot = LR2021::LR2021Manager::RadioSlot;

//! Maximum bytes handled in a single transfer (matches HAL_BUFFER_SIZE).
constexpr uint16_t HAL_BUF_SIZE = LR2021::LR2021Manager::HAL_BUFFER_SIZE;

//! Recover the radio slot from the driver's opaque context pointer.
RadioSlot* slot(const void* context) {
    return static_cast<RadioSlot*>(const_cast<void*>(context));
}

//! Run one full-duplex transfer through the slot's manager / port set.
lr20xx_hal_status_t hal_xfer(RadioSlot* r, const uint8_t* tx, uint8_t* rx, uint16_t len) {
    if ((r == nullptr) || (r->mgr == nullptr) || (len == 0)) {
        return LR20XX_HAL_STATUS_ERROR;
    }
    return r->mgr->spiTransfer(r->idx, tx, rx, len) ? LR20XX_HAL_STATUS_OK : LR20XX_HAL_STATUS_ERROR;
}

}  // namespace

extern "C" {

lr20xx_hal_status_t lr20xx_hal_reset(const void* context) {
    RadioSlot* r = slot(context);
    if ((r == nullptr) || (r->mgr == nullptr)) {
        return LR20XX_HAL_STATUS_ERROR;
    }
    r->mgr->halReset(r->idx);
    return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_wakeup(const void* context) {
    RadioSlot* r = slot(context);
    if ((r == nullptr) || (r->mgr == nullptr)) {
        return LR20XX_HAL_STATUS_ERROR;
    }
    return r->mgr->halWakeup(r->idx) ? LR20XX_HAL_STATUS_OK : LR20XX_HAL_STATUS_ERROR;
}

lr20xx_hal_status_t lr20xx_hal_write(const void* context, const uint8_t* command, const uint16_t command_length,
                                     const uint8_t* data, const uint16_t data_length) {
    RadioSlot* r = slot(context);
    if ((r == nullptr) || (r->mgr == nullptr)) {
        return LR20XX_HAL_STATUS_ERROR;
    }
    // The radio must be ready before it can accept a command.
    if (!r->mgr->waitOnBusy(r->idx)) {
        return LR20XX_HAL_STATUS_ERROR;
    }

    const uint16_t total = command_length + data_length;
    if (total > HAL_BUF_SIZE) {
        return LR20XX_HAL_STATUS_ERROR;
    }

    uint8_t txbuf[HAL_BUF_SIZE];
    uint8_t rxbuf[HAL_BUF_SIZE];
    std::memcpy(txbuf, command, command_length);
    if (data_length > 0) {
        std::memcpy(txbuf + command_length, data, data_length);
    }
    return hal_xfer(r, txbuf, rxbuf, total);
}

lr20xx_hal_status_t lr20xx_hal_read(const void* context, const uint8_t* command, const uint16_t command_length,
                                    uint8_t* data, const uint16_t data_length) {
    RadioSlot* r = slot(context);
    if ((r == nullptr) || (r->mgr == nullptr)) {
        return LR20XX_HAL_STATUS_ERROR;
    }
    if (!r->mgr->waitOnBusy(r->idx)) {
        DEBUG("waitOnBusy failed before command write\n");
        return LR20XX_HAL_STATUS_ERROR;
    }

    // Step 1: write the command with its own NSS assert/deassert.
    if (command_length > HAL_BUF_SIZE) {
        DEBUG("command_length %u exceeds HAL_BUF_SIZE %u\n", command_length, HAL_BUF_SIZE);
        return LR20XX_HAL_STATUS_ERROR;
    }
    uint8_t txbuf[HAL_BUF_SIZE];
    uint8_t rxbuf[HAL_BUF_SIZE];
    std::memcpy(txbuf, command, command_length);
    lr20xx_hal_status_t status = hal_xfer(r, txbuf, rxbuf, command_length);
    if (status != LR20XX_HAL_STATUS_OK) {
        return status;
    }

    // The radio needs time to prepare the response after the command.
    if (!r->mgr->waitOnBusy(r->idx)) {
        DEBUG("waitOnBusy failed after command write\n");
        return LR20XX_HAL_STATUS_ERROR;
    }

    // Step 2: read the discarded dummy bytes followed by data_length bytes,
    // clocking out only zeros (NOP). The LR20xx returns two dummy bytes before
    // the response payload (see the Semtech reference lr20xx_hal.c).
    constexpr uint16_t DUMMY_BYTES = 2;
    const uint16_t total = DUMMY_BYTES + data_length;
    if (total > HAL_BUF_SIZE) {
        DEBUG("total %u exceeds HAL_BUF_SIZE %u\n", total, HAL_BUF_SIZE);
        return LR20XX_HAL_STATUS_ERROR;
    }
    std::memset(txbuf, 0, total);
    status = hal_xfer(r, txbuf, rxbuf, total);
    if (status != LR20XX_HAL_STATUS_OK) {
        return status;
    }
    std::memcpy(data, rxbuf + DUMMY_BYTES, data_length);
    return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_direct_read(const void* context, uint8_t* data, const uint16_t data_length) {
    // Simple SS / read / nSS operation (used by lr20xx_system_get_status). The
    // Semtech reference waits for the radio to be ready first.
    RadioSlot* r = slot(context);
    if ((r == nullptr) || (r->mgr == nullptr)) {
        return LR20XX_HAL_STATUS_ERROR;
    }
    if (!r->mgr->waitOnBusy(r->idx)) {
        return LR20XX_HAL_STATUS_ERROR;
    }
    if (data_length > HAL_BUF_SIZE) {
        return LR20XX_HAL_STATUS_ERROR;
    }
    uint8_t txbuf[HAL_BUF_SIZE];
    std::memset(txbuf, 0, data_length);
    return hal_xfer(r, txbuf, data, data_length);
}

lr20xx_hal_status_t lr20xx_hal_direct_read_fifo(const void* context, const uint8_t* command,
                                                const uint16_t command_length, uint8_t* data,
                                                const uint16_t data_length) {
    RadioSlot* r = slot(context);
    if ((r == nullptr) || (r->mgr == nullptr)) {
        return LR20XX_HAL_STATUS_ERROR;
    }
    if (!r->mgr->waitOnBusy(r->idx)) {
        return LR20XX_HAL_STATUS_ERROR;
    }

    // One-step read: write the command then read data_length bytes in a single
    // NSS assertion. Zero-fill the read portion so only NOPs go out on MOSI.
    const uint16_t total = command_length + data_length;
    if (total > HAL_BUF_SIZE) {
        return LR20XX_HAL_STATUS_ERROR;
    }
    uint8_t txbuf[HAL_BUF_SIZE];
    uint8_t rxbuf[HAL_BUF_SIZE];
    std::memcpy(txbuf, command, command_length);
    std::memset(txbuf + command_length, 0, data_length);
    lr20xx_hal_status_t status = hal_xfer(r, txbuf, rxbuf, total);
    if (status != LR20XX_HAL_STATUS_OK) {
        return status;
    }
    std::memcpy(data, rxbuf + command_length, data_length);
    return LR20XX_HAL_STATUS_OK;
}

}  // extern "C"
