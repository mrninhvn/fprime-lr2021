// ======================================================================
// \title  LR2021Manager.hpp
// \author v.ninhdh4
// \brief  hpp file for LR2021Manager component implementation class
//
// F Prime manager for the Semtech LR2021 (LR20xx) transceiver. It owns the
// lr20xx_driver C driver and exposes the helper methods used by the HAL
// bridge (LR2021Hal.cpp) to reach the SPI bus and the reset / busy GPIOs.
// ======================================================================

#ifndef LR2021_LR2021Manager_HPP
#define LR2021_LR2021Manager_HPP

#include "Components/LR2021Manager/LR2021ManagerComponentAc.hpp"

#include <Fw/Types/BasicTypes.hpp>
#include <Os/Task.hpp>
#include <Os/Console.hpp>
#include <Fw/Types/String.hpp>

#define DEBUG_CONSOLE 1

#ifdef DEBUG_CONSOLE
#define DEBUG(msg, ...) do { \
    char _dbg_buf[128]; \
    snprintf(_dbg_buf, sizeof(_dbg_buf), "[LR2021] %s: " msg "\n", __func__, ##__VA_ARGS__); \
    Fw::String _dbg_str(_dbg_buf); \
    Os::Console::write(_dbg_str); \
} while(0)
#else
#define DEBUG(msg, ...) do { } while(0)
#endif

namespace LR2021 {

class LR2021Manager final : public LR2021ManagerComponentBase {
  public:
    //! Maximum size of a single SPI transaction issued by the HAL, in bytes.
    //! Large enough for a command header plus a full 256-byte FIFO transfer.
    static constexpr FwSizeType HAL_BUFFER_SIZE = 512;

    //! Default time to wait for the BUSY line to go low, in microseconds.
    static constexpr U32 BUSY_TIMEOUT_US = 100000;

    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct LR2021Manager object
    LR2021Manager(const char* const compName  //!< The component name
    );

    //! Destroy LR2021Manager object
    ~LR2021Manager();

  public:
    // ----------------------------------------------------------------------
    // Helpers used by the HAL bridge (LR2021Hal.cpp)
    // ----------------------------------------------------------------------

    //! Perform a full-duplex SPI transfer of \p len bytes.
    //! \p tx and \p rx may point at the same or different buffers.
    //! \return true on success
    bool spiTransfer(const U8* tx, U8* rx, U16 len);

    //! Toggle the active-low NRESET line to reset the radio.
    void halReset();

    //! Wake the radio from sleep by generating a NSS edge and waiting on BUSY.
    //! \return true on success
    bool halWakeup();

    //! Poll the BUSY line until it goes low or the timeout elapses.
    //! If no BUSY port is connected this returns true immediately.
    //! \return true when the radio is ready, false on timeout / error
    bool waitOnBusy(U32 timeout_us = BUSY_TIMEOUT_US);

    //! Get chip version, make sure SPI is working and the radio is responding.
    void chipVersion();

    //! Emit a debug event
    void logDebug(const Fw::LogStringArg& msg);

    //! Emit a hex dump of a buffer as a Debug event (bring-up diagnostic)
    void logHex(const char* tag, const U8* data, U16 len);

    //! Delay helpers
    void delayMs(U32 ms) { Os::Task::delay(Fw::TimeInterval(ms / 1000, (ms % 1000) * 1000)); }
    void delayUs(U32 us) { Os::Task::delay(Fw::TimeInterval(us / 1000000, us % 1000000)); }

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for typed input ports
    // ----------------------------------------------------------------------

    //! Handler implementation for run
    //!
    //! Rate-group input for periodic servicing (e.g. IRQ / RX polling)
    void run_handler(FwIndexType portNum,  //!< The port number
                     U32 context           //!< The call order
                     ) override;

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for commands
    // ----------------------------------------------------------------------

    //! Handler implementation for command RESET
    //!
    //! Reset the LR2021 radio
    void RESET_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                          U32 cmdSeq            //!< The command sequence number
                          ) override;
};

//! Global pointer to the (single) manager instance, used by the C HAL bridge
//! implemented in LR2021Hal.cpp to reach the F Prime ports.
extern LR2021Manager* g_lr2021_manager;

}  // namespace LR2021

#endif
