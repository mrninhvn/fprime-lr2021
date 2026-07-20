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
    //! Handler implementation for send
    //!
    void asyncSendIn_handler(FwIndexType portNum,
                             Fw::Buffer& sendBuffer) override;
    
    //! Handler implementation for recvReturnIn
    //!
    //! Port receiving back ownership of data sent out on $recv port
    void recvReturnIn_handler(FwIndexType portNum,  //!< The port number
                              Fw::Buffer& fwBuffer  //!< The buffer
                              ) override;

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

    //! Read the IRQ line (radio DIO7). Returns true when an IRQ is pending,
    //! or when no IRQ port is connected (callers then poll over SPI).
    bool irqPending();

    //! Get chip version, make sure SPI is working and the radio is responding.
    void chipVersion();

    //! Initialize the radio chip (HF PA / RX path, etc.).
    bool radioInit();

    // ----------------------------------------------------------------------
    // Mode selection
    // ----------------------------------------------------------------------

    //! Active modulation / packet engine. Only one is active at a time
    //! on the radio.
    enum class RadioMode { NONE, FLRC, FSK };

    //! Configure the radio for \p mode and enter continuous RX. Used for
    //! the startup default (topology) and by the SET_MODE command.
    //! Aborts any in-flight TX, returning its buffer to the ComStub.
    //! \return true on success
    bool setMode(RadioMode mode, U32 freq_hz, I8 power_dbm);

    // ----------------------------------------------------------------------
    // FLRC radio operations (implemented in LR2021Flrc.cpp)
    // ----------------------------------------------------------------------

    //! Maximum FLRC payload handled by this component, in bytes
    static constexpr U16 FLRC_MAX_PAYLOAD = 511;
    //! Minimum FLRC payload allowed by the radio, in bytes
    static constexpr U16 FLRC_MIN_PAYLOAD = 6;

    //! Configure the radio for FLRC: packet type, RF frequency, HF PA/RX
    //! path, modulation / packet params and syncword.
    //! \return true on success
    bool flrcInit(U32 freq_hz, I8 power_dbm);

    //! Transmit \p len bytes (padded up to FLRC_MIN_PAYLOAD if shorter)
    //! \return true on success
    bool flrcTx(const U8* data, U16 len);

    //! Enter RX mode; timeout_ms == 0 selects continuous RX
    //! \return true on success
    bool flrcRx(U32 timeout_ms);

    //! Poll and handle radio IRQs (TX done / RX done / errors).
    //! Called from the rate-group run handler while FLRC is active.
    void flrcService();

    // ----------------------------------------------------------------------
    // FSK radio operations (implemented in LR2021Fsk.cpp)
    // ----------------------------------------------------------------------

    //! Maximum FSK payload handled by this component, in bytes
    static constexpr U16 FSK_MAX_PAYLOAD = 511;

    //! CCSDS role of this node on the FSK channel. The spacecraft sends TM
    //! frames on an ASM channel and receives TC frames as CLTUs
    //! (CCSDS 231.0-B); ground is the mirror image.
    enum class CcsdsRole { SPACECRAFT, GROUND };

    //! Select the CCSDS role. Takes effect on the next TX/RX operation;
    //! call before setMode(). Defaults to SPACECRAFT.
    void setCcsdsRole(CcsdsRole role) { this->m_ccsdsRole = role; }

    //! Configure the radio for FSK: packet type, RF frequency, PA/RX path,
    //! modulation / packet params and syncword.
    //! \return true on success
    bool fskInit(U32 freq_hz, I8 power_dbm);

    //! Transmit \p len bytes using FSK
    //! \return true on success
    bool fskTx(const U8* data, U16 len);

    //! Enter FSK RX mode; timeout_ms == 0 selects continuous RX
    //! \return true on success
    bool fskRx(U32 timeout_ms);

    //! Poll and handle radio IRQs (TX done / RX done / errors).
    //! Called from the rate-group run handler while FSK is active.
    void fskService();

    // ----------------------------------------------------------------------
    // RF power detectors (implemented in LR2021Adc.cpp)
    // ----------------------------------------------------------------------

    //! Read the LT5538 detectors on the LR2021 antenna coupler: forward
    //! (RF2 tap) and reflected (RF21 tap) power, in dBm at the detector
    //! inputs. Only meaningful while the PA is transmitting.
    //! See LR2021Adc.cpp for the transfer function and calibration.
    //! \return true when both channels were read successfully
    bool readRfPower(F32& fwd_dbm, F32& refl_dbm);

    //! Sample the RF detectors during the TX that just started and write
    //! the RfFwdPower / RfReflPower telemetry. Waits a short settle time
    //! for the PA ramp, so call right after set_tx succeeds (blocks the
    //! component thread ~1 ms). Implemented in LR2021Adc.cpp.
    void rfPowerMeasureTx();

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
    //! Reset the LR2021 radio and re-initialise it, restoring the active mode
    void RESET_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                          U32 cmdSeq            //!< The command sequence number
                          ) override;

    //! Handler implementation for command SET_MODE
    //!
    //! Switch the active modulation at runtime
    void SET_MODE_cmdHandler(FwOpcodeType opCode,      //!< The opcode
                             U32 cmdSeq,               //!< The command sequence number
                             LR2021Manager_Mode mode,  //!< Modulation to activate
                             U32 freq_hz,              //!< RF centre frequency in Hz
                             I8 power_dbm              //!< TX output power in dBm
                             ) override;

  private:
    // ----------------------------------------------------------------------
    // Radio state
    // ----------------------------------------------------------------------

    RadioMode m_mode = RadioMode::NONE; //!< Modulation selected by the last successful setMode()
    CcsdsRole m_ccsdsRole = CcsdsRole::SPACECRAFT; //!< CCSDS role on the FSK channel
    U32 m_freqHz = 0;            //!< RF frequency of the last successful setMode()
    I8 m_powerDbm = 0;           //!< TX power of the last successful setMode()
    bool m_rxContinuous = false; //!< Re-enter RX automatically after each packet
    bool m_txInFlight = false;   //!< TX started, TX_DONE not yet seen
    U32 m_txCount = 0;           //!< FLRC packets transmitted
    U32 m_rxCount = 0;           //!< FLRC packets received
    U32 m_fskTxCount = 0;        //!< FSK packets transmitted
    U32 m_fskRxCount = 0;        //!< FSK packets received
    bool m_adcReady = false;     //!< RF detector ADC channels configured
    Fw::Buffer m_workingBuffer;
};

//! Global pointer to the (single) manager instance, used by the C HAL bridge
//! implemented in LR2021Hal.cpp to reach the F Prime ports.
extern LR2021Manager* g_lr2021_manager;

}  // namespace LR2021

#endif
