// ======================================================================
// \title  LR2021Manager.hpp
// \author v.ninhdh4
// \brief  hpp file for LR2021Manager component implementation class
//
// F Prime manager for the Semtech LR2021 (LR20xx) transceivers. One
// component instance manages both radio modules of the board on the shared
// SPI bus (each module has its own chip select and GPIO set). It owns the
// lr20xx_driver C driver; the driver's opaque context pointer is a
// RadioSlot, which carries the manager back-pointer plus the radio index,
// so the HAL bridge (LR2021Hal.cpp) can reach the right SPI/GPIO ports.
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

    //! Number of radio modules managed by this component.
    static constexpr FwIndexType NUM_RADIOS = 2;

    //! Route-table target meaning "send out the byte-stream driver (UART)"
    //! instead of a radio. One past the last radio index.
    static constexpr FwIndexType UART_RADIO = NUM_RADIOS;

    //! Number of synchronous send retries on the byte-stream driver.
    static constexpr FwIndexType UART_RETRY_LIMIT = 3;

    // ----------------------------------------------------------------------
    // Radio-level types
    // ----------------------------------------------------------------------

    //! Active modulation / packet engine of one radio.
    enum class RadioMode { NONE, FLRC, FSK };

    //! Destination for received (uplink) packets.
    //!   DATA_OUT: the Svc.Com dataOut port (frame accumulator / deframer);
    //!             the flight default.
    //!   UART:     straight out drvSendOut to the byte-stream (UART) driver,
    //!             raw, for a ground relay that pipes radio RX to a host GDS.
    enum class RxSink { DATA_OUT, UART };

    //! Radio module populated on an SPI slot. The modules differ in
    //! clocking (NiceRF: chip-supplied TCXO; RY42F: crystal) and in their
    //! DIO map (IRQ pad and RF-switch tree), so radioInit() branches on it.
    enum class ModuleType { NICERF, RY42F };

    //! CCSDS role of a node on the FSK channel. The spacecraft sends TM
    //! frames on an ASM channel and receives TC frames as CLTUs
    //! (CCSDS 231.0-B); ground is the mirror image.
    enum class CcsdsRole { SPACECRAFT, GROUND };

    //! Per-radio state. A pointer to this struct is the lr20xx_driver
    //! context, so it carries the back-pointer needed by the C HAL bridge.
    struct RadioSlot {
        LR2021Manager* mgr = nullptr;  //!< Owning component (for the HAL)
        FwIndexType idx = 0;           //!< Radio index (selects the port set)
        ModuleType moduleType = ModuleType::NICERF;
        CcsdsRole ccsdsRole = CcsdsRole::SPACECRAFT;
        RadioMode mode = RadioMode::NONE;  //!< Set by the last successful setMode()
        U32 freqHz = 0;      //!< Base RF frequency of the last setMode()
        U32 txFreqHz = 0;    //!< TX frequency override; 0 = use freqHz
        U32 rxFreqHz = 0;    //!< RX frequency override; 0 = use freqHz
        U32 progFreqHz = 0;  //!< RF frequency currently programmed on the chip
        I8 powerDbm = 0;     //!< TX power of the last setMode()
        bool rxContinuous = false;  //!< Re-enter RX automatically after each packet
        bool txInFlight = false;    //!< TX started, TX_DONE not yet seen
        Fw::Buffer workingBuffer;   //!< Buffer of the in-flight TX
        ComCfg::FrameContext workingContext;  //!< Frame context of the in-flight TX
    };

    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct LR2021Manager object
    LR2021Manager(const char* const compName  //!< The component name
    );

    //! Destroy LR2021Manager object
    ~LR2021Manager();

  public:
    //! Handler implementation for dataIn (Svc.Com interface, async)
    //!
    //! Downlink frame to transmit; routed to a radio by context.comQueueIndex
    void dataIn_handler(FwIndexType portNum,
                        Fw::Buffer& data,
                        const ComCfg::FrameContext& context) override;

    //! Handler implementation for dataReturnIn (Svc.Com interface)
    //!
    //! Receives back ownership of buffers sent out on dataOut
    void dataReturnIn_handler(FwIndexType portNum,
                              Fw::Buffer& data,
                              const ComCfg::FrameContext& context) override;

  public:
    // ----------------------------------------------------------------------
    // Helpers used by the HAL bridge (LR2021Hal.cpp), per radio index
    // ----------------------------------------------------------------------

    //! Perform a full-duplex SPI transfer of \p len bytes on radio \p idx.
    //! \p tx and \p rx may point at the same or different buffers.
    //! \return true on success
    bool spiTransfer(FwIndexType idx, const U8* tx, U8* rx, U16 len);

    //! Toggle the active-low NRESET line of radio \p idx.
    void halReset(FwIndexType idx);

    //! Wake radio \p idx from sleep by generating a NSS edge + BUSY wait.
    //! \return true on success
    bool halWakeup(FwIndexType idx);

    //! Poll the BUSY line of radio \p idx until low or timeout.
    //! If no BUSY port is connected this returns true immediately.
    //! \return true when the radio is ready, false on timeout / error
    bool waitOnBusy(FwIndexType idx, U32 timeout_us = BUSY_TIMEOUT_US);

    //! Read the IRQ line of radio \p idx. Returns true when an IRQ is
    //! pending, or when no IRQ port is connected (callers then poll over SPI).
    bool irqPending(FwIndexType idx);

    //! Get chip version of radio \p idx (SPI sanity check).
    void chipVersion(FwIndexType idx);

    // ----------------------------------------------------------------------
    // Radio bring-up / selection (called from the topology and commands)
    // ----------------------------------------------------------------------

    //! Select the module type of radio \p idx. Call before radioInit().
    void setModuleType(FwIndexType idx, ModuleType type);

    //! Select the CCSDS role of radio \p idx on the FSK channel. Takes
    //! effect on the next TX/RX operation; call before setMode().
    void setCcsdsRole(FwIndexType idx, CcsdsRole role);

    //! Manually set the TX RF frequency of radio \p idx, in Hz. 0 (default)
    //! means "use the frequency passed to setMode". Keep it in the same band
    //! as setMode's frequency (the PA / RX path is selected once at init).
    void setTxFreq(FwIndexType idx, U32 tx_freq_hz);

    //! Manually set the RX (rest) RF frequency of radio \p idx, in Hz.
    //! 0 (default) means "use the frequency passed to setMode".
    void setRxFreq(FwIndexType idx, U32 rx_freq_hz);

    //! Route downlink frames of ComQueue queue \p comQueueIndex to radio
    //! \p radioIdx. Unconfigured queue indices go to radio 0. Called from
    //! the topology (e.g. route the FILE buffer queue to the S-band radio).
    void setTxRoute(FwIndexType comQueueIndex, FwIndexType radioIdx);

    //! Select where received packets are forwarded: the Svc.Com dataOut port
    //! (RxSink::DATA_OUT, default / flight) or straight out the byte-stream
    //! (UART) driver via drvSendOut (RxSink::UART, ground relay). Call from the
    //! topology before RX starts.
    void setRxSink(RxSink sink);

    //! Initialize the chip of radio \p idx (clocks, DIO/RF-switch map for
    //! its module type, regulator).
    bool radioInit(FwIndexType idx);

    //! Configure radio \p idx for \p mode and enter continuous RX. Used for
    //! the startup default (topology) and by the SET_MODE command.
    //! Aborts any in-flight TX of that radio, returning its buffer.
    //! \return true on success
    bool setMode(FwIndexType idx, RadioMode mode, U32 freq_hz, I8 power_dbm);

    //! Bench test: key an unmodulated continuous-wave carrier on radio \p idx
    //! at \p freq_hz / \p power_dbm (keep freq_hz in the band selected by the
    //! preceding setMode). Stays on until the next setMode()/RESET. Meant to
    //! be called at boot for spectrum-analyser measurements.
    //! \return true on success
    bool txCw(FwIndexType idx, U32 freq_hz, I8 power_dbm);

    // ----------------------------------------------------------------------
    // FLRC radio operations (implemented in LR2021Flrc.cpp)
    // ----------------------------------------------------------------------

    //! Maximum FLRC payload handled by this component, in bytes
    static constexpr U16 FLRC_MAX_PAYLOAD = 511;
    //! Minimum FLRC payload allowed by the radio, in bytes
    static constexpr U16 FLRC_MIN_PAYLOAD = 6;

    //! Configure radio \p r for FLRC: packet type, RF frequency, PA/RX
    //! path, modulation / packet params and syncword.
    //! \return true on success
    bool flrcInit(RadioSlot& r, U32 freq_hz, I8 power_dbm);

    //! Transmit \p len bytes (padded up to FLRC_MIN_PAYLOAD if shorter)
    //! \return true on success
    bool flrcTx(RadioSlot& r, const U8* data, U16 len);

    //! Enter RX mode; timeout_ms == 0 selects continuous RX
    //! \return true on success
    bool flrcRx(RadioSlot& r, U32 timeout_ms);

    //! Poll and handle radio IRQs (TX done / RX done / errors).
    void flrcService(RadioSlot& r);

    // ----------------------------------------------------------------------
    // FSK radio operations (implemented in LR2021Fsk.cpp)
    // ----------------------------------------------------------------------

    //! Maximum FSK payload handled by this component, in bytes
    static constexpr U16 FSK_MAX_PAYLOAD = 511;

    //! Configure radio \p r for FSK: packet type, RF frequency, PA/RX path,
    //! modulation / packet params and syncword.
    //! \return true on success
    bool fskInit(RadioSlot& r, U32 freq_hz, I8 power_dbm);

    //! Program the RF center frequency of radio \p r, skipping the SPI
    //! write when the chip is already tuned to it.
    //! \return true on success
    bool fskTune(RadioSlot& r, U32 freq_hz);

    //! Transmit \p len bytes using FSK
    //! \return true on success
    bool fskTx(RadioSlot& r, const U8* data, U16 len);

    //! Enter FSK RX mode; timeout_ms == 0 selects continuous RX
    //! \return true on success
    bool fskRx(RadioSlot& r, U32 timeout_ms);

    //! Poll and handle radio IRQs (TX done / RX done / errors).
    void fskService(RadioSlot& r);

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

    // ----------------------------------------------------------------------
    // PolyDb (implemented in LR2021Manager.cpp)
    // ----------------------------------------------------------------------

    //! Read radio \p idx's junction temperature via lr20xx_system_get_temp.
    //! \return true on success
    bool readDieTemp(FwIndexType idx, I8& tempC);

    //! Sample both radios' die temperature and publish the higher (worst
    //! case) reading to POLYDB_ENTRY_OBC_LR2021_Temperature. No-op if setPoly
    //! is unconnected.
    void sendTempPoly();

    //! Publish an RSSI reading (dBm) to the given PolyDb entry, right after a
    //! packet RX. No-op if setPoly is unconnected.
    void sendRssiPoly(Svc::PolyDbCfg::PolyDbEntry entry, I16 rssiDbm);

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

    //! Handler implementation for drvConnected (byte-stream driver ready).
    //! No-op: the initial comStatus credit is emitted by setMode(); crediting
    //! again here would let the framer send two frames before the first
    //! completes.
    void drvConnected_handler(FwIndexType portNum) override;

    //! Handler implementation for drvReceiveIn: forward uplink bytes from the
    //! byte-stream driver to dataOut (like a radio RX), feeding the deframe /
    //! APID-router stack. Frees the buffer on a receive error. Async: runs on
    //! this component's thread.
    void drvReceiveIn_handler(FwIndexType portNum,                     //!< The port number
                              Fw::Buffer& recvBuffer,                  //!< The received buffer
                              const Drv::ByteStreamStatus& recvStatus  //!< Receive status
                              ) override;

    //! Handler implementation for relayIn: transmit a complete frame out the
    //! radio verbatim (fire-and-forget), then free the buffer. Async: runs on
    //! this component's thread, serialized with run().
    void relayIn_handler(FwIndexType portNum,
                         Fw::Buffer& data,
                         const ComCfg::FrameContext& context) override;

    // ----------------------------------------------------------------------
    // Handler implementations for commands
    // ----------------------------------------------------------------------

    //! Handler implementation for command RESET
    //!
    //! Reset one radio and re-initialise it, restoring its active mode
    void RadioReset_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                               U32 cmdSeq,           //!< The command sequence number
                               U8 radio              //!< Radio index
                              ) override;

    //! Handler implementation for command SET_MODE
    //!
    //! Switch the active modulation of one radio at runtime
    void RadioSetMode_cmdHandler(FwOpcodeType opCode,      //!< The opcode
                                 U32 cmdSeq,               //!< The command sequence number
                                 U8 radio,                 //!< Radio index
                                 LR2021Manager_Mode mode,  //!< Modulation to activate
                                 U32 freq_hz,              //!< RF centre frequency in Hz
                                 I8 power_dbm              //!< TX output power in dBm
                                ) override;

    //! Handler implementation for command RadioPower
    //!
    //! Turn a radio module's load switch on or off
    void RadioPower_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                          U32 cmdSeq,           //!< The command sequence number
                          U8 radio,             //!< Radio index
                          Fw::On power          //!< ON = power the module, OFF = cut power
                          ) override;

    //! Handler implementation for command RadioSetModulation
    //!
    //! Change only the modulation of a radio, keeping its last freq/power
    void RadioSetModulation_cmdHandler(FwOpcodeType opCode,      //!< The opcode
                                   U32 cmdSeq,               //!< The command sequence number
                                   U8 radio,                 //!< Radio index
                                   LR2021Manager_Mode mode   //!< Modulation to activate
                                   ) override;

    //! Handler implementation for command RadioSetFreq
    //!
    //! Change only the RF centre frequency of a radio, keeping mode/power
    void RadioSetFreq_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                 U32 cmdSeq,           //!< The command sequence number
                                 U8 radio,             //!< Radio index
                                 U32 freq_hz           //!< RF centre frequency in Hz
                                ) override;

    //! Handler implementation for command RadioSetPower
    //!
    //! Change only the TX output power of a radio, keeping mode/frequency
    void RadioSetPower_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                              U32 cmdSeq,           //!< The command sequence number
                              U8 radio,             //!< Radio index
                              I8 power_dbm          //!< TX output power in dBm
                              ) override;

    //! Handler implementation for command RadioTxRoute
    //!
    //! Retarget a downlink source (events / telemetry / file) at runtime
    void RadioTxRoute_cmdHandler(FwOpcodeType opCode,             //!< The opcode
                                 U32 cmdSeq,                      //!< The command sequence number
                                 LR2021Manager_RouteQueue source,  //!< Downlink source to retarget
                                 U8 target                        //!< Radio index (0/1) or UART_RADIO
                                 ) override;

  private:
    // ----------------------------------------------------------------------
    // Downlink routing / com status helpers
    // ----------------------------------------------------------------------

    //! Size of the TX route table, in ComQueue queue indices. Large enough
    //! for the com queues plus the buffer (file) queues of the deployment.
    static constexpr FwIndexType TX_ROUTE_TABLE_SIZE = 8;

    //! Select the TX radio for a downlink frame from its frame context
    //! (route table lookup on comQueueIndex; out-of-table indices -> radio 0).
    FwIndexType routeTx(const ComCfg::FrameContext& context) const;

    //! Finish the in-flight TX of slot \p r: return the working buffer with
    //! its context on dataReturnOut and emit \p status on comStatusOut.
    //! FAILURE closes the com flow until the next successful setMode().
    void txComplete(RadioSlot& r, Fw::Success status);

    //! Forward a received packet of \p len bytes to the configured RX sink
    //! (dataOut in flight, or the byte-stream/UART driver on a ground relay).
    //! Allocates a buffer from the buffer manager and always returns it. Does
    //! nothing when \p data is nullptr or \p len is 0.
    void forwardRxPacket(const U8* data, U16 len);

    //! Ground uplink relay: transmit the bytes in \p data over the radio.
    //! Full route to radio 0 for now (TODO: route by CCSDS APID). The radio
    //! TX copies the payload into its FIFO synchronously, so the caller keeps
    //! ownership of \p data and frees it after this returns.
    void relayUplink(Fw::Buffer& data);

    // ----------------------------------------------------------------------
    // State
    // ----------------------------------------------------------------------

    RadioSlot m_radio[NUM_RADIOS];  //!< Per-radio state (driver contexts)
    FwIndexType m_txRoute[TX_ROUTE_TABLE_SIZE] = {0};  //!< comQueueIndex -> radio
    RxSink m_rxSink = RxSink::DATA_OUT;                 //!< Destination for received packets
    bool m_comOpen = false;         //!< Initial comStatus READY emitted / flow open
    U32 m_txCount = 0;              //!< FLRC packets transmitted (all radios)
    U32 m_rxCount = 0;              //!< FLRC packets received (all radios)
    U32 m_fskTxCount = 0;           //!< FSK packets transmitted (all radios)
    U32 m_fskRxCount = 0;           //!< FSK packets received (all radios)
    bool m_adcReady = false;        //!< RF detector ADC channels configured
};

}  // namespace LR2021

#endif
