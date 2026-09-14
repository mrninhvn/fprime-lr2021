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

    //! Largest BER-test payload, in bytes (max of the FSK / FLRC payloads).
    static constexpr U16 BER_MAX_PAYLOAD = 511;

    //! Upper bound on the BER-test packet count (keeps the bit tallies within
    //! U32: BER_MAX_PACKETS * BER_MAX_PAYLOAD * 8 < 2^32).
    static constexpr U32 BER_MAX_PACKETS = 1000000;

    //! Grace period after the last BER packet is sent, letting stragglers
    //! arrive before the result is tallied, in milliseconds.
    static constexpr U32 BER_DRAIN_MS = 2000;

    // ----------------------------------------------------------------------
    // Radio-level types
    // ----------------------------------------------------------------------

    //! Active modulation / packet engine of one radio. CW is a bench-test
    //! state (unmodulated carrier, no packet engine): run()/dataIn treat it
    //! like NONE (service/TX no-ops via their switch default), so a radio
    //! holding a carrier never gets IRQ-serviced or handed a downlink frame.
    enum class RadioMode { NONE, FLRC, FSK, CW };

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

        // Timed PRBS9 TX test (RadioTxTest command). Non-blocking: the carrier
        // is keyed, then run() stops it and restores RX once the deadline
        // passes -- blocking the thread would overflow the async message queue.
        bool txTesting = false;                    //!< A timed test carrier is up on this slot
        Fw::Time txTestEnd;                        //!< Wall-clock deadline to stop it
        RadioMode txTestMode = RadioMode::NONE;    //!< Mode to restore when the test ends
        U32 txTestFreqHz = 0;                      //!< Frequency to restore
        I8 txTestPowerDbm = 0;                     //!< Power to restore
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

    //! Configure radio \p idx for \p mode and enter continuous RX (or, for
    //! RadioMode::CW, key an unmodulated carrier instead). Used for the
    //! startup default (topology) and by the RadioSetMode/RadioReset commands.
    //! Aborts any in-flight TX of that radio, returning its buffer.
    //! \return true on success
    bool setMode(FwIndexType idx, RadioMode mode, U32 freq_hz, I8 power_dbm);

    //! Bench test convenience: key an unmodulated continuous-wave carrier on
    //! radio \p idx at \p freq_hz / \p power_dbm. Thin wrapper around
    //! setMode(idx, RadioMode::CW, freq_hz, power_dbm); stays on until the
    //! next setMode()/RESET.
    //! \return true on success
    bool txCw(FwIndexType idx, U32 freq_hz, I8 power_dbm);

    //! Bench test: key a continuous PRBS9-modulated carrier on radio \p idx
    //! (packet engine selected by \p mode, FSK or FLRC only) at \p freq_hz /
    //! \p power_dbm for \p duration_s seconds. Non-blocking: keys the carrier
    //! and schedules its own stop in run() at the deadline (blocking would
    //! overflow the async message queue). run() restores continuous RX in
    //! \p mode when the deadline passes.
    //! \return true if the carrier was keyed (false if mode is not FSK/FLRC,
    //!         or a radio operation failed -- RX is restored best-effort)
    bool txTest(FwIndexType idx, RadioMode mode, U32 freq_hz, I8 power_dbm, U32 duration_s);

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

    //! Raw FSK BER-test TX: send \p len bytes verbatim on the BER syncword,
    //! with no CCSDS coding (implicit fixed length, CRC off so a corrupted
    //! packet is still delivered for the bit-compare). \return true on success
    bool fskBerTx(RadioSlot& r, const U8* data, U16 len);

    //! Enter continuous FSK RX on the BER syncword for fixed-length \p len
    //! packets (raw, no CCSDS coding). \return true on success
    bool fskBerRx(RadioSlot& r, U16 len);

    // ----------------------------------------------------------------------
    // BER test (implemented in LR2021Ber.cpp)
    // ----------------------------------------------------------------------

    //! Arm a BER test: configure \p txRadio and \p rxRadio for \p mode /
    //! \p freq_hz / \p power_dbm on the dedicated BER channel and start the
    //! non-blocking send/receive state machine driven by run(). \p txRadio
    //! and \p rxRadio must differ. \return true if both radios were armed
    bool berStart(FwIndexType txRadio, FwIndexType rxRadio, RadioMode mode, U32 freq_hz,
                  I8 power_dbm, U32 num_packets, U16 payload_len, U32 interval_ms);

    //! Advance the running BER test: paces test-packet TX and, once every
    //! packet is sent plus a drain window, finalizes the result. Called from
    //! run() each tick; no-op when no test is active.
    void berDrive();

    //! Finalize the running BER test: emit BerTestDone + telemetry and restore
    //! both radios to normal continuous RX in their mode.
    void berFinish();

    //! Transmit one BER test packet (the stored pattern) from slot \p r,
    //! dispatching to the FSK / FLRC raw TX. \return true on success
    bool berTxOne(RadioSlot& r);

    //! Called from the FSK / FLRC RX_DONE service: when a BER test is running
    //! and \p r is its RX radio, read the \p pkt_len received bytes, count bit
    //! errors against the pattern, re-arm RX, and return true (the caller then
    //! skips its normal RX forwarding). Returns false otherwise.
    bool berRxIntercept(RadioSlot& r, U16 pkt_len);

    //! Count the payload bit errors of one received BER packet (\p len bytes)
    //! against the stored pattern and accumulate into the running tallies.
    void berCompare(const U8* recv, U16 len);

    //! Fill \p buf with \p len bytes of the deterministic PRBS9 test pattern
    //! used by both the transmitter and the receiver's reference.
    static void berFillPattern(U8* buf, U16 len);

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
    //! The Measure Unit ADC only converts in STDBY_XOSC (it reads 0 during
    //! TX/RX), so the radio must be in STDBY_XOSC when this is called.
    //! \return true on success
    bool readDieTemp(FwIndexType idx, I8& tempC);

    //! Read radio \p idx's die temperature and publish it to that module's
    //! PolyDb entry (NiceRF vs RY42F). Must be called while the radio is in
    //! STDBY_XOSC (e.g. the TX->RX turnaround in fskRx/flrcRx). No-op if
    //! setPoly is unconnected or the read fails.
    void publishTempPoly(FwIndexType idx);

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

    //! Handler implementation for command RadioTxTest
    //!
    //! Key a continuous PRBS9-modulated test carrier for a bounded duration
    void RadioTxTest_cmdHandler(FwOpcodeType opCode,      //!< The opcode
                                U32 cmdSeq,               //!< The command sequence number
                                U8 radio,                 //!< Radio index
                                LR2021Manager_Mode mode,  //!< FSK or FLRC
                                U32 freq_hz,              //!< RF centre frequency in Hz
                                I8 power_dbm,             //!< TX output power in dBm
                                U32 duration_s            //!< Test duration in seconds
                                ) override;

    //! Handler implementation for command RadioBerTest
    //!
    //! Measure the over-the-air bit error rate between two radio modules
    void RadioBerTest_cmdHandler(FwOpcodeType opCode,      //!< The opcode
                                 U32 cmdSeq,               //!< The command sequence number
                                 U8 tx_radio,              //!< Transmitting radio index
                                 U8 rx_radio,              //!< Receiving radio index
                                 LR2021Manager_Mode mode,  //!< FSK or FLRC
                                 U32 freq_hz,              //!< RF centre frequency in Hz
                                 I8 power_dbm,             //!< TX output power in dBm
                                 U32 num_packets,          //!< Number of test packets
                                 U16 payload_len,          //!< Test payload length, bytes
                                 U32 interval_ms           //!< Minimum spacing between packets
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
    // BER test state (RadioBerTest command, driven by run())
    // ----------------------------------------------------------------------

    //! Phase of a running BER test.
    enum class BerPhase {
        SENDING,   //!< Still transmitting test packets
        DRAINING   //!< All sent; waiting for the last packets to arrive
    };

    //! Non-blocking BER test. active gates run()'s send/tally loop and the
    //! RX-service interception; every field is set by berStart().
    struct BerTest {
        bool active = false;
        BerPhase phase = BerPhase::SENDING;
        FwIndexType txRadio = 0;
        FwIndexType rxRadio = 1;
        RadioMode mode = RadioMode::NONE;
        U32 freqHz = 0;
        I8 powerDbm = 0;
        U32 totalPackets = 0;   //!< Packets to transmit
        U16 payloadLen = 0;     //!< Bytes per packet
        U32 intervalMs = 0;     //!< Minimum spacing between packets
        U32 sentCount = 0;      //!< Packets transmitted so far
        U32 recvCount = 0;      //!< Packets received so far
        U32 bitErrors = 0;      //!< Payload bit errors counted so far
        U32 totalBits = 0;      //!< Payload bits compared so far
        Fw::Time nextTxTime;    //!< Earliest time to send the next packet
        Fw::Time drainEnd;      //!< Deadline that ends the drain phase
        U8 pattern[BER_MAX_PAYLOAD];  //!< Reference PRBS9 payload pattern
    };

    // ----------------------------------------------------------------------
    // State
    // ----------------------------------------------------------------------

    RadioSlot m_radio[NUM_RADIOS];  //!< Per-radio state (driver contexts)
    BerTest m_ber;                  //!< Running / last BER test state
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
