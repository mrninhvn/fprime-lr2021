module LR2021 {

    @ RF power measured by the LT5538 detectors on the antenna coupler,
    @ fixed-point so the ground segment never has to decode floats.
    @ Divide each field by 10 to recover dBm.
    struct RfPower {
        FwdDbmX10: I16   @< Forward power [deci-dBm] at the detector input
        ReflDbmX10: I16  @< Reflected power [deci-dBm] at the detector input
    }

    @ F Prime manager for the Semtech LR2021 (LR20xx) transceivers.
    @ Manages both radio modules of the board on the shared SPI bus: each
    @ module has its own chip select and GPIO set. Both radios rest in
    @ continuous RX on their own frequency. The component implements the
    @ Svc.Com adapter interface directly (no ComStub): each downlink frame
    @ is routed to a radio by its frame context comQueueIndex through a
    @ route table configured from the topology (setTxRoute).
    active component LR2021Manager {

        # ----------------------------------------------------------------------
        # Types
        # ----------------------------------------------------------------------

        @ Radio modulation mode
        enum Mode {
            FLRC @< Fast Long Range Communication
            FSK @< GMSK Frequency Shift Keying
            CW @< Bench test: unmodulated carrier, no link (downlink frames dropped)
        }

        @ Downlink source queue routed by SET_TX_ROUTE. Values match the
        @ comQueueIndex the topology assigns each ComQueue queue (EVENTS=0,
        @ TELEMETRY=1, FILE=Svc.ComQueue.COM_PORT_COUNT+0=2): must be kept in
        @ sync with the ComCcsds subtopology's queue layout.
        enum RouteQueue {
            EVT @< Event packets (ComQueue EVENTS queue)
            TLM @< Telemetry packets (ComQueue TELEMETRY queue)
            FILE @< File downlink buffers (ComQueue FILE buffer queue)
        }

        @ Downlink route target: a radio index (0 or 1) or UART_RADIO (2, the
        @ byte-stream / USB CDC target). Kept as a plain U8 (not an enum) so
        @ the same wire value works whether NUM_RADIOS ever changes.


        # ----------------------------------------------------------------------
        # Com adapter interface (Svc.Com), routed by frame context.
        # dataIn is async (the interface declares it sync) so the slow radio
        # TX path runs on this component's thread, serialized with run().
        # ----------------------------------------------------------------------

        @ Downlink frame to transmit; routed to a radio by context.comQueueIndex
        async input port dataIn: Svc.ComDataWithContext

        @ Returns ownership of dataIn frames after TX (or on drop)
        output port dataReturnOut: Svc.ComDataWithContext

        @ Com status to the framer: ready / TX success / failure
        output port comStatusOut: Fw.SuccessCondition

        @ Received (uplink) data, from any radio, to the frame accumulator
        output port dataOut: Svc.ComDataWithContext

        @ Receives back ownership of buffers sent on dataOut
        sync input port dataReturnIn: Svc.ComDataWithContext

        @Allocate new buffer
        output port allocate: Fw.BufferGet

        @return the allocated buffer
        output port deallocate: Fw.BufferSend

        # ----------------------------------------------------------------------
        # Byte-stream client (optional wired UART downlink target)
        # ----------------------------------------------------------------------
        # A third downlink target besides the two radios: frames routed to
        # UART_RADIO are sent straight to a Drv.ByteStreamDriver (comDriver over
        # USB CDC). Mirrors what ComStub does internally, so no ComStub instance
        # is needed. Uplink bytes from the driver are forwarded to dataOut like
        # a radio RX. The driver allocates/deallocates from the same buffer
        # manager as our allocate/deallocate ports, so returned RX buffers are
        # freed directly in dataReturnIn (no separate recv-return port needed).

        @ Send a downlink frame out the byte-stream driver (synchronous)
        output port drvSendOut: Drv.ByteStreamSend

        @ Ready signal from the byte-stream driver
        sync input port drvConnected: Drv.ByteStreamReady

        @ Uplink data received from the byte-stream driver, forwarded to dataOut
        @ (like a radio RX) for the deframe / APID-router stack. Async so it runs
        @ on this component's thread.
        async input port drvReceiveIn: Drv.ByteStreamData

        # ----------------------------------------------------------------------
        # Relay TX (uplink routing target)
        # ----------------------------------------------------------------------

        @ Complete frame to relay out the radio, verbatim (fire-and-forget).
        @ Wired from the APID router's radioOut: frames whose APID routes to a
        @ satellite are transmitted here and the buffer is freed after TX.
        @ Async so radio TX runs on this component's thread, serialized with run().
        async input port relayIn: Svc.ComDataWithContext

        # ----------------------------------------------------------------------
        # PolyDb
        # ----------------------------------------------------------------------

        @ Publish LR2021 die temperature and last-packet RSSI to the PolyDb
        output port setPoly: Svc.Poly

        # ----------------------------------------------------------------------
        # Commands
        # ----------------------------------------------------------------------

        @ Reset one radio and re-initialise it, restoring its active mode
        async command RadioReset(
            radio: U8 @< Radio index (0 or 1)
        )

        @ Switch the active modulation of one radio at runtime
        @ (re-initialises the radio and enters continuous RX)
        async command RadioSetMode(
            radio: U8 @< Radio index (0 or 1)
            mode: Mode @< Modulation to activate
            freq_hz: U32 @< RF centre frequency in Hz (e.g. 437000000)
            power_dbm: I8 @< TX output power in dBm
        )

        @ Turn a radio module's load switch on or off. OFF cuts power to the
        @ chip (bench / fault-recovery use); bring it back with POWER ON
        @ followed by RESET (which re-runs radioInit + restores the mode).
        async command RadioPower(
            radio: U8 @< Radio index (0 or 1)
            power: Fw.On @< ON = power the module, OFF = cut power
        )

        @ Change only the modulation of a radio, keeping its last
        @ frequency / power (from the last SET_MODE or SET_FREQ / SET_POWER).
        @ Requires the radio to already have an active mode; use SET_MODE
        @ for the first configuration after boot / RESET / POWER ON.
        async command RadioSetModulation(
            radio: U8 @< Radio index (0 or 1)
            mode: Mode @< Modulation to activate
        )

        @ Change only the RF centre frequency of a radio, keeping its active
        @ mode and TX power. Requires the radio to already have an active mode.
        async command RadioSetFreq(
            radio: U8 @< Radio index (0 or 1)
            freq_hz: U32 @< RF centre frequency in Hz (e.g. 437000000)
        )

        @ Change only the TX output power of a radio, keeping its active mode
        @ and frequency. Requires the radio to already have an active mode.
        async command RadioSetPower(
            radio: U8 @< Radio index (0 or 1)
            power_dbm: I8 @< TX output power in dBm
        )

        @ Retarget a downlink source (events / telemetry / file) to a
        @ different radio or the UART at runtime, without touching the other
        @ two routes. Mirrors the topology's setTxRoute() call, callable from
        @ the ground. Takes effect on the next frame of that queue.
        async command RadioTxRoute(
            source: RouteQueue @< Downlink source to retarget
            target: U8 @< Radio index (0 or 1), or 2 for UART
        )

        @ Bench test: key a continuous PRBS9-modulated carrier (FSK or FLRC,
        @ whichever packet engine is requested) at freq_hz/power_dbm for
        @ duration_s seconds, then automatically restore normal continuous RX
        @ in that mode. mode must be FSK or FLRC (CW is rejected; use
        @ RadioSetMode for an unmodulated carrier instead). Non-blocking: the
        @ command returns as soon as the carrier is keyed (TxTestStarted);
        @ run() stops it and emits TxTestDone once the deadline passes. Only
        @ the tested radio is paused; the other keeps running. Bench /
        @ RF-characterisation only.
        async command RadioTxTest(
            radio: U8 @< Radio index (0 or 1)
            mode: Mode @< FSK or FLRC packet engine to key the test pattern on
            freq_hz: U32 @< RF centre frequency in Hz
            power_dbm: I8 @< TX output power in dBm
            duration_s: U32 @< How long to key the test TX, in seconds
        )

        @ Bench BER test: measure the bit error rate between two radio modules
        @ over the air. One radio (tx_radio) transmits num_packets frames of a
        @ fixed PRBS9 pattern; the other (rx_radio) receives them and the
        @ software counts bit errors against the known pattern. The raw path
        @ bypasses all CCSDS coding (no RS / CLTU / randomization) so the result
        @ is the true channel BER. Both radios are (re)configured to the same
        @ mode / freq / power on their own dedicated BER syncword, so they must
        @ be on the same band and coupled (coax + attenuator on the bench).
        @ mode must be FSK or FLRC (CW is rejected). Non-blocking: the command
        @ returns once the test is armed (BerTestStarted); run() paces the TX,
        @ tallies RX, and emits BerTestDone with the result when it finishes.
        @ Downlink frames routed to either radio are dropped while a test runs.
        async command RadioBerTest(
            tx_radio: U8 @< Transmitting radio index (0 or 1)
            rx_radio: U8 @< Receiving radio index (0 or 1), must differ from tx_radio
            mode: Mode @< FSK or FLRC packet engine for the test
            freq_hz: U32 @< RF centre frequency in Hz (same for both radios)
            power_dbm: I8 @< TX output power in dBm
            num_packets: U32 @< Number of test packets to transmit
            payload_len: U16 @< Test payload length per packet, in bytes
            interval_ms: U32 @< Minimum spacing between packets, in ms (0 = as fast as possible)
        )

        # ----------------------------------------------------------------------
        # Events
        # ----------------------------------------------------------------------

        @ Debug log message
        event LR2021(msg: string size 128) severity diagnostic format "{}"

        @ HAL / SPI operation failed
        event HalError(radio: U8, status: I32) severity warning high \
            format "LR2021 radio {} HAL error: {}" throttle 5

        @ Radio mode changed
        event ModeSet(radio: U8, mode: Mode) severity activity high \
            format "Radio {} mode set to {}"

        @ Invalid radio index in a command
        event BadRadioIndex(radio: U8) severity warning low \
            format "Invalid radio index {}"

        @ Radio module load switch turned on/off
        event PowerSet(radio: U8, power: Fw.On) severity activity high \
            format "Radio {} power set to {}"

        @ SET_RADIO_MODE / SET_FREQ / SET_POWER issued before the radio had an
        @ active mode (no prior SET_MODE / RESET with a stored mode)
        event RadioNotConfigured(radio: U8) severity warning low \
            format "Radio {} has no active mode; use SET_MODE first"

        @ Downlink route target changed at runtime
        event RouteSet(source: RouteQueue, target: U8) severity activity high \
            format "Downlink route {} set to target {}"

        @ SET_TX_ROUTE target out of range (not a valid radio index or UART)
        event BadRouteTarget(target: U8) severity warning low \
            format "Invalid route target {}"

        @ RadioTxTest started: keying the PRBS9 test carrier
        event TxTestStarted(radio: U8, mode: Mode, duration_s: U32) severity activity high \
            format "Radio {} TX test ({}) started for {} s"

        @ RadioTxTest finished: normal continuous RX restored
        event TxTestDone(radio: U8) severity activity high \
            format "Radio {} TX test done, RX restored"

        @ RadioTxTest given a mode other than FSK/FLRC (e.g. CW), or the test
        @ carrier / RX restore failed to key
        event TxTestError(radio: U8) severity warning high \
            format "Radio {} TX test failed (invalid mode or radio error)"

        @ RadioBerTest armed: transmitting the PRBS9 test packets
        event BerTestStarted(tx_radio: U8, rx_radio: U8, mode: Mode, num_packets: U32) \
            severity activity high \
            format "BER test started: radio {} -> {} ({}), {} packets"

        @ RadioBerTest finished: reports the measured bit error rate
        event BerTestDone(tx_radio: U8, rx_radio: U8, sent: U32, received: U32, \
                          lost: U32, bit_errors: U32, total_bits: U32, ber_ppm: U32) \
            severity activity high \
            format "BER test done: radio {} -> {}, sent {}, rx {}, lost {}, bit errors {}/{}, BER {} ppm"

        @ RadioBerTest given a bad mode / radio pair, or a radio operation
        @ failed while arming or running the test
        event BerTestError(tx_radio: U8, rx_radio: U8) severity warning high \
            format "BER test failed (radio {} -> {}): invalid args or radio error"

        @ A downlink frame could not be transmitted and was dropped
        event TxFrameDropped(radio: U8) severity warning high \
            format "TX frame dropped (radio {} not ready)" throttle 5

        @ FLRC packet transmission completed
        event FlrcTxDone(radio: U8) severity activity high \
            format "FLRC TX done (radio {})"

        @ FLRC packet received
        event FlrcRxPacket(radio: U8, length: U16, rssi: I16) severity activity high \
            format "FLRC RX packet (radio {}): {} bytes, RSSI {} dBm"

        @ FLRC radio error (timeout / CRC / length), raw IRQ mask
        event FlrcError(radio: U8, irq: U32) severity warning high \
            format "FLRC radio {} error, IRQ mask 0x{x}" throttle 5

        @ FSK packet transmission completed
        event FskTxDone(radio: U8) severity activity high \
            format "FSK TX done (radio {})"

        @ FSK packet received
        event FskRxPacket(radio: U8, length: U16, rssi: I16) severity activity high \
            format "FSK RX packet (radio {}): {} bytes, RSSI {} dBm"

        @ FSK radio error (timeout / CRC / length), raw IRQ mask
        event FskError(radio: U8, irq: U32) severity warning high \
            format "FSK radio {} error, IRQ mask 0x{x}" throttle 5

        # ----------------------------------------------------------------------
        # Telemetry
        # ----------------------------------------------------------------------

        @ Count of FLRC packets transmitted
        telemetry FlrcTxCount: U32 update on change

        @ Count of FLRC packets received
        telemetry FlrcRxCount: U32 update on change

        @ RSSI of the last received FLRC packet, in dBm
        telemetry FlrcRssi: I16 update on change

        @ Count of FSK packets transmitted
        telemetry FskTxCount: U32 update on change

        @ Count of FSK packets received
        telemetry FskRxCount: U32 update on change

        @ RSSI of the last received FSK packet, in dBm
        telemetry FskRssi: I16 update on change

        @ Forward / reflected RF power on the LR2021 antenna line, measured
        @ by the LT5538 detectors on the PCB directional coupler (rf21 =
        @ forward, rf2 = reflected). Sampled during each TX, right after the
        @ PA ramps up. High reflected vs forward power indicates a bad
        @ antenna match / disconnected antenna.
        telemetry RfPower: RfPower update on change

        @ Total bit errors counted in the last / running BER test
        telemetry BerBitErrors: U32 update on change

        @ Total payload bits compared in the last / running BER test
        telemetry BerBitsTotal: U32 update on change

        @ Bit error rate of the last BER test, in parts per million (errors/bits)
        telemetry BerRatePpm: U32 update on change

        @ Packets received during the last / running BER test
        telemetry BerPacketsRecv: U32 update on change

        @ Packets transmitted but not received during the last BER test
        telemetry BerPacketsLost: U32 update on change

        # ----------------------------------------------------------------------
        # Radio interface ports
        # ----------------------------------------------------------------------

        @ Rate-group input for periodic servicing (IRQ / RX polling).
        @ Async so the SPI polling runs on this component's thread.
        @ Drop on queue overflow: ticks may pile up while a slow radio
        @ operation (busy-wait) blocks the thread; missing one is harmless.
        async input port run: Svc.Sched drop

        # Hardware port arrays: index = radio index (0 / 1), each element
        # wired to that module's SPI chip-select / GPIO driver.

        @ SPI bus ports (each a ZephyrSpiDriver with that radio's chip select)
        output port spiWriteRead: [2] Drv.SpiWriteRead

        @ Power (load-switch EN) GPIO control, active high
        output port powerGpioWrite: [2] Drv.GpioWrite

        @ Reset (NRESET) GPIO control, active low
        output port resetGpioWrite: [2] Drv.GpioWrite

        @ BUSY lines, read to know when the radio can take a command
        output port busyGpioRead: [2] Drv.GpioRead

        @ IRQ lines. Read from the run handler to skip the SPI IRQ-status
        @ poll when no IRQ is pending. Optional: when unconnected the
        @ component polls IRQ status over SPI every run tick.
        output port irqGpioRead: [2] Drv.GpioRead

        # ----------------------------------------------------------------------
        # Standard AC Ports: Required for Channels, Events, Commands, Parameters
        # ----------------------------------------------------------------------
        @ Port for requesting the current time
        time get port timeCaller

        @ Port for sending command registrations
        command reg port cmdRegOut

        @ Port for receiving commands
        command recv port cmdIn

        @ Port for sending command responses
        command resp port cmdResponseOut

        @ Port for sending textual representation of events
        text event port logTextOut

        @ Port for sending events to downlink
        event port logOut

        @ Port for sending telemetry channels to downlink
        telemetry port tlmOut

    }
}
