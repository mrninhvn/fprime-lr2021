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
        }

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

        @ Uplink data received from the byte-stream driver. In flight it is
        @ forwarded to dataOut (like a radio RX); on a ground relay (RxSink
        @ UART) it is transmitted over the radio instead. Async so that radio
        @ TX runs on this component's thread, serialized with run().
        async input port drvReceiveIn: Drv.ByteStreamData

        # ----------------------------------------------------------------------
        # Commands
        # ----------------------------------------------------------------------

        @ Reset one radio and re-initialise it, restoring its active mode
        async command RESET(
            radio: U8 @< Radio index (0 or 1)
        )

        @ Switch the active modulation of one radio at runtime
        @ (re-initialises the radio and enters continuous RX)
        async command SET_MODE(
            radio: U8 @< Radio index (0 or 1)
            mode: Mode @< Modulation to activate
            freq_hz: U32 @< RF centre frequency in Hz (e.g. 437000000)
            power_dbm: I8 @< TX output power in dBm
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
