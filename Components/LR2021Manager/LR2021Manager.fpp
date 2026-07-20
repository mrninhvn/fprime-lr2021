module LR2021 {

    @ RF power measured by the LT5538 detectors on the antenna coupler,
    @ fixed-point so the ground segment never has to decode floats.
    @ Divide each field by 10 to recover dBm.
    struct RfPower {
        FwdDbmX10: I16   @< Forward power [deci-dBm] at the detector input
        ReflDbmX10: I16  @< Reflected power [deci-dBm] at the detector input
    }

    @ F Prime manager for the Semtech LR2021 (LR20xx) transceiver.
    @ Wraps the lr20xx_driver C driver and drives it over a Zephyr SPI bus
    @ plus reset / busy GPIO lines.
    active component LR2021Manager {

        # ----------------------------------------------------------------------
        # Types
        # ----------------------------------------------------------------------

        @ Radio modulation mode
        enum Mode {
            FLRC @< Fast Long Range Communication
            FSK @< GMSK Frequency Shift Keying
        }

        @ Port invoked when the driver is ready to send/receive data
        output port ready: Drv.ByteStreamReady

        @ Port invoked by the driver when it receives data
        output port $recv: Drv.ByteStreamData

        @ Port receiving back ownership of data sent out on $recv port
        guarded input port recvReturnIn: Fw.BufferSend

        @ ComStub async
        async input port asyncSendIn: Fw.BufferSend

        @ ComStub return buffer
        output port asyncSendReturnIn: Drv.ByteStreamData

        @Allocate new buffer
        output port allocate: Fw.BufferGet

        @return the allocated buffer
        output port deallocate: Fw.BufferSend

        # ----------------------------------------------------------------------
        # Commands
        # ----------------------------------------------------------------------

        @ Reset the LR2021 radio and re-initialise it, restoring the active mode
        async command RESET

        @ Switch the active modulation at runtime (re-initialises the radio
        @ and enters continuous RX)
        async command SET_MODE(
            mode: Mode @< Modulation to activate
            freq_hz: U32 @< RF centre frequency in Hz (e.g. 2444000000)
            power_dbm: I8 @< TX output power in dBm
        )

        # ----------------------------------------------------------------------
        # Events
        # ----------------------------------------------------------------------

        @ Debug log message
        event LR2021(msg: string size 128) severity diagnostic format "{}"

        @ HAL / SPI operation failed
        event HalError(status: I32) severity warning high \
            format "LR2021 HAL error: {}" throttle 5

        @ Radio mode changed
        event ModeSet(mode: Mode) severity activity high \
            format "Radio mode set to {}"

        @ FLRC packet transmission completed
        event FlrcTxDone() severity activity high format "FLRC TX done"

        @ FLRC packet received
        event FlrcRxPacket(length: U16, rssi: I16) severity activity high \
            format "FLRC RX packet: {} bytes, RSSI {} dBm"

        @ FLRC radio error (timeout / CRC / length), raw IRQ mask
        event FlrcError(irq: U32) severity warning high \
            format "FLRC radio error, IRQ mask 0x{x}" throttle 5

        @ FSK packet transmission completed
        event FskTxDone() severity activity high format "FSK TX done"

        @ FSK packet received
        event FskRxPacket(length: U16, rssi: I16) severity activity high \
            format "FSK RX packet: {} bytes, RSSI {} dBm"

        @ FSK radio error (timeout / CRC / length), raw IRQ mask
        event FskError(irq: U32) severity warning high \
            format "FSK radio error, IRQ mask 0x{x}" throttle 5

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

        @ SPI bus port (connected to a ZephyrSpiDriver instance)
        output port spiWriteRead: Drv.SpiWriteRead

        @ Power GPIO control, active high
        output port powerGpioWrite: Drv.GpioWrite

        @ Reset (NRESET) GPIO control, active low
        output port resetGpioWrite: Drv.GpioWrite

        @ BUSY line, read to know when the radio is ready for a command
        output port busyGpioRead: Drv.GpioRead

        @ IRQ line (radio DIO7). Read from the run handler to skip the SPI
        @ IRQ-status poll when no IRQ is pending. Optional: when unconnected
        @ the component polls IRQ status over SPI every run tick.
        output port irqGpioRead: Drv.GpioRead

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
