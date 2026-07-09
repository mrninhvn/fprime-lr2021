module LR2021 {
    @ F Prime manager for the Semtech LR2021 (LR20xx) transceiver.
    @ Wraps the lr20xx_driver C driver and drives it over a Zephyr SPI bus
    @ plus reset / busy GPIO lines.
    active component LR2021Manager {

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

        @ Reset the LR2021 radio (toggles the NRESET line and re-initialises)
        async command RESET

        @ Configure the radio for FLRC operation (2.4 GHz band)
        async command FLRC_INIT(
            freq_hz: U32 @< RF centre frequency in Hz (e.g. 2444000000)
            power_dbm: I8 @< TX output power in dBm
        )

        @ Transmit a payload using FLRC (radio must be FLRC-initialised)
        async command FLRC_TX(
            data: string size 200 @< Payload to transmit
        )

        @ Enter FLRC receive mode
        async command FLRC_RX(
            timeout_ms: U32 @< RX timeout in ms; 0 for continuous RX
        )

        # ----------------------------------------------------------------------
        # Events
        # ----------------------------------------------------------------------

        @ Debug log message
        event LR2021(msg: string size 128) severity diagnostic format "{}"

        @ HAL / SPI operation failed
        event HalError(status: I32) severity warning high \
            format "LR2021 HAL error: {}" throttle 5

        @ FLRC packet transmission completed
        event FlrcTxDone() severity activity high format "FLRC TX done"

        @ FLRC packet received
        event FlrcRxPacket(length: U16, rssi: I16) severity activity high \
            format "FLRC RX packet: {} bytes, RSSI {} dBm"

        @ FLRC radio error (timeout / CRC / length), raw IRQ mask
        event FlrcError(irq: U32) severity warning high \
            format "FLRC radio error, IRQ mask 0x{x}" throttle 5

        # ----------------------------------------------------------------------
        # Telemetry
        # ----------------------------------------------------------------------

        @ Count of FLRC packets transmitted
        telemetry FlrcTxCount: U32 update on change

        @ Count of FLRC packets received
        telemetry FlrcRxCount: U32 update on change

        @ RSSI of the last received FLRC packet, in dBm
        telemetry FlrcRssi: I16 update on change

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
