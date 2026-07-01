module LR2021 {
    @ F Prime manager for the Semtech LR2021 (LR20xx) transceiver.
    @ Wraps the lr20xx_driver C driver and drives it over a Zephyr SPI bus
    @ plus reset / busy GPIO lines.
    active component LR2021Manager {

        # ----------------------------------------------------------------------
        # Commands
        # ----------------------------------------------------------------------

        @ Reset the LR2021 radio (toggles the NRESET line and re-initialises)
        async command RESET

        @ Read and report the LR2021 firmware version over SPI
        async command GET_VERSION

        # ----------------------------------------------------------------------
        # Events
        # ----------------------------------------------------------------------

        @ Debug log message
        event Debug(msg: string size 200) severity diagnostic format "LR2021: {}"

        @ Reported firmware version
        event Version(major: U8, minor: U8) severity activity high \
            format "LR2021 version {}.{}"

        @ HAL / SPI operation failed
        event HalError(status: I32) severity warning high \
            format "LR2021 HAL error: {}" throttle 5

        # ----------------------------------------------------------------------
        # Telemetry
        # ----------------------------------------------------------------------

        # ----------------------------------------------------------------------
        # Radio interface ports
        # ----------------------------------------------------------------------

        @ Rate-group input for periodic servicing (e.g. IRQ / RX polling)
        sync input port run: Svc.Sched

        @ SPI bus port (connected to a ZephyrSpiDriver instance)
        output port spiWriteRead: Drv.SpiWriteRead

        @ Reset (NRESET) GPIO control, active low
        output port resetGpioWrite: Drv.GpioWrite

        @ BUSY line, read to know when the radio is ready for a command
        output port busyGpioRead: Drv.GpioRead

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
