####
# LR2021Manager component
#
# Wraps the Semtech lr20xx_driver C driver as an F Prime component and drives it
# over a Zephyr SPI bus (Zephyr.ZephyrSpiDriver) plus reset / busy GPIO lines.
####

# Root of the vendored LR20xx radio driver (from the LR2021Hal submodule)
set(LR20XX_DRIVER
    "${CMAKE_CURRENT_LIST_DIR}/../LR2021Hal/smtc_rac_lib/radio_drivers/lr20xx_driver")

# Core driver sources plus the commonly used LoRa / FSK modulation drivers.
# Add more lr20xx_radio_*.c files here to enable other modulations.
set(lr20xx_src
    "${LR20XX_DRIVER}/src/lr20xx_driver_version.c"
    "${LR20XX_DRIVER}/src/lr20xx_radio_common.c"
    "${LR20XX_DRIVER}/src/lr20xx_radio_fifo.c"
    "${LR20XX_DRIVER}/src/lr20xx_regmem.c"
    "${LR20XX_DRIVER}/src/lr20xx_system.c"
    "${LR20XX_DRIVER}/src/lr20xx_workarounds.c"
    "${LR20XX_DRIVER}/src/lr20xx_radio_lora.c"
    "${LR20XX_DRIVER}/src/lr20xx_radio_fsk.c"
)

include_directories(
    "${LR20XX_DRIVER}/inc"
)

register_fprime_module(
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/LR2021Manager.fpp"
    SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/LR2021Manager.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/LR2021Hal.cpp"
        ${lr20xx_src}
    DEPENDS
        Fw_Types
        Drv_Ports
)

# The vendored driver files are C, not C++.
set_source_files_properties(
    ${lr20xx_src}
    PROPERTIES LANGUAGE C
)
