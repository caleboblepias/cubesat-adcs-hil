# ================================================================
# PYNQ-Z1 ADCS HIL
# ================================================================


# ----------------------------------------------------------------
# 125 MHz PL clock
# ----------------------------------------------------------------

set_property -dict { PACKAGE_PIN H16 IOSTANDARD LVCMOS33 } \
    [get_ports {clk_125mhz}]

create_clock -add \
    -name sys_clk \
    -period 8.000 \
    -waveform {0.000 4.000} \
    [get_ports {clk_125mhz}]


# ----------------------------------------------------------------
# Reset
#
# SW0 = M20
# Logic high asserts ADCS HIL reset.
#
# reset_sw is asynchronous to sys_clk. The hardware wrapper
# performs asynchronous assertion and synchronous deassertion.
# ----------------------------------------------------------------

set_property -dict { PACKAGE_PIN M20 IOSTANDARD LVCMOS33 } \
    [get_ports {reset_sw}]


# ----------------------------------------------------------------
# SPI - PMODA
#
# JA0 = CS
# JA1 = SCLK
# JA2 = MOSI
# JA3 = MISO
#
# SPI Mode 0
# CS active low
# Target SCLK approximately 1 MHz
# ----------------------------------------------------------------

# JA0 - CS
set_property -dict { PACKAGE_PIN Y18 IOSTANDARD LVCMOS33 } \
    [get_ports {spi_cs_n}]

# JA1 - SCLK
set_property -dict { PACKAGE_PIN Y19 IOSTANDARD LVCMOS33 } \
    [get_ports {spi_sclk}]

# JA2 - MOSI
set_property -dict { PACKAGE_PIN Y16 IOSTANDARD LVCMOS33 } \
    [get_ports {spi_mosi}]

# JA3 - MISO
set_property -dict { PACKAGE_PIN Y17 IOSTANDARD LVCMOS33 } \
    [get_ports {spi_miso}]


# ----------------------------------------------------------------
# SPI CDC constraints
#
# spi_cs_n, spi_sclk, and spi_mosi are asynchronous to sys_clk.
#
# Each input enters an explicit two-flop synchronizer marked
# ASYNC_REG in spi_slave.sv.
#
# Only the path from the asynchronous package input to the first
# synchronizer stage is excluded from synchronous timing analysis.
# The first-stage -> second-stage synchronizer paths remain timed.
# ----------------------------------------------------------------

set_false_path \
    -from [get_ports {spi_cs_n}] \
    -to [get_cells {adcs_hil_inst/spi_slave_inst/cs_meta_reg}]

set_false_path \
    -from [get_ports {spi_sclk}] \
    -to [get_cells {adcs_hil_inst/spi_slave_inst/sclk_meta_reg}]

set_false_path \
    -from [get_ports {spi_mosi}] \
    -to [get_cells {adcs_hil_inst/spi_slave_inst/mosi_meta_reg}]


# ----------------------------------------------------------------
# MISO timing
#
# spi_miso is a real externally observable SPI output.
# Do not false-path it merely to suppress check_timing warnings.
#
# External SPI output timing can be constrained separately once
# the STM32 master timing requirements are incorporated.
# ----------------------------------------------------------------
