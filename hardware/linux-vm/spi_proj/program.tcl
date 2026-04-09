open_hw
connect_hw_server
open_hw_target

# Select first device
set device [lindex [get_hw_devices] 0]

# Refresh device
refresh_hw_device $device

# Set bitstream
set_property PROGRAM.FILE spi_slave_test.bit $device

# Program FPGA
program_hw_devices $device
