open_hw
connect_hw_server
open_hw_target

set device [lindex [get_hw_devices xc7z020*] 0]
refresh_hw_device $device

set_property PROGRAM.FILE spi_proj.runs/impl_1/spi_slave_test.bit $device

program_hw_devices $device
