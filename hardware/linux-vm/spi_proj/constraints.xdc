## SPI (PMODA)

set_property PACKAGE_PIN Y14 [get_ports rst]
set_property IOSTANDARD LVCMOS33 [get_ports rst]

set_property PACKAGE_PIN Y18 [get_ports sclk]
set_property IOSTANDARD LVCMOS33 [get_ports sclk]

set_property PACKAGE_PIN Y19 [get_ports mosi]
set_property IOSTANDARD LVCMOS33 [get_ports mosi]

set_property PACKAGE_PIN Y16 [get_ports miso]
set_property IOSTANDARD LVCMOS33 [get_ports miso]

set_property PACKAGE_PIN Y17 [get_ports cs]
set_property IOSTANDARD LVCMOS33 [get_ports cs]

## CLOCK

set_property PACKAGE_PIN H16 [get_ports clk]
set_property IOSTANDARD LVCMOS33 [get_ports clk]

create_clock -period 10.0 [get_ports clk]
