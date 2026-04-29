# SPI (PMODA)

#set_property PACKAGE_PIN Y14 [get_ports rst]
#set_property IOSTANDARD LVCMOS33 [get_ports rst]

set_property PACKAGE_PIN Y18 [get_ports sclk]
set_property IOSTANDARD LVCMOS33 [get_ports sclk]
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets sclk_IBUF]

set_property PACKAGE_PIN Y19 [get_ports mosi]
set_property IOSTANDARD LVCMOS33 [get_ports mosi]

set_property PACKAGE_PIN Y16 [get_ports miso]
set_property IOSTANDARD LVCMOS33 [get_ports miso]

set_property PACKAGE_PIN Y17 [get_ports cs]
set_property IOSTANDARD LVCMOS33 [get_ports cs]

## CLOCK

#set_property PACKAGE_PIN H16 [get_ports clk]
#set_property IOSTANDARD LVCMOS33 [get_ports clk]

#create_clock -period 10.0 [get_ports clk]

## DEBUG

set_property PACKAGE_PIN W18 [get_ports done]
set_property IOSTANDARD LVCMOS33 [get_ports done]

set_property PACKAGE_PIN W19 [get_ports {debug_received[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {debug_received[0]}]

set_property PACKAGE_PIN U18 [get_ports {debug_received[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {debug_received[1]}]

set_property PACKAGE_PIN U19 [get_ports {debug_received[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {debug_received[2]}]
