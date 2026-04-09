create_project spi_proj . -part xc7z020clg400-1 -force

add_files spi_slave.sv
add_files spi_slave_test.sv
add_files -fileset constrs_1 constraints.xdc

set_property top spi_slave_test [current_fileset]

launch_runs synth_1 -jobs 4
wait_on_run synth_1

launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1
