if {[file exists spi_proj.xpr]} {
    puts "Opening existing project..."
    open_project spi_proj.xpr
} else {
    puts "Creating new project..."
    create_project spi_proj . -part xc7z020clg400-1
}

# ================= FILES =================

# Remove old files (prevents duplicates / stale top issues)
remove_files [get_files *spi_slave_test.sv*] -quiet

# Add required design files
add_files spi_slave_sclk.sv
add_files spi_protocol.sv
add_files spi_protocol_test.sv

# Constraints
add_files -fileset constrs_1 constraints.xdc

# ================= TOP =================

set_property top spi_protocol_test [current_fileset]

# ================= BUILD =================

reset_run synth_1
reset_run impl_1

launch_runs synth_1 -jobs 4
wait_on_run synth_1

launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1
