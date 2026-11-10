#!/bin/bash
set -e

VIVADO=/home/caleboblepias/Xilinx/Vivado/2021.1
PROJECT=/home/caleboblepias/cubesat-adcs-hil/hardware/linux-vm/adcs_hil.xpr
BITSTREAM=/home/caleboblepias/cubesat-adcs-hil/hardware/linux-vm/adcs_hil.runs/impl_1/adcs_hil_hw_top.bit

echo "========================================"
echo " ADCS HIL FPGA Build"
echo "========================================"

source "$VIVADO/settings64.sh"

# ------------------------------------------------------------
# Generate temporary Vivado build script
# ------------------------------------------------------------

cat > /tmp/adcs_hil_build.tcl <<EOF
open_project $PROJECT

puts "========================================"
puts " SYNTHESIS"
puts "========================================"

reset_run synth_1
launch_runs synth_1 -jobs 4
wait_on_run synth_1

puts "Synthesis status:"
puts [get_property STATUS [get_runs synth_1]]

puts "========================================"
puts " IMPLEMENTATION + BITSTREAM"
puts "========================================"

reset_run impl_1
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1

puts "Implementation status:"
puts [get_property STATUS [get_runs impl_1]]

exit
EOF

vivado -mode batch -source /tmp/adcs_hil_build.tcl

echo
echo "========================================"
echo " BUILD COMPLETE"
echo "========================================"

if [ ! -f "$BITSTREAM" ]; then
    echo "ERROR: Bitstream not found:"
    echo "$BITSTREAM"
    exit 1
fi

ls -lh "$BITSTREAM"

echo
echo "Bitstream ready:"
echo "$BITSTREAM"
