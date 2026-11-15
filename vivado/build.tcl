# ---------------------------------------------------------------------------
# Builds the complete Zybo Z7-20 project:
#   Zynq PS (board preset, UART1 = USB) --M_AXI_GP0--> rk4_axi, FCLK0 = 100 MHz
#   rk4_axi la_out -> Pmod JE1/JE2 (timing pins for the logic analyser)
# then runs synthesis + implementation + bitstream and exports rk4_zybo.xsa.
#
# Usage (Vivado Tcl console):
#     cd <path-to>/rk4_zybo/vivado
#     source build.tcl
# ---------------------------------------------------------------------------
set here     [file normalize [file dirname [info script]]]
set root     [file normalize $here/..]
set proj     rk4_zybo
set proj_dir $here/$proj

create_project $proj $proj_dir -part xc7z020clg400-1 -force

# Digilent board files give the correct DDR / UART / clock settings
set bp [lindex [lsort [get_board_parts -quiet *zybo-z7-20*]] end]
if {$bp eq ""} {
    error "Zybo Z7-20 board files not found. Install them (Tools > Vivado Store > Boards > Digilent, or copy Digilent's vivado-boards repo) and re-run."
}
set_property board_part $bp [current_project]
puts "Using board part $bp"

# ---- sources ----
add_files -norecurse [list $root/rtl/rk4_core.v $root/rtl/rk4_engine.v $root/rtl/rk4_axi.v]
add_files -fileset constrs_1 -norecurse $here/rk4_la.xdc
add_files -fileset sim_1 -norecurse [concat [list $root/sim/tb_rk4_axi.v] [glob $root/sim/*.mem]]
set_property top tb_rk4_axi [get_filesets sim_1]
set_property -name {xsim.simulate.runtime} -value {all} -objects [get_filesets sim_1]
update_compile_order -fileset sources_1

# ---- block design ----
create_bd_design system
create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 ps7
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 \
    -config {make_external "FIXED_IO, DDR" apply_board_preset "1" Master "Disable" Slave "Disable"} [get_bd_cells ps7]
set_property -dict [list CONFIG.PCW_USE_M_AXI_GP0 {1} CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100}] [get_bd_cells ps7]

create_bd_cell -type module -reference rk4_axi rk4_0
apply_bd_automation -rule xilinx.com:bd_rule:axi4 \
    -config {Clk_master {Auto} Clk_slave {Auto} Clk_xbar {Auto} Master {/ps7/M_AXI_GP0} Slave {/rk4_0/s_axi} intc_ip {New AXI Interconnect} master_apm {0}} \
    [get_bd_intf_pins rk4_0/s_axi]

# logic-analyser pins -> external port la_out_0 (pinned to Pmod JE by rk4_la.xdc)
make_bd_pins_external [get_bd_pins rk4_0/la_out]

assign_bd_address
validate_bd_design
save_bd_design

add_files -norecurse [make_wrapper -files [get_files system.bd] -top]
set_property top system_wrapper [current_fileset]
update_compile_order -fileset sources_1

# ---- build ----
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1
open_run impl_1
report_utilization    -file $here/utilization.rpt
report_timing_summary -file $here/timing.rpt
puts "WNS = [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]] ns (must be >= 0)"

write_hw_platform -fixed -include_bit -force -file $here/rk4_zybo.xsa
puts "DONE -> $here/rk4_zybo.xsa"
