## Logic-analyser timing pins on Pmod JE (Zybo Z7-20)
## "la_out_0" = the external port made with "Make External" on rk4_axi_0/la_out
##   JE1 (V12) = FPGA busy      JE2 (W16) = ARM busy (software RK4)
##   JE pin 5 or 11 = GND,  pin 6 / 12 = 3.3 V (do not connect)
set_property -dict { PACKAGE_PIN V12 IOSTANDARD LVCMOS33 } [get_ports { la_out_0[0] }]
set_property -dict { PACKAGE_PIN W16 IOSTANDARD LVCMOS33 } [get_ports { la_out_0[1] }]
set_false_path -to [get_ports { la_out_0[*] }]
