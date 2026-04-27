# Standalone signoff target: 1.1 x 125 MHz = 137.5 MHz => 7.273 ns
create_clock -name clk125 -period 7.273 [get_ports {clk125}]

set_false_path -from [get_ports {reset_n}]
set_false_path -to [get_ports {probe_out[*]}]
