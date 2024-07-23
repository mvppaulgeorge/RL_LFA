# delay calc example
read_liberty /Users/evan/Documents/Drills/adder_file/tech.lib
read_verilog playground/50/50-mapped.v
link_design adder

create_clock -name clk -period 10
set_input_delay 0 -clock clk [get_ports {a[0] a[1] a[2] a[3] a[4] a[5] a[6] a[7] a[8] a[9] a[10] a[11] a[12] a[13] a[14] a[15] a[16] a[17] a[18] a[19] a[20] a[21] a[22] a[23] a[24] a[25] a[26] a[27] a[28] a[29] a[30] a[31] b[0] b[1] b[2] b[3] b[4] b[5] b[6] b[7] b[8] b[9] b[10] b[11] b[12] b[13] b[14] b[15] b[16] b[17] b[18] b[19] b[20] b[21] b[22] b[23] b[24] b[25] b[26] b[27] b[28] b[29] b[30] }]

set_output_delay 0 -clock clk [get_ports {s[0] s[1] s[2] s[3] s[4] s[5] s[6] s[7] s[8] s[9] s[10] s[11] s[12] s[13] s[14] s[15] s[16] s[17] s[18] s[19] s[20] s[21] s[22] s[23] s[24] s[25] s[26] s[27] s[28] s[29] s[30] s[31]}]
report_checks
report_power
