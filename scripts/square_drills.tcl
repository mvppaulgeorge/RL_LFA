# Script generated by DRiLLS agent

read asap7.lib
read square.v

strash
rewrite -z
refactor -z
balance
balance
resub
resub
resub
refactor
rewrite -z
resub -z
resub
balance
refactor -z
resub
refactor -z

write_verilog square_synth_drills.v

map -D 2200
stime
