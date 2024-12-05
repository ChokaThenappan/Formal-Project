clear -all

analyze -sv09 router_fifo.sv
elaborate -top router_fifo

clock clk
reset rst
