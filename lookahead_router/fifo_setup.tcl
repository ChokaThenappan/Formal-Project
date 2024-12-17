clear -all

analyze -sv09 noc_pkg.sv
analyze -sv09 router_fifo.sv
analyze -sv09 lookahead_routing.sv
analyze -sv09 router_arbiter.sv
analyze -sv09 lookahead_router.sv
analyze -sv09 lookahead_routing.sv
elaborate -top lookahead_router

clock clk
reset rst
