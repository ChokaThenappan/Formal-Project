clear -all

analyze -sv09 fifo.sv
elaborate -top fifo

clock clk
reset ~rst_n
