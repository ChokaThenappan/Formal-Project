# ----------------------------------------
# Jasper Version Info
# tool      : Jasper 2024.06
# platform  : Linux 4.18.0-553.27.1.el8_10.x86_64
# version   : 2024.06p002 64 bits
# build date: 2024.09.02 16:28:38 UTC
# ----------------------------------------
# started   : 2024-12-13 12:02:38 EST
# hostname  : cadpc16.(none)
# pid       : 378664
# arguments : '-label' 'session_0' '-console' '//127.0.0.1:38149' '-style' 'windows' '-data' 'AAAAhHicY2RgYLCp////PwMYMD6A0Aw2jAyoAMRnQhUJbEChGRhYYZqRNckwpDFkAnE+QzxDMUMqQwlDKUMBgx6QTmbIAasCAD8RDII=' '-proj' '/homes/user/stud/fall23/ct3185/Formal-Project/jgproject/sessionLogs/session_0' '-init' '-hidden' '/homes/user/stud/fall23/ct3185/Formal-Project/jgproject/.tmp/.initCmds.tcl' 'fifo_setup.tcl'
clear -all

analyze -sv09 router_fifo.sv
elaborate -top router_fifo

clock clk
reset rst
prove -bg -all
include fifo_setup.tcl
prove -bg -all
visualize -violation -property <embedded>::router_fifo.a_no_read_when_empty -new_window
visualize -min_length [expr [visualize -get_length -window visualize:0] + 4] -window visualize:0; visualize -freeze [visualize -get_length -window visualize:0] -window visualize:0; visualize -replot -bg -window visualize:0
visualize -violation -property <embedded>::router_fifo.a_no_write_when_full -new_window
include fifo_setup.tcl
prove -bg -all
include fifo_setup.tcl
prove -bg -all
include fifo_setup.tcl
prove -bg -all
include fifo_setup.tcl
include fifo_setup.tcl
prove -bg -all
include fifo_setup.tcl
prove -bg -all
prove -bg -all
prove -bg -all
include fifo_setup.tcl
prove -bg -all
include fifo_setup.tcl
prove -bg -all
include fifo_setup.tcl
prove -bg -all
include fifo_setup.tcl
prove -bg -all
