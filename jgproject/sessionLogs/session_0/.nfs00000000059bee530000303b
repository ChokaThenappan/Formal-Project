# ----------------------------------------
# Jasper Version Info
# tool      : Jasper 2024.06
# platform  : Linux 4.18.0-553.27.1.el8_10.x86_64
# version   : 2024.06p002 64 bits
# build date: 2024.09.02 16:28:38 UTC
# ----------------------------------------
# started   : 2024-11-18 14:43:59 EST
# hostname  : cadpc13.(none)
# pid       : 1441634
# arguments : '-label' 'session_0' '-console' '//127.0.0.1:44285' '-style' 'windows' '-data' 'AAAAknicY2RgYLCp////PwMYMD6A0Aw2jAyoAMRnQhUJbEChGRhYYZqRNWkxFDHkM5QylDCkAlnxDGkMmUCcD2QVA0VKgDIFDHpAOpkhB6wHABpcD5A=' '-proj' '/homes/user/stud/fall23/ct3185/Formal-Project/jgproject/sessionLogs/session_0' '-init' '-hidden' '/homes/user/stud/fall23/ct3185/Formal-Project/jgproject/.tmp/.initCmds.tcl' 'router_fifo_setup.tcl'
clear -all

analyze -sv09 router_fifo.sv
elaborate -top router_fifo

clock clk
reset rst
prove -bg -all
