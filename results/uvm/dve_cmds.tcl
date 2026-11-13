set f [open dve_cmds.txt w]; puts $f [lsort [info commands gui_*]]; close $f; quit
