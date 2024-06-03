#!/bin/bash
#openocd -f interface/picoprobe.cfg -f target/rp2040.cfg -c "program $1 verify reset exit"
##openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -s tcl -c "program $1 verify reset exit"
openocd -s tcl -c "adapter speed 5000" -f interface/cmsis-dap.cfg  -f target/rp2040.cfg -c "program $1 verify reset exit"

