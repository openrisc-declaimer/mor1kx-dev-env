######################################################################
####                                                              ####
####  ORPSoCv2 Testbenches Makefile                               ####
####                                                              ####
####  Description                                                 ####
####  ORPSoCv2 Testbenches Makefile, containing rules for         ####
####  configuring and running different tests on the current      ####
####  ORPSoC(v2) design.                                          ####
####                                                              ####
####  To do:                                                      ####
####                                                              ####
####  Author(s):                                                  ####
####      - Jin Fei, jm8371@gmail.com                             ####
####                                                              ####
####                                                              ####
######################################################################
####                                                              ####
#### Copyright (C) 2017 Authors and JIN FEI                       ####
####                                                              ####
#### This source file may be used and distributed without         ####
#### restriction provided that this copyright statement is not    ####
#### removed from the file and that any derivative work contains  ####
#### the original copyright notice and the associated disclaimer. ####
####                                                              ####
#### This source file is free software; you can redistribute it   ####
#### and/or modify it under the terms of the GNU Lesser General   ####
#### Public License as published by the Free Software Foundation; ####
#### either version 2.1 of the License, or (at your option) any   ####
#### later version.                                               ####
####                                                              ####
#### This source is distributed in the hope that it will be       ####
#### useful, but WITHOUT ANY WARRANTY; without even the implied   ####
#### warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      ####
#### PURPOSE.  See the GNU Lesser General Public License for more ####
#### details.                                                     ####
####                                                              ####
#### You should have received a copy of the GNU Lesser General    ####
#### Public License along with this source; if not, download it   ####
#### from http://www.opencores.org/lgpl.shtml                     ####
####                                                              ####
######################################################################

# 这几个文件不是变量，是目标的标号使用PHONY来声明
.PHONY: verdi-com verdi-debug verdi-clean

# 宏定义，文件名称
VERDI_FSDB_NAME = $(VCS_SIM).fsdb
VERDI_RC_NAME   = $(VCS_SIM).rc
VERDI_LOG_NAME  = $(VCS_SIM).verdi.log
VERDI_SES_NAME  = $(VCS_SIM).ses

######################################################################
# VERDI debug macro define
######################################################################
# setting fsdb and rc param.
VERDI_DEFINE  = -l $(VERDI_LOG_NAME)
VERDI_DEFINE += -ssf $(VERDI_FSDB_NAME)
VERDI_DEFINE += -sswr $(VERDI_RC_NAME)
VERDI_DEFINE += -ssr $(VERDI_SES_NAME)
# setting debug param
VERDI_DEFINE += +notimingchecks
VERDI_DEFINE += +ieee
VERDI_DEFINE += +licqueue
VERDI_DEFINE += +sdf_no_warnings
VERDI_DEFINE += +ncesp
VERDI_DEFINE += +notchkmsg
VERDI_DEFINE += +nocopyright
VERDI_DEFINE += +libext+.udp+.v+.vmd
VERDI_DEFINE += +pathpulse
VERDI_DEFINE += +pulse_r/0
VERDI_DEFINE += +pulse_e/100
VERDI_DEFINE += +no_pulse_msg
VERDI_DEFINE += +ncaccess+rwc
VERDI_DEFINE += +ncstatus
# setting time param
VERDI_DEFINE += +nctimescale+'1ns/1ns'
VERDI_DEFINE += +ncoverride_timescale+$(VCS_SIM)
#...

######################################################################
# VERDI debug source and include define
######################################################################
VERDI_SRC  = $(RTL_VERILOG_SRC) 
VERDI_SRC += $(BENCH_VERILOG_SRC) 
VERDI_SRC += $(BENCH_TOP)
VERDI_INC  = +incdir+$(RTL_VERILOG_INCLUDE_DIR)
VERDI_INC += +incdir+$(BOOTROM_SW_DIR)
VERDI_INC += +incdir+$(BENCH_VERILOG_INCLUDE_DIR)

# Start simulation
# 
verdi-debug: 
	$(Q)verdi $(VERDI_DEFINE) $(VERDI_INC) $(VERDI_SRC) 

# start compile
verdi-com: 
	make vcs-com END_TIME=5000000 VCD=1 MGC_NO_VOPT=1 OR1200=1 OR1200_MONITOR_EXEC_STATE=1 TEST_NAME_STRING=orpsoc_testbench
	make vcs-sim

# Start clean
verdi-clean:
	$(Q)rm -rf *.fsdb novas.rc novas.conf
	$(Q)rm -rf verdiLog
	$(Q)rm -rf $(VERDI_SES_NAME).png
	$(Q)rm -rf $(VERDI_SES_NAME).wave.*
	
