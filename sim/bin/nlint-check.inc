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
.PHONY: nlint-chk nlint-gui nlint-clean

# 宏定义，文件名称
NLINT_LOG_NAME  = $(VCS_SIM).nlint.log

######################################################################
# NLINT debug macro define
######################################################################
# setting fsdb and rc param.
NLINT_DEFINE  = -logdir ../out/ -logfile $(NLINT_LOG_NAME)
LEDA_DEFINE  += -top $(VCS_SIM)
NLINT_DEFINE += -verilog -2001
NLINT_DEFINE += -beauty
NLINT_DEFINE += -fullname
NLINT_DEFINE += -detail
# setting debug param
NLINT_DEFINE += +notimingchecks
NLINT_DEFINE += +ieee
NLINT_DEFINE += +licqueue
NLINT_DEFINE += +sdf_no_warnings
NLINT_DEFINE += +ncesp
NLINT_DEFINE += +notchkmsg
NLINT_DEFINE += +nocopyright
NLINT_DEFINE += +libext+.udp+.v+.vmd
NLINT_DEFINE += +pathpulse
NLINT_DEFINE += +pulse_r/0
NLINT_DEFINE += +pulse_e/100
NLINT_DEFINE += +no_pulse_msg
NLINT_DEFINE += +ncaccess+rwc
NLINT_DEFINE += +ncstatus
# setting time param
NLINT_DEFINE += +nctimescale+'1ns/1ns'
NLINT_DEFINE += +ncoverride_timescale+$(VCS_SIM)
#...

######################################################################
# NLINT debug source and include define
######################################################################
NLINT_SRC  = $(RTL_VERILOG_SRC) 
NLINT_SRC += $(BENCH_VERILOG_SRC) 
NLINT_SRC += $(BENCH_TOP)
NLINT_INC  = +incdir+$(RTL_VERILOG_INCLUDE_DIR)
NLINT_INC += +incdir+$(BOOTROM_SW_DIR)
NLINT_INC += +incdir+$(BENCH_VERILOG_INCLUDE_DIR)

# Start gui interface ...
# 
nlint-gui: restore-profileoutput $(TEST_DEFINES_VLG) $(RTL_VERILOG_SRC) $(RTL_VERILOG_INCLUDES) $(BOOTROM_VERILOG) $(BENCH_VERILOG_SRC) $(BENCH_TOP) $(VPI_SRCS)
	$(Q)nLint -gui $(NLINT_DEFINE) $(NLINT_INC) $(NLINT_SRC)

# Start source code check ...
nlint-chk: restore-profileoutput $(TEST_DEFINES_VLG) $(RTL_VERILOG_SRC) $(RTL_VERILOG_INCLUDES) $(BOOTROM_VERILOG) $(BENCH_VERILOG_SRC) $(BENCH_TOP) $(VPI_SRCS)
	$(Q)nLint $(NLINT_DEFINE) $(NLINT_INC) $(NLINT_SRC)

# Start clean
nlint-clean:
	$(Q)rm -rf nLint.ds nLint.rc orpsoc_testbench.rc
	$(Q)rm -rf nLintDB nLintLog 
	$(Q)rm -rf verdiLog
	
