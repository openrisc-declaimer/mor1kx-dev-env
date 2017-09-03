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
.PHONY: leda-chk leda-gui leda-clean

# 宏定义，文件名称
LEDA_LOG_NAME  = $(VCS_SIM).leda.log

######################################################################
# LEDA debug macro define
######################################################################
# setting fsdb and rc param.
LEDA_DEFINE  = -log_dir ../out -l $(LEDA_LOG_NAME)
LEDA_DEFINE += +v2k
LEDA_DEFINE += -chip
LEDA_DEFINE += -top $(VCS_SIM)
# setting debug param
LEDA_DEFINE += +libext+.udp+.v+.vmd
LEDA_DEFINE += +tcl_rule+leda_rule.tcl

######################################################################
# LEDA debug source and include define
######################################################################
LEDA_SRC  = $(RTL_VERILOG_SRC) 
LEDA_SRC += $(BENCH_VERILOG_SRC) 
LEDA_SRC += $(BENCH_TOP)
LEDA_INC  = +incdir+$(RTL_VERILOG_INCLUDE_DIR)
LEDA_INC += +incdir+$(BOOTROM_SW_DIR)
LEDA_INC += +incdir+$(BENCH_VERILOG_INCLUDE_DIR)

# Start simulation
# 
leda-gui: restore-profileoutput $(TEST_DEFINES_VLG) $(RTL_VERILOG_SRC) $(RTL_VERILOG_INCLUDES) $(BOOTROM_VERILOG) $(BENCH_VERILOG_SRC) $(BENCH_TOP) $(VPI_SRCS)
	$(Q)leda +gui $(LEDA_DEFINE) $(LEDA_INC) $(LEDA_SRC) 

# start compile
leda-chk: restore-profileoutput $(TEST_DEFINES_VLG) $(RTL_VERILOG_SRC) $(RTL_VERILOG_INCLUDES) $(BOOTROM_VERILOG) $(BENCH_VERILOG_SRC) $(BENCH_TOP) $(VPI_SRCS)
	$(Q)leda $(LEDA_DEFINE) $(LEDA_INC) $(LEDA_SRC) 
	
# -tcl_rule+rule.tcl 

# Start clean
leda-clean: vcs-clean
	$(Q)rm -rf leda.cmd leda-libs leda-logs
	$(Q)rm -rf leda.cmd leda-libs leda.pro
	$(Q)rm -rf .leda_work .bbox_for_block_check
	# $(Q)rm -rf ../out/
	
