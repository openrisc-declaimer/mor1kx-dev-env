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

.PHONY: vcs-com vcs-cov vcs-cov_rep vcs-clean vcs-debug

# 使能VCD+记录
ALL_DEFINE = +define+DUMP_VPD


TB_INC  = +incdir+$(RTL_VERILOG_INCLUDE_DIR)
TB_INC += +incdir+$(BOOTROM_SW_DIR)
TB_INC += +incdir+$(BENCH_VERILOG_INCLUDE_DIR)
	
# Code coverage command 
# 代码覆盖率相关
# line: Enables statement (line) coverage
# tgl: Enables toggle coverage
# cond: Enables condition coverage
# fsm: Enables FSM coverage
# path: Enables path coverage
# -cm_nocasedef: 忽略对default的检查
# 
CM = -cm line+cond+fsm+branch+tgl+path -cm_nocasedef
CM_NAME = -cm_name $(VCS_SIM_EXE) 
CM_DIR = -cm_dir ./$(VCS_SIM_EXE).vdb
CM_LOG = -cm_log cm_$(VCS_SIM_EXE).log

# -cm_hier
CM_RUN = -lca

# vpd file name
# 输出的VCD+文件名
VPD_NAME = +vpdfile+$(VCS_SIM_EXE).vpd

#########################################################################################
# 后仿命令
# 编译命令：
# +allmtm: 编译所有的类型
# +mindelays: 最小延时
# +typedelays: 典型延时
# +maxdelays: 最大延时
# +transport_path_delays: Turns on transport delay mode for path delays
# +transport_int_delays: Turns on transport delays mode for interconnects
# +pluse_e/number: 
# +pluse_r/number:
# +pluse_int_e/number: 
# +pluse_int_r/number:
#
# 运行时：
# +mindelays: 最小延时
# +typedelays: 典型延时
# +maxdelays: 最大延时
#
#########################################################################################
LIB_EXT = +libext+.v

DUT_CMP_OPTIONS   += -timescale=1ns/1ns
DUT_CMP_OPTIONS   += -override_timescale=1ns/1ns
# VCS commands
# -timescale=1ns/1ns
VCS_VSIM_ARGS = -sverilog +v2k             \
                -full64                                      \
					 -lca                                         \
					 -debug_all   		                       		 \
					 -Mupdate                                     \
					 -race                                        \
					 -simprofile                                  \
					 +notimingcheck                               \
					 +nospecify                                   \
					 +vcs+flush+all

# simulation command
SIM = ./$(VCS_SIM_EXE)                \
      $(CM) $(CM_NAME) $(CM_DIR) \
      $(VPD_NAME)                \
      -l $(VCS_SIM_EXE).log  
		
# start compile
vcs-com: sim-restore-profileoutput $(TEST_DEFINES_VLG) $(RTL_VERILOG_SRC) $(RTL_VERILOG_INCLUDES) $(BOOTROM_VERILOG) $(BENCH_VERILOG_SRC) 
#	$(Q)touch synthesis-defines.v
#	$(Q)echo '// Just a test' > synthesis-defines.v
	$(Q)$(VCS) -full64 -top orpsoc_testbench $(DUT_CMP_OPTIONS) $(TB_INC) $(RTL_VERILOG_SRC) $(BENCH_VERILOG_SRC) $(BENCH_TOP)

# Start simulation
vcs-sim: 
	$(SIM) 

# Show the coverage
vcs-cov: 
	dve $(CM_RUN) -covdir *.vdb &
    
# 生成覆盖率报告
vcs-cov_rpt:    
	urg $(CM_RUN) -dir *.vdb 

vcs-debug:
	dve -vpd $(VCS_SIM_EXE).vpd &

# Start clean
vcs-clean: 
	rm -rf  ./csrc  *.daidir  ./csrc   *.log   *.vpd   *.vdb  simv*  *.key  *race*.out* DVEfiles $(VCS_SIM_EXE) profileReport.* simprofile_dir.* profileReport simprofile_dir urgReport* vc_hdrs.h
