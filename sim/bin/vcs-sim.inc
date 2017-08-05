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
.PHONY: vcs-com vcs-debug vcs-cov vcs-rpt vcs-clean 

# 宏定义，输出的文件替换simv
OUTPUT = $(VCS_SIM).elf

######################################################################
# VCS compile macro define
######################################################################
# enable vpd dump at testbench module.
VCS_DEFINE  = +define+VPD
# setting simulate run time
VCS_DEFINE += +define+END_TIME=5000000

######################################################################
# all include files search path
######################################################################
TB_INC  = +incdir+$(RTL_VERILOG_INCLUDE_DIR)
TB_INC += +incdir+$(BOOTROM_SW_DIR)
TB_INC += +incdir+$(BENCH_VERILOG_INCLUDE_DIR)

######################################################################
# all source files 
######################################################################
ALL_SRC  = $(RTL_VERILOG_SRC) 
ALL_SRC += $(BENCH_VERILOG_SRC) 
ALL_SRC += $(BENCH_TOP)
# ALL_SRC += $(VPI_SRCS)

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
CM_NAME = -cm_name $(VCS_SIM) 
CM_DIR = -cm_dir ./$(VCS_SIM).vdb
CM_LOG = -cm_log cm_$(VCS_SIM).log

# -cm_hier
CM_RUN = -lca -full64

# vpd file name
# 输出的VCD+文件名
VPD_NAME = +vpdfile+$(VCS_SIM).vpd

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
# +mindelays:  最小延时
# +typedelays: 典型延时
# +maxdelays:  最大延时
#
#########################################################################################
LIB_EXT = +libext+.v
LIB_FILE = -v /home/klin/digital_lib/charter/fb_csm18ic_sc-x_2007q2v1/aci/sc-x/verilog/csm18ic.v

DUT_CMP_OPTIONS   += -timescale=1ns/1ns
DUT_CMP_OPTIONS   += -override_timescale=1ns/1ns
# VCS commands
# Compile command 
# 编译指令
# -race: 竞争检查
# -raced
# +alwaystrigger: 在0时刻加上这个参数，我们认为这个信号总是被传递的；默认是打开的
# +rad: 编译时对代码的优化操作，这个应该打开，注意在做覆盖率的时候不能打开这个选项！
# -simprofile: 性能分析
# -cm_hier vcs_cov.cfg: 要测试这个
# -sverilog 
VCS_VSIM_ARGS = +v2k             		   \
				-timescale=1ns/1ns         \
                -full64                    \
				-lca                       \
				-debug_all   		       \
				-Mupdate                   \
				-race                      \
				-simprofile                \
				+notimingcheck             \
				+nospecify                 \
				+vcs+flush+all             \
				$(CM)                      \
				$(CM_NAME)                 \
				$(CM_DIR)                  \
				$(CM_LOG)                  \
				$(LIB_DIR)                 \
				$(LIB_EXT)                 \
				$(LIB_FILE)                \
				$(VCS_DEFINE)              \
				$(DUT_CMP_OPTIONS)         \
				$(VPD_NAME)                \
				-o  $(OUTPUT)              \
                -l  compile.log

# simulation command
SIM = ./$(VCS_SIM).elf           \
      $(CM) $(CM_NAME) $(CM_DIR) \
      $(VPD_NAME)                \
      -l $(VCS_SIM).log  

# debug run param
DEBUG_RUN = -full64

# start compile
vcs-com: sim-restore-profileoutput $(TEST_DEFINES_VLG) $(RTL_VERILOG_SRC) $(RTL_VERILOG_INCLUDES) $(BOOTROM_VERILOG) $(BENCH_VERILOG_SRC) $(BENCH_TOP) $(VPI_SRCS)
	$(Q)$(VCS) $(VCS_VSIM_ARGS) -top $(RTL_TESTBENCH_TOP) $(TB_INC) $(ALL_SRC)

# Start simulation
# 
vcs-sim: $(SIM_DIR_VMEM_IMAGE)
	$(Q)$(SIM) 

# Show the coverage
vcs-cov:
	$(Q)dve $(CM_RUN) -covdir *.vdb &
    
# 生成覆盖率报告
vcs-rpt:
	$(Q)urg $(CM_RUN) -dir *.vdb

vcs-debug:
	$(Q)dve $(DEBUG_RUN) -vpd $(VCS_SIM).vpd &

# Start clean
vcs-clean: distclean 
	$(Q)rm -rf ./csrc  *.daidir  ./csrc   *.log   *.vpd   *.vdb  simv*  *.key  
	$(Q)rm -rf *race*.out* DVEfiles $(VCS_SIM) profileReport.* simprofile_dir.* 
	$(Q)rm -rf profileReport simprofile_dir urgReport* vc_hdrs.h test-defines.v
	$(Q)rm -rf urgReport simprofile_dir* profileReport*
	$(Q)rm -rf orpsoc_testbench.vdb orpsoc_testbench.elf.daidir
	$(Q)rm -rf $(OUTPUT) ../out/
	$(Q)rm -rf $(OUTPUT) sram.vmem