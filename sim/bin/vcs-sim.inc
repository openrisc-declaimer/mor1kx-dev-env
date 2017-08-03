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

# �⼸���ļ����Ǳ�������Ŀ��ı��ʹ��PHONY������
.PHONY: vcs-com vcs-debug vcs-cov vcs-rpt vcs-clean 

# �궨�壬������ļ��滻simv
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

# Code coverage command 
# ���븲�������
# line: Enables statement (line) coverage
# tgl: Enables toggle coverage
# cond: Enables condition coverage
# fsm: Enables FSM coverage
# path: Enables path coverage
# -cm_nocasedef: ���Զ�default�ļ��
# 
CM = -cm line+cond+fsm+branch+tgl+path -cm_nocasedef
CM_NAME = -cm_name $(VCS_SIM) 
CM_DIR = -cm_dir ./$(VCS_SIM).vdb
CM_LOG = -cm_log cm_$(VCS_SIM).log

# -cm_hier
CM_RUN = -lca

# vpd file name
# �����VCD+�ļ���
VPD_NAME = +vpdfile+$(VCS_SIM).vpd

#########################################################################################
# �������
# �������
# +allmtm: �������е�����
# +mindelays: ��С��ʱ
# +typedelays: ������ʱ
# +maxdelays: �����ʱ
# +transport_path_delays: Turns on transport delay mode for path delays
# +transport_int_delays: Turns on transport delays mode for interconnects
# +pluse_e/number: 
# +pluse_r/number:
# +pluse_int_e/number: 
# +pluse_int_r/number:
#
# ����ʱ��
# +mindelays:  ��С��ʱ
# +typedelays: ������ʱ
# +maxdelays:  �����ʱ
#
#########################################################################################
LIB_EXT = +libext+.v

DUT_CMP_OPTIONS   += -timescale=1ns/1ns
DUT_CMP_OPTIONS   += -override_timescale=1ns/1ns
# VCS commands
# Compile command 
# ����ָ��
# -race: �������
# -raced
# +alwaystrigger: ��0ʱ�̼������������������Ϊ����ź����Ǳ����ݵģ�Ĭ���Ǵ򿪵�
# +rad: ����ʱ�Դ�����Ż����������Ӧ�ô򿪣�ע�����������ʵ�ʱ���ܴ����ѡ�
# -simprofile: ���ܷ���
# -cm_hier vcs_cov.cfg: Ҫ�������
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
		
# start compile
vcs-com: sim-restore-profileoutput $(TEST_DEFINES_VLG) $(RTL_VERILOG_SRC) $(RTL_VERILOG_INCLUDES) $(BOOTROM_VERILOG) $(BENCH_VERILOG_SRC) $(BENCH_TOP)
	$(Q)$(VCS) $(VCS_VSIM_ARGS) -top $(RTL_TESTBENCH_TOP) $(TB_INC) $(ALL_SRC)

# Start simulation
vcs-sim: $(SIM_DIR_VMEM_IMAGE)
	$(Q)$(SIM) 

# Show the coverage
vcs-cov:
	$(Q)dve $(CM_RUN) -cov *.vpd &
    
# ���ɸ����ʱ���
vcs-rpt:
	$(Q)urg $(CM_RUN) -dir *.vdb

vcs-debug:
	$(Q)dve -vpd $(VCS_SIM).vpd &

# Start clean
vcs-clean: 
	$(Q)rm -rf ./csrc  *.daidir  ./csrc   *.log   *.vpd   *.vdb  simv*  *.key  
	$(Q)rm -rf *race*.out* DVEfiles $(VCS_SIM) profileReport.* simprofile_dir.* 
	$(Q)rm -rf profileReport simprofile_dir urgReport* vc_hdrs.h test-defines.v
	$(Q)rm -rf urgReport simprofile_dir* profileReport*
	$(Q)rm -rf orpsoc_testbench.vdb orpsoc_testbench.elf.daidir
	$(Q)rm -rf $(OUTPUT) ../out/