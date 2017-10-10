//////////////////////////////////////////////////////////////////////
////                                                              ////
////  OR1200's Instruction decode                                 ////
////                                                              ////
////  This file is part of the OpenRISC 1200 project              ////
////  http://www.opencores.org/project,or1k                       ////
////                                                              ////
////  Description                                                 ////
////  Majority of instruction decoding is performed here.         ////
////                                                              ////
////  To Do:                                                      ////
////   - make it smaller and faster                               ////
////                                                              ////
////  Author(s):                                                  ////
////      - Damjan Lampret, lampret@opencores.org                 ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
////                                                              ////
//// Copyright (C) 2000 Authors and OPENCORES.ORG                 ////
////                                                              ////
//// This source file may be used and distributed without         ////
//// restriction provided that this copyright statement is not    ////
//// removed from the file and that any derivative work contains  ////
//// the original copyright notice and the associated disclaimer. ////
////                                                              ////
//// This source file is free software; you can redistribute it   ////
//// and/or modify it under the terms of the GNU Lesser General   ////
//// Public License as published by the Free Software Foundation; ////
//// either version 2.1 of the License, or (at your option) any   ////
//// later version.                                               ////
////                                                              ////
//// This source is distributed in the hope that it will be       ////
//// useful, but WITHOUT ANY WARRANTY; without even the implied   ////
//// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      ////
//// PURPOSE.  See the GNU Lesser General Public License for more ////
//// details.                                                     ////
////                                                              ////
//// You should have received a copy of the GNU Lesser General    ////
//// Public License along with this source; if not, download it   ////
//// from http://www.opencores.org/lgpl.shtml                     ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
//
//
// $Log: or1200_ctrl.v,v $
// Revision 2.0  2010/06/30 11:00:00  ORSoC
// Major update:
// Structure reordered and bugs fixed.

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "or1200_defines.v"

// 解码l.macrc指令（存储MAC结果，清除MAC寄存器）产生id_macrc_op信号；
// 解码l.rfe指令（从例外返回）产生rfe信号；
// 分析指令选择运算中A操作数的来源，产生sel_a；
// 分析指令选择运算中B操作数的来源，产生sel_b；
// 解码指令l.mfspr和l.mtspr的立即数产生spr_addrimm信号；
// 分析指令使用的时钟周期产生multicycle信号；
// 从Load/Store指令立即操作数计算LSU地址偏移放在lsu_addrofs信号线上；
// 从指令中得到寄存器序号地址放在rf_addrw信号线上，在wb级将寄存器序号地址放在wb_rfaddrw信号线上；
// 如果指令中含有立即操作数，设置sel_imm为1；
// 如果有不合法的指令设置不合法例外信号except_illegal；
// alu运算指令解码到alu_op；
// 乘法指令解码到mac_op；
// 移位旋转指令解码到shrot_op；
// 所有的指令的回写操作解码到rfwb_op；
// 跳转分支指令解码到pre_branch_op；
// 各种load/store指令解码到lsu_op；
// 寄存器比较指令解码到comp_op；
// 指令l.sys解码到sig_syscall；
// 指令l.trap解码到sig_trap。

module or1200_ctrl
  (
   // Clock and reset
   // 时钟和复位信号
   clk, rst,

   // Internal i/f
   // 内部接口
   except_flushpipe, extend_flush, if_flushpipe, id_flushpipe, ex_flushpipe,
   wb_flushpipe,
   id_freeze, ex_freeze, wb_freeze, if_insn, id_insn, ex_insn, abort_mvspr,
   id_branch_op, ex_branch_op, ex_branch_taken, pc_we,
   rf_addra, rf_addrb, rf_rda, rf_rdb, alu_op, alu_op2, mac_op,
   comp_op, rf_addrw, rfwb_op, fpu_op,
   wb_insn, id_simm, ex_simm, id_branch_addrtarget, ex_branch_addrtarget, sel_a,
   sel_b, id_lsu_op,
   cust5_op, cust5_limm, id_pc, ex_pc, du_hwbkpt,
   multicycle, wait_on, wbforw_valid, sig_syscall, sig_trap,
   force_dslot_fetch, no_more_dslot, id_void, ex_void, ex_spr_read,
   ex_spr_write,
   id_mac_op, id_macrc_op, ex_macrc_op, rfe, except_illegal, dc_no_writethrough
  );

  //
  // I/O
  //
  input               clk;
  input               rst;
  
  input               id_freeze;
  input               ex_freeze /* verilator public */;
  input               wb_freeze /* verilator public */;
  
  output              if_flushpipe; // IF段的刷新命令
  output              id_flushpipe; // ID段的刷新命令
  output              ex_flushpipe; // EX段的刷新命令
  output              wb_flushpipe; // WB段的刷新命令
  input               extend_flush;
  input               except_flushpipe;
  input               abort_mvspr ;
  
  input   [31:0]      if_insn;  // 从IF段得到的Instruction；
  output  [31:0]      id_insn;  // 处于流水线译ID段的指令
  output  [31:0]      ex_insn /* verilator public */;   // 处于流水线EX阶段的指令
  
  output  [`OR1200_BRANCHOP_WIDTH-1:0]      ex_branch_op;
  output  [`OR1200_BRANCHOP_WIDTH-1:0]      id_branch_op;
  input               ex_branch_taken;
  output  [`OR1200_REGFILE_ADDR_WIDTH-1:0]  rf_addrw;
  output  [`OR1200_REGFILE_ADDR_WIDTH-1:0]  rf_addra;
  output  [`OR1200_REGFILE_ADDR_WIDTH-1:0]  rf_addrb;
  output              rf_rda;
  output              rf_rdb;
  output  [`OR1200_ALUOP_WIDTH-1:0]         alu_op;
  output [`OR1200_ALUOP2_WIDTH-1:0]         alu_op2;
  output  [`OR1200_MACOP_WIDTH-1:0]         mac_op;
  output [`OR1200_RFWBOP_WIDTH-1:0]         rfwb_op;
  output  [`OR1200_FPUOP_WIDTH-1:0]         fpu_op;
  input                                     pc_we;
  output  [31:0]                            wb_insn;
  output  [31:2]                            id_branch_addrtarget;
  output  [31:2]                            ex_branch_addrtarget;
  output  [`OR1200_SEL_WIDTH-1:0]           sel_a;
  output  [`OR1200_SEL_WIDTH-1:0]           sel_b;
  output  [`OR1200_LSUOP_WIDTH-1:0]         id_lsu_op;
  output  [`OR1200_COMPOP_WIDTH-1:0]        comp_op;
  output  [`OR1200_MULTICYCLE_WIDTH-1:0]    multicycle;
  output  [`OR1200_WAIT_ON_WIDTH-1:0]       wait_on;
  output  [4:0]       cust5_op;
  output  [5:0]       cust5_limm;
  input   [31:0]      id_pc;
  input   [31:0]      ex_pc;
  output  [31:0]      id_simm;
  output  [31:0]      ex_simm;
  input               wbforw_valid;
  input               du_hwbkpt;
  output              sig_syscall;
  output              sig_trap;
  output              force_dslot_fetch;
  output              no_more_dslot;
  output              id_void;
  output              ex_void;
  output              ex_spr_read;
  output              ex_spr_write;
  output  [`OR1200_MACOP_WIDTH-1:0]  id_mac_op;
  output              id_macrc_op;
  output              ex_macrc_op;
  output              rfe;
  output              except_illegal;
  output              dc_no_writethrough;

  //
  // Internal wires and regs
  //
  reg  [`OR1200_BRANCHOP_WIDTH-1:0]   id_branch_op;
  reg  [`OR1200_BRANCHOP_WIDTH-1:0]   ex_branch_op;
  reg  [`OR1200_ALUOP_WIDTH-1:0]      alu_op;
  reg  [`OR1200_ALUOP2_WIDTH-1:0]     alu_op2;
  wire                                if_maci_op;
`ifdef OR1200_MAC_IMPLEMENTED
  reg  [`OR1200_MACOP_WIDTH-1:0]      ex_mac_op;
  reg  [`OR1200_MACOP_WIDTH-1:0]      id_mac_op;
  wire [`OR1200_MACOP_WIDTH-1:0]      mac_op;
  reg                                 ex_macrc_op;
`else
  wire [`OR1200_MACOP_WIDTH-1:0]      mac_op;
  wire                                ex_macrc_op;
`endif
  reg  [31:0]        id_insn /* verilator public */;
  reg  [31:0]        ex_insn /* verilator public */;
  reg  [31:0]        wb_insn /* verilator public */;
  reg  [`OR1200_REGFILE_ADDR_WIDTH-1:0]   rf_addrw;
  reg  [`OR1200_REGFILE_ADDR_WIDTH-1:0]   wb_rfaddrw;
  reg  [`OR1200_RFWBOP_WIDTH-1:0]         rfwb_op;
  reg  [`OR1200_SEL_WIDTH-1:0]            sel_a;
  reg  [`OR1200_SEL_WIDTH-1:0]            sel_b;
  reg                                     sel_imm;
  reg  [`OR1200_LSUOP_WIDTH-1:0]          id_lsu_op;
  reg  [`OR1200_COMPOP_WIDTH-1:0]         comp_op;
  reg  [`OR1200_MULTICYCLE_WIDTH-1:0]     multicycle;
  reg [`OR1200_WAIT_ON_WIDTH-1:0]         wait_on;
  reg   [31:0]        id_simm;
  reg   [31:0]        ex_simm;
  reg                 sig_syscall;
  reg                 sig_trap;
  reg                  except_illegal;
  wire                id_void;
  wire                ex_void;
  wire                wb_void;
  reg                 ex_delayslot_dsi;
  reg                 ex_delayslot_nop;
  reg                 spr_read;
  reg                 spr_write;
  reg     [31:2]      ex_branch_addrtarget;
`ifdef OR1200_DC_NOSTACKWRITETHROUGH
  reg                 dc_no_writethrough;
`endif

  //
  // Register file read addresses
  //
  // 从指令编码中获得寄存器文件序号，输出到模块外
  assign rf_addra = if_insn[20:16]; // 指令中A寄存器序号
  assign rf_addrb = if_insn[15:11]; // 指令中B寄存器序号
  assign rf_rda   = if_insn[31] || if_maci_op; // 控制A寄存器
  assign rf_rdb   = if_insn[30]; // 控制B寄存器

  //
  // Force fetch of delay slot instruction when jump/branch is preceeded by
  // load/store instructions
  // 延迟槽控制信号
  // 当跳转/分支被load/store指令处理时，强迫延迟槽指令的支取。
  assign force_dslot_fetch = 1'b0;
  // RFE表示从例外返回指令，ex_branch_taken表示if已取走分支指令信号，得到没有更多的延迟槽信号
  assign no_more_dslot = (|ex_branch_op & !id_void & ex_branch_taken) |
                         (ex_branch_op == `OR1200_BRANCHOP_RFE); // 注意： RFE指令没有延迟槽

  // 对NOP指令的解析
  // id空指令，id_insn[31:26]为指令编码，id_insn[20:16]为源寄存器编号，id_insn[16]表示非r0寄存器
  assign id_void = (id_insn[31:26] == `OR1200_OR32_NOP) & id_insn[16];

  // ex空指令
  assign ex_void = (ex_insn[31:26] == `OR1200_OR32_NOP) & ex_insn[16];

  // wb空指令
  assign wb_void = (wb_insn[31:26] == `OR1200_OR32_NOP) & wb_insn[16];

  // 产生对sprs的读写信号
  assign ex_spr_write = spr_write && !abort_mvspr;
  assign ex_spr_read  = spr_read && !abort_mvspr;

  //
  // ex_delayslot_dsi: delay slot insn is in EX stage
  // ex_delayslot_nop: (filler) nop insn is in EX stage (before nops
  //                   jump/branch was executed)
  //
  //  ex_delayslot_dsi & !ex_delayslot_nop - DS insn in EX stage
  //  !ex_delayslot_dsi & ex_delayslot_nop - NOP insn in EX stage,
  //       next different is DS insn, previous different was Jump/Branch
  //  !ex_delayslot_dsi & !ex_delayslot_nop - normal insn in EX stage
  //
  always @(posedge clk or `OR1200_RST_EVENT rst) begin
    if (rst == `OR1200_RST_VALUE) begin
      ex_delayslot_nop <=  1'b0;
      ex_delayslot_dsi <=  1'b0;
    end

    else if (!ex_freeze & !ex_delayslot_dsi & ex_delayslot_nop) begin
      ex_delayslot_nop <=  id_void;
      ex_delayslot_dsi <=  !id_void;
    end

    else if (!ex_freeze & ex_delayslot_dsi & !ex_delayslot_nop) begin
      ex_delayslot_nop <=  1'b0;
      ex_delayslot_dsi <=  1'b0;
    end

    else if (!ex_freeze) begin
      ex_delayslot_nop <=  id_void && ex_branch_taken &&
                                     (ex_branch_op != `OR1200_BRANCHOP_NOP) &&
                                     (ex_branch_op != `OR1200_BRANCHOP_RFE);
      ex_delayslot_dsi <=  !id_void && ex_branch_taken &&
                                      (ex_branch_op != `OR1200_BRANCHOP_NOP) &&
                                      (ex_branch_op != `OR1200_BRANCHOP_RFE);
    end
  end

  //
  // Flush pipeline
  //
  assign if_flushpipe = except_flushpipe | pc_we | extend_flush;
  assign id_flushpipe = except_flushpipe | pc_we | extend_flush;
  assign ex_flushpipe = except_flushpipe | pc_we | extend_flush;
  assign wb_flushpipe = except_flushpipe | pc_we | extend_flush;

  //
  // EX Sign/Zero extension of immediates
  //
  always @(posedge clk or `OR1200_RST_EVENT rst) begin
    if (rst == `OR1200_RST_VALUE)
      ex_simm <=  32'h0000_0000;
    else

    if (!ex_freeze) begin
      ex_simm <=  id_simm;
    end
  end

  //
  // ID Sign/Zero extension of immediate
  //
  // 立即数的符号或0扩展
  always @(id_insn) begin
    case (id_insn[31:26])     // synopsys parallel_case

    // l.addi
    `OR1200_OR32_ADDI:
      id_simm = {{16{id_insn[15]}}, id_insn[15:0]};

    // l.addic
    `OR1200_OR32_ADDIC:
      id_simm = {{16{id_insn[15]}}, id_insn[15:0]};

    // l.lxx (load instructions)
    `OR1200_OR32_LWZ, `OR1200_OR32_LBZ, `OR1200_OR32_LBS,
    `OR1200_OR32_LHZ, `OR1200_OR32_LHS:
      id_simm = {{16{id_insn[15]}}, id_insn[15:0]};

    // l.muli
`ifdef OR1200_MULT_IMPLEMENTED
    `OR1200_OR32_MULI:
      id_simm = {{16{id_insn[15]}}, id_insn[15:0]};
`endif

    // l.maci
`ifdef OR1200_MAC_IMPLEMENTED
    `OR1200_OR32_MACI:
      id_simm = {{16{id_insn[15]}}, id_insn[15:0]};
`endif

    // l.mtspr
    `OR1200_OR32_MTSPR:
      id_simm = {16'b0, id_insn[25:21], id_insn[10:0]};

    // l.sxx (store instructions)
    `OR1200_OR32_SW, `OR1200_OR32_SH, `OR1200_OR32_SB:
      id_simm = {{16{id_insn[25]}}, id_insn[25:21], id_insn[10:0]};

    // l.xori
    `OR1200_OR32_XORI:
      id_simm = {{16{id_insn[15]}}, id_insn[15:0]};

    // l.sfxxi (SFXX with immediate)
    `OR1200_OR32_SFXXI:
      id_simm = {{16{id_insn[15]}}, id_insn[15:0]};

    // Instructions with no or zero extended immediate
    default:
      id_simm = {{16'b0}, id_insn[15:0]};

    endcase
  end

  //
  // ID Sign extension of branch offset
  //
  // 分支偏移地址的符号扩展，分支指令中[25:0]位为偏移地址。
  assign id_branch_addrtarget = {{4{id_insn[25]}}, id_insn[25:0]} + id_pc[31:2];

  //
  // EX Sign extension of branch offset
  //

  // pipeline ID and EX branch target address
  always @(posedge clk or `OR1200_RST_EVENT rst) begin
    if (rst == `OR1200_RST_VALUE)
      ex_branch_addrtarget <=  0;
    else if (!ex_freeze)
      ex_branch_addrtarget <=  id_branch_addrtarget;
  end
  // not pipelined
  //assign ex_branch_addrtarget = {{4{ex_insn[25]}}, ex_insn[25:0]} + ex_pc[31:2];

  //
  // l.maci in IF stage
  //
`ifdef OR1200_MAC_IMPLEMENTED
  assign if_maci_op = (if_insn[31:26] == `OR1200_OR32_MACI);
`else
  assign if_maci_op = 1'b0;
`endif

  //
  // l.macrc in ID stage
  // mac运算的操作码解析
  // 在ID阶段的l.macrc指令
`ifdef OR1200_MAC_IMPLEMENTED
  // l.movhi指令操作为rD[31:0] < - extz(Immediate) << 16，指令l.macrc将MAC内容放入rD，并清除MAC累加器。
  // id_macrc_op表示发信号进行l.macrc操作。
  assign id_macrc_op = (id_insn[31:26] == `OR1200_OR32_MACRC) & id_insn[16];
`else
  assign id_macrc_op = 1'b0;
`endif

  //
  // l.macrc in EX stage
  //
`ifdef OR1200_MAC_IMPLEMENTED
  // 在EX级执行指令l.macrc的信号，l.macrc清除MAC的乘法累加器
  always @(posedge clk or `OR1200_RST_EVENT rst) begin
    if (rst == `OR1200_RST_VALUE)
      ex_macrc_op <=  1'b0;
    else
    if (!ex_freeze & id_freeze | ex_flushpipe)
      ex_macrc_op <=  1'b0;
    else
    if (!ex_freeze)
      ex_macrc_op <=  id_macrc_op;
  end
`else
  assign ex_macrc_op = 1'b0;
`endif

  //
  // cust5_op, cust5_limm (L immediate)
  //
  // 用户定制指令l.cust5 rD,rA,rB,L,K
  assign cust5_op   = ex_insn[4:0];
  assign cust5_limm = ex_insn[10:5];

  // 异常返回指示信号
  // pre_branch_op操作指令是模块内部使用的，它对应编码在输入线if_insn[31:26]，得到“从例外中返回”指令信号rfe。
  assign rfe = (id_branch_op == `OR1200_BRANCHOP_RFE) |
               (ex_branch_op == `OR1200_BRANCHOP_RFE);

`ifdef verilator
     // Function to access wb_insn (for Verilator). Have to hide this from
     // simulator, since functions with no inputs are not allowed in IEEE
     // 1364-2001.
     function [31:0] get_wb_insn;
        // verilator public
        get_wb_insn = wb_insn;
     endfunction // get_wb_insn

     // Function to access id_insn (for Verilator). Have to hide this from
     // simulator, since functions with no inputs are not allowed in IEEE
     // 1364-2001.
     function [31:0] get_id_insn;
        // verilator public
        get_id_insn = id_insn;
     endfunction // get_id_insn

     // Function to access ex_insn (for Verilator). Have to hide this from
     // simulator, since functions with no inputs are not allowed in IEEE
     // 1364-2001.
     function [31:0] get_ex_insn;
        // verilator public
        get_ex_insn = ex_insn;
     endfunction // get_ex_insn
`endif

  //
  // Generation of sel_a
  //
  // sel_a的产生（即选择运算中A操作数的来源），来自通用寄存器、wb回写数据或ex执行结果
  always @(rf_addrw or id_insn or rfwb_op or wbforw_valid or wb_rfaddrw)
    // 指令id_insn[20:16]为源通用寄存器地址，rf_addrw为通用寄存器写地址
    // rfwb_op为寄存器操作指令编码
    if ((id_insn[20:16] == rf_addrw) && rfwb_op[0])
      sel_a = `OR1200_SEL_EX_FORW; // 来自ex级执行结果

    // wb_rfaddrw为wb级的通用寄存器写地址，wbforw_valid为回写有效信号
    else if ((id_insn[20:16] == wb_rfaddrw) && wbforw_valid)
      sel_a = `OR1200_SEL_WB_FORW; // 来自wb回写

    else
      sel_a = `OR1200_SEL_RF; // 来自寄存器

  //
  // Generation of sel_b
  //
  // sel_b用于选择B操作数，来源于立即数、通用寄存器、wb回写数据或ex执行结果
  always @(rf_addrw or sel_imm or id_insn or rfwb_op or wbforw_valid or wb_rfaddrw)

    if (sel_imm)
      sel_b = `OR1200_SEL_IMM; // 选择立即数

    else if ((id_insn[15:11] == rf_addrw) && rfwb_op[0])
      sel_b = `OR1200_SEL_EX_FORW;

    else if ((id_insn[15:11] == wb_rfaddrw) && wbforw_valid)
      sel_b = `OR1200_SEL_WB_FORW;

    else
      sel_b = `OR1200_SEL_RF;

  //
  // Decode of multicycle
  //
  // 多周期的解码，得到指令使用的时钟周期。装载和存储指令为2个时钟周期的指令。
  // 指令中l.l*表示是load指令，从内存中装入数据到寄存器rD；l.s*表示是store指令，将rB中数据存入内存有效地址。

  always @(id_insn) begin
    // 指令中[31:26]位是指令编码
    case (id_insn[31:26])    // synopsys parallel_case
      // l.rfe
      `OR1200_OR32_RFE,
      // l.mfspr
      `OR1200_OR32_MFSPR:
        multicycle = `OR1200_TWO_CYCLES;  // to read from ITLB/DTLB (sync RAMs)
      // Single cycle instructions
      default: begin
        // 单周期指令
        multicycle = `OR1200_ONE_CYCLE;
      end
    endcase
  end // always @ (id_insn)

  //
  // Encode wait_on signal
  // 流水线等待的周期数
  always @(id_insn)
  begin
    // 指令中[31:26]位是指令编码
    case (id_insn[31:26])    // synopsys parallel_case
    `OR1200_OR32_ALU:
      wait_on =  ( 1'b0
`ifdef OR1200_DIV_IMPLEMENTED
                  | (id_insn[4:0] == `OR1200_ALUOP_DIV)
                  | (id_insn[4:0] == `OR1200_ALUOP_DIVU)
`endif

`ifdef OR1200_MULT_IMPLEMENTED
                  | (id_insn[4:0] == `OR1200_ALUOP_MUL)
                  | (id_insn[4:0] == `OR1200_ALUOP_MULU)
`endif
                  ) ? `OR1200_WAIT_ON_MULTMAC : `OR1200_WAIT_ON_NOTHING;

`ifdef OR1200_MULT_IMPLEMENTED
`ifdef OR1200_MAC_IMPLEMENTED
    `OR1200_OR32_MACMSB,
    `OR1200_OR32_MACI,
`endif
    `OR1200_OR32_MULI:
      wait_on = `OR1200_WAIT_ON_MULTMAC;
`endif

`ifdef OR1200_MAC_IMPLEMENTED
    `OR1200_OR32_MACRC:
      wait_on = id_insn[16] ? `OR1200_WAIT_ON_MULTMAC : `OR1200_WAIT_ON_NOTHING;
`endif
`ifdef OR1200_FPU_IMPLEMENTED
    `OR1200_OR32_FLOAT:
    begin
      wait_on = id_insn[`OR1200_FPUOP_DOUBLE_BIT] ? 0 : `OR1200_WAIT_ON_FPU;
    end
`endif
`ifndef OR1200_DC_WRITEHROUGH
    // l.mtspr
    `OR1200_OR32_MTSPR:
    begin
        wait_on = `OR1200_WAIT_ON_MTSPR;
    end
`endif

    default: begin
      wait_on = `OR1200_WAIT_ON_NOTHING;
    end
    endcase // case (id_insn[31:26])
  end // always @ (id_insn)

  //
  // Register file write address
  //
  // 计算通用寄存器写地址rf_addrw
  always @(posedge clk or `OR1200_RST_EVENT rst) begin
    if (rst == `OR1200_RST_VALUE)
      rf_addrw <=  5'd0;

    else if (!ex_freeze & id_freeze)
      rf_addrw <=  5'd00;

    else if (!ex_freeze)
      case (id_insn[31:26])  // synopsys parallel_case
      // l.jr rB跳转寄存器rB值
      `OR1200_OR32_JAL, `OR1200_OR32_JALR:
        rf_addrw <=  5'd09;  // link register r9
      default:
        // 指令中rD的序号地址
        rf_addrw <=  id_insn[25:21];
      endcase
  end

  //
  // rf_addrw in wb stage (used in forwarding logic)
  //
  // 计算在wb级通用寄存器写地址wb_rfaddrw，用在转发逻辑。
  always @(posedge clk or `OR1200_RST_EVENT rst) begin
    if (rst == `OR1200_RST_VALUE)
      wb_rfaddrw <=  5'd0;
    else if (!wb_freeze)
      wb_rfaddrw <=  rf_addrw;
  end

  //
  // Instruction latch in id_insn
  // 流水线正在解码的指令暂时保存
  // 在id级没有暂停时，id_insn得到if输入指令
  always @(posedge clk or `OR1200_RST_EVENT rst) begin
    if (rst == `OR1200_RST_VALUE)
      id_insn <=  {`OR1200_OR32_NOP, 26'h041_0000};
    
    else if (id_flushpipe)
      // 刷新流水线时，赋给id段空操作指令。
      id_insn <=  {`OR1200_OR32_NOP, 26'h041_0000};        // NOP -> id_insn[16] must be 1
    
    else if (!id_freeze) begin
      // 如果id级没有暂停时，从if得到输入指令if_insn
      id_insn <=  if_insn;
`ifdef OR1200_VERBOSE
      // synopsys translate_off
      $display("%t: id_insn <= %h", $time, if_insn);
      // synopsys translate_on
`endif
    end
    
  end

  //
  // Instruction latch in ex_insn
  //
  // 在ex级没有暂停时，ex_insn得到id输入指令
  always @(posedge clk or `OR1200_RST_EVENT rst) begin
    if (rst == `OR1200_RST_VALUE)
      ex_insn <=  {`OR1200_OR32_NOP, 26'h041_0000};
    
    else if (!ex_freeze & id_freeze | ex_flushpipe)
      // ex_insn[16] must be 1
      ex_insn <=  {`OR1200_OR32_NOP, 26'h041_0000};  // NOP -> ex_insn[16] must be 1
    
    else if (!ex_freeze) begin
      //如果ex级没有暂停时，从id得到输入指令id_insn
      ex_insn <=  id_insn;
`ifdef OR1200_VERBOSE
      // synopsys translate_off
      $display("%t: ex_insn <= %h", $time, id_insn);
      // synopsys translate_on
`endif
    end

  end

  //
  // Instruction latch in wb_insn
  //
  // 在wb级没有暂停时，从ex_insn得到指令
  always @(posedge clk or `OR1200_RST_EVENT rst) begin

    if (rst == `OR1200_RST_VALUE)
      wb_insn <=  {`OR1200_OR32_NOP, 26'h041_0000};
      
    // wb_insn should not be changed by exceptions due to correct
    // recording of display_arch_state in the or1200_monitor!
    // wb_insn changed by exception is not used elsewhere!
    else if (!wb_freeze) begin
      wb_insn <=  ex_insn;
    end

  end

  //
  // Decode of sel_imm
  //
  // 选择立即数信号sel_imm的解码，缺省时sel_imm为1，对于不带立即数的指令，sel_imm为0。
  always @(posedge clk or `OR1200_RST_EVENT rst) begin
    if (rst == `OR1200_RST_VALUE)
      sel_imm <=  1'b0;

    else if (!id_freeze) begin

      // 指令[31:26]位为操作码
      case (if_insn[31:26])    // synopsys parallel_case

      // j.jalr
      `OR1200_OR32_JALR:
        sel_imm <=  1'b0;

      // l.jr
      `OR1200_OR32_JR:
        sel_imm <=  1'b0;

      // l.rfe
      `OR1200_OR32_RFE:
        sel_imm <=  1'b0;

      // l.mfspr
      `OR1200_OR32_MFSPR:
        sel_imm <=  1'b0;

      // l.mtspr
      `OR1200_OR32_MTSPR:
        sel_imm <=  1'b0;

      // l.sys, l.brk and all three sync insns
      `OR1200_OR32_XSYNC:
        sel_imm <=  1'b0;

      // l.mac/l.msb
`ifdef OR1200_MAC_IMPLEMENTED
      `OR1200_OR32_MACMSB:
        sel_imm <=  1'b0;
`endif

      // l.sw
      `OR1200_OR32_SW:
        sel_imm <=  1'b0;

      // l.sb
      `OR1200_OR32_SB:
        sel_imm <=  1'b0;

      // l.sh
      `OR1200_OR32_SH:
        sel_imm <=  1'b0;

      // ALU instructions except the one with immediate
      // 除了带一个立即数的ALU指令
      `OR1200_OR32_ALU:
        sel_imm <=  1'b0;

      // SFXX instructions
      `OR1200_OR32_SFXX:
        sel_imm <=  1'b0;

`ifdef OR1200_IMPL_ALU_CUST5
      // l.cust5 instructions
      `OR1200_OR32_CUST5:
        sel_imm <=  1'b0;
`endif
`ifdef OR1200_FPU_IMPLEMENTED
      // FPU instructions
      `OR1200_OR32_FLOAT:
        sel_imm <=  1'b0;
`endif
      // l.nop
      `OR1200_OR32_NOP:
        sel_imm <=  1'b0;

      // All instructions with immediates
      default: begin
        // 带有立即数的所有指令
        sel_imm <=  1'b1;
      end

      endcase

    end
  end

  //
  // Decode of except_illegal
  // 非法指令异常信号
  // 不合法例外信号except_illegal的解码
  always @(posedge clk or `OR1200_RST_EVENT rst) begin

    if (rst == `OR1200_RST_VALUE)
      except_illegal <=  1'b0;

    else if (!ex_freeze & id_freeze | ex_flushpipe)
      except_illegal <=  1'b0;

    else if (!ex_freeze) begin
      // 指令[31:26]位为操作码
      case (id_insn[31:26])    // synopsys parallel_case

      // 列举合法的指令
      `OR1200_OR32_J,
      `OR1200_OR32_JAL,
      `OR1200_OR32_JALR,
      `OR1200_OR32_JR,
      `OR1200_OR32_BNF,
      `OR1200_OR32_BF,
      `OR1200_OR32_RFE,
      `OR1200_OR32_MOVHI,
      `OR1200_OR32_MFSPR,
      `OR1200_OR32_XSYNC,
`ifdef OR1200_MAC_IMPLEMENTED
      `OR1200_OR32_MACI,
`endif
      `OR1200_OR32_LWZ,
      `OR1200_OR32_LBZ,
      `OR1200_OR32_LBS,
      `OR1200_OR32_LHZ,
      `OR1200_OR32_LHS,
      `OR1200_OR32_ADDI,
      `OR1200_OR32_ADDIC,
      `OR1200_OR32_ANDI,
      `OR1200_OR32_ORI,
      `OR1200_OR32_XORI,
`ifdef OR1200_MULT_IMPLEMENTED
      `OR1200_OR32_MULI,
`endif
      `OR1200_OR32_SH_ROTI,
      `OR1200_OR32_SFXXI,
      `OR1200_OR32_MTSPR,
`ifdef OR1200_MAC_IMPLEMENTED
      `OR1200_OR32_MACMSB,
`endif
      `OR1200_OR32_SW,
      `OR1200_OR32_SB,
      `OR1200_OR32_SH,
      `OR1200_OR32_SFXX,
`ifdef OR1200_IMPL_ALU_CUST5
      `OR1200_OR32_CUST5,
`endif
      `OR1200_OR32_NOP:
        // 在指令合法时为0
        except_illegal <=  1'b0;
`ifdef OR1200_FPU_IMPLEMENTED
      `OR1200_OR32_FLOAT:
        // Check it's not a double precision instruction
        except_illegal <=  id_insn[`OR1200_FPUOP_DOUBLE_BIT];
`endif

      `OR1200_OR32_ALU:
        // 在指令合法时为0
        except_illegal <=  1'b0

`ifdef OR1200_MULT_IMPLEMENTED
`ifdef OR1200_DIV_IMPLEMENTED
`else
      | (id_insn[4:0] == `OR1200_ALUOP_DIV)
      | (id_insn[4:0] == `OR1200_ALUOP_DIVU)
`endif
`else
      | (id_insn[4:0] == `OR1200_ALUOP_DIV)
      | (id_insn[4:0] == `OR1200_ALUOP_DIVU)
      | (id_insn[4:0] == `OR1200_ALUOP_MUL)
`endif

`ifdef OR1200_IMPL_ADDC
`else
      | (id_insn[4:0] == `OR1200_ALUOP_ADDC)
`endif

`ifdef OR1200_IMPL_ALU_FFL1
`else
      | (id_insn[4:0] == `OR1200_ALUOP_FFL1)
`endif

`ifdef OR1200_IMPL_ALU_ROTATE
`else
      | ((id_insn[4:0] == `OR1200_ALUOP_SHROT) &
         (id_insn[9:6] == `OR1200_SHROTOP_ROR))
`endif

`ifdef OR1200_IMPL_SUB
`else
      | (id_insn[4:0] == `OR1200_ALUOP_SUB)
`endif
`ifdef OR1200_IMPL_ALU_EXT
`else
      | (id_insn[4:0] == `OR1200_ALUOP_EXTHB)
      | (id_insn[4:0] == `OR1200_ALUOP_EXTW)
`endif
      ;

      // Illegal and OR1200 unsupported instructions
    default:
      // 不合法的且OR1200不支持的指令
      except_illegal <=  1'b1;

    endcase
    end // if (!ex_freeze)
  end

  //
  // Decode of alu_op
  //
  // alu指令信号解码到alu_op
  always @(posedge clk or `OR1200_RST_EVENT rst) begin

    if (rst == `OR1200_RST_VALUE)
      alu_op <=  `OR1200_ALUOP_NOP;

    else if (!ex_freeze & id_freeze | ex_flushpipe)
      alu_op <=  `OR1200_ALUOP_NOP;

    else if (!ex_freeze) begin

      // 指令[31:26]位为操作码
      case (id_insn[31:26])    // synopsys parallel_case

        // l.movhi
        `OR1200_OR32_MOVHI:
          alu_op <=  `OR1200_ALUOP_MOVHI;

        // l.addi
        `OR1200_OR32_ADDI:
          alu_op <=  `OR1200_ALUOP_ADD;

        // l.addic
        `OR1200_OR32_ADDIC:
          alu_op <=  `OR1200_ALUOP_ADDC;

        // l.andi
        `OR1200_OR32_ANDI:
          alu_op <=  `OR1200_ALUOP_AND;

        // l.ori
        `OR1200_OR32_ORI:
          alu_op <=  `OR1200_ALUOP_OR;

        // l.xori
        `OR1200_OR32_XORI:
          alu_op <=  `OR1200_ALUOP_XOR;

        // l.muli
`ifdef OR1200_MULT_IMPLEMENTED
        `OR1200_OR32_MULI:
          alu_op <=  `OR1200_ALUOP_MUL;
`endif

        // Shift and rotate insns with immediate
        `OR1200_OR32_SH_ROTI:
          alu_op <=  `OR1200_ALUOP_SHROT;

        // SFXX insns with immediate
        `OR1200_OR32_SFXXI:
          alu_op <=  `OR1200_ALUOP_COMP;

        // ALU instructions except the one with immediate
        // 除了带有1个立即数ALU指令外的所有指令
        `OR1200_OR32_ALU:
          // 指令中[3:0]是具体的指令编码，是寄存器运算指令。
          alu_op <=  {1'b0,id_insn[3:0]};

        // SFXX instructions
        `OR1200_OR32_SFXX:
          alu_op <=  `OR1200_ALUOP_COMP;
`ifdef OR1200_IMPL_ALU_CUST5
        // l.cust5
        `OR1200_OR32_CUST5:
          alu_op <=  `OR1200_ALUOP_CUST5;
`endif
        // Default
        default: begin
          // 缺省情况下
          alu_op <=  `OR1200_ALUOP_NOP;
        end

      endcase

    end
  end

  //
  // Decode of second ALU operation field [9:6]
  //
  always @(posedge clk or `OR1200_RST_EVENT rst) begin
    if (rst == `OR1200_RST_VALUE)
      alu_op2 <=  0;
    else if (!ex_freeze & id_freeze | ex_flushpipe)
            alu_op2 <= 0;
      else if (!ex_freeze) begin
      alu_op2 <=  id_insn[`OR1200_ALUOP2_POS];
    end
  end

  //
  // Decode of spr_read, spr_write
  //
  always @(posedge clk or `OR1200_RST_EVENT rst) begin
    if (rst == `OR1200_RST_VALUE) begin
      spr_read <=  1'b0;
      spr_write <=  1'b0;
    end
    else if (!ex_freeze & id_freeze | ex_flushpipe) begin
      spr_read <=  1'b0;
      spr_write <=  1'b0;
    end
    else if (!ex_freeze) begin
      case (id_insn[31:26])     // synopsys parallel_case

      // l.mfspr
      `OR1200_OR32_MFSPR: begin
        spr_read <=  1'b1;
        spr_write <=  1'b0;
      end

      // l.mtspr
      `OR1200_OR32_MTSPR: begin
        spr_read <=  1'b0;
        spr_write <=  1'b1;
      end

      // Default
      default: begin
        spr_read <=  1'b0;
        spr_write <=  1'b0;
      end

      endcase
    end
  end

  //
  // Decode of mac_op
  //
  // 乘法指令解码到mac_op
`ifdef OR1200_MAC_IMPLEMENTED
  always @(id_insn) begin

    // 指令[31:26]位为操作码
    case (id_insn[31:26])    // synopsys parallel_case

    // l.maci
    `OR1200_OR32_MACI:
      id_mac_op =  `OR1200_MACOP_MAC;

    // l.mac, l.msb
    `OR1200_OR32_MACMSB:
      id_mac_op =  id_insn[2:0];

    // Illegal and OR1200 unsupported instructions
    default:
      id_mac_op =  `OR1200_MACOP_NOP;

    endcase
  end

  always @(posedge clk or `OR1200_RST_EVENT rst) begin
    if (rst == `OR1200_RST_VALUE)
      ex_mac_op <=  `OR1200_MACOP_NOP;
    else if (!ex_freeze & id_freeze | ex_flushpipe)
      ex_mac_op <=  `OR1200_MACOP_NOP;
    else if (!ex_freeze)
      ex_mac_op <=  id_mac_op;
  end

  assign mac_op = abort_mvspr ? `OR1200_MACOP_NOP : ex_mac_op;
`else
  assign id_mac_op = `OR1200_MACOP_NOP;
  assign mac_op    = `OR1200_MACOP_NOP;
`endif

  //
  // Decode of rfwb_op
  //
  // 所有的指令的回写操作解码到rfwb_op
  always @(posedge clk or `OR1200_RST_EVENT rst) begin

    if (rst == `OR1200_RST_VALUE)
      rfwb_op <=  `OR1200_RFWBOP_NOP;

    else  if (!ex_freeze & id_freeze | ex_flushpipe)
      rfwb_op <=  `OR1200_RFWBOP_NOP;

    else  if (!ex_freeze) begin

      // 指令[31:26]位为操作码
      case (id_insn[31:26])    // synopsys parallel_case

      // j.jal
      `OR1200_OR32_JAL:
        rfwb_op <=  {`OR1200_RFWBOP_LR, 1'b1};

      // j.jalr
      `OR1200_OR32_JALR:
        rfwb_op <=  {`OR1200_RFWBOP_LR, 1'b1};

      // l.movhi
      `OR1200_OR32_MOVHI:
        rfwb_op <=  {`OR1200_RFWBOP_ALU, 1'b1};

      // l.mfspr
      `OR1200_OR32_MFSPR:
        rfwb_op <=  {`OR1200_RFWBOP_SPRS, 1'b1};

      // l.lwz
      `OR1200_OR32_LWZ:
        rfwb_op <=  {`OR1200_RFWBOP_LSU, 1'b1};

      // l.lbz
      `OR1200_OR32_LBZ:
        rfwb_op <=  {`OR1200_RFWBOP_LSU, 1'b1};

      // l.lbs
      `OR1200_OR32_LBS:
        rfwb_op <=  {`OR1200_RFWBOP_LSU, 1'b1};

      // l.lhz
      `OR1200_OR32_LHZ:
        rfwb_op <=  {`OR1200_RFWBOP_LSU, 1'b1};

      // l.lhs
      `OR1200_OR32_LHS:
        rfwb_op <=  {`OR1200_RFWBOP_LSU, 1'b1};

      // l.addi
      `OR1200_OR32_ADDI:
        rfwb_op <=  {`OR1200_RFWBOP_ALU, 1'b1};

      // l.addic
      `OR1200_OR32_ADDIC:
        rfwb_op <=  {`OR1200_RFWBOP_ALU, 1'b1};

      // l.andi
      `OR1200_OR32_ANDI:
        rfwb_op <=  {`OR1200_RFWBOP_ALU, 1'b1};

      // l.ori
      `OR1200_OR32_ORI:
        rfwb_op <=  {`OR1200_RFWBOP_ALU, 1'b1};

      // l.xori
      `OR1200_OR32_XORI:
        rfwb_op <=  {`OR1200_RFWBOP_ALU, 1'b1};

      // l.muli
`ifdef OR1200_MULT_IMPLEMENTED
      `OR1200_OR32_MULI:
        rfwb_op <=  {`OR1200_RFWBOP_ALU, 1'b1};
`endif

      // Shift and rotate insns with immediate
      `OR1200_OR32_SH_ROTI:
        rfwb_op <=  {`OR1200_RFWBOP_ALU, 1'b1};

      // ALU instructions except the one with immediate
      // 除了带立即操作数之外的所有ALU指令
      `OR1200_OR32_ALU:
        rfwb_op <=  {`OR1200_RFWBOP_ALU, 1'b1};

`ifdef OR1200_ALU_IMPL_CUST5
      // l.cust5 instructions
      `OR1200_OR32_CUST5:
        rfwb_op <=  {`OR1200_RFWBOP_ALU, 1'b1};
`endif
`ifdef OR1200_FPU_IMPLEMENTED
      // FPU instructions, lf.XXX.s, except sfxx
      `OR1200_OR32_FLOAT:
        rfwb_op <=  {`OR1200_RFWBOP_FPU,!id_insn[3]};
`endif

      // Instructions w/o register-file write-back
      default:
        // 寄存器文件回写的缺省操作信号
        rfwb_op <=  `OR1200_RFWBOP_NOP;

      endcase
    end
  end

  //
  // Decode of id_branch_op
  //
  // 跳转分支指令解码到 id_branch_op
  always @(posedge clk or `OR1200_RST_EVENT rst) begin

    if (rst == `OR1200_RST_VALUE)
      id_branch_op <=  `OR1200_BRANCHOP_NOP;

    else if (id_flushpipe)
      id_branch_op <=  `OR1200_BRANCHOP_NOP;

    else if (!id_freeze) begin

      case (if_insn[31:26])    // synopsys parallel_case

      // l.j
      `OR1200_OR32_J:
        id_branch_op <=  `OR1200_BRANCHOP_J;

      // j.jal
      `OR1200_OR32_JAL:
        id_branch_op <=  `OR1200_BRANCHOP_J;

      // j.jalr
      `OR1200_OR32_JALR:
        id_branch_op <=  `OR1200_BRANCHOP_JR;

      // l.jr
      `OR1200_OR32_JR:
        id_branch_op <=  `OR1200_BRANCHOP_JR;

      // l.bnf
      `OR1200_OR32_BNF:
        id_branch_op <=  `OR1200_BRANCHOP_BNF;

      // l.bf
      `OR1200_OR32_BF:
        id_branch_op <=  `OR1200_BRANCHOP_BF;

      // l.rfe
      `OR1200_OR32_RFE:
        id_branch_op <=  `OR1200_BRANCHOP_RFE;

      // Non branch instructions
      default:
        // 没有分支指令
        id_branch_op <=  `OR1200_BRANCHOP_NOP;

      endcase
    end
  end

  //
  // Generation of ex_branch_op
  //
  // ex_branch_op 的产生
  always @(posedge clk or `OR1200_RST_EVENT rst)

    if (rst == `OR1200_RST_VALUE)
      // OR1200_BRANCHOP_NOP=3'd0，故在复位操作的时候，ex_branch_op为0
      ex_branch_op <=  `OR1200_BRANCHOP_NOP;

    else if (!ex_freeze & id_freeze | ex_flushpipe)
      ex_branch_op <=  `OR1200_BRANCHOP_NOP;

    else if (!ex_freeze)
      ex_branch_op <=  id_branch_op;

  //
  // Decode of id_lsu_op
  //
  // 各种load/store指令解码到 ld_lsu_op
  always @(id_insn) begin

    case (id_insn[31:26])    // synopsys parallel_case

    // l.lwz
    `OR1200_OR32_LWZ:
      id_lsu_op =  `OR1200_LSUOP_LWZ;

    // l.lbz
    `OR1200_OR32_LBZ:
      id_lsu_op =  `OR1200_LSUOP_LBZ;

    // l.lbs
    `OR1200_OR32_LBS:
      id_lsu_op =  `OR1200_LSUOP_LBS;

    // l.lhz
    `OR1200_OR32_LHZ:
      id_lsu_op =  `OR1200_LSUOP_LHZ;

    // l.lhs
    `OR1200_OR32_LHS:
      id_lsu_op =  `OR1200_LSUOP_LHS;

    // l.sw
    `OR1200_OR32_SW:
      id_lsu_op =  `OR1200_LSUOP_SW;

    // l.sb
    `OR1200_OR32_SB:
      id_lsu_op =  `OR1200_LSUOP_SB;

    // l.sh
    `OR1200_OR32_SH:
      id_lsu_op =  `OR1200_LSUOP_SH;

    // Non load/store instructions
    default:
      // 没有load/store指令
      id_lsu_op =  `OR1200_LSUOP_NOP;

    endcase
  end

  //
  // Decode of comp_op
  //
  // 寄存器比较指令解码到comp_op
  // 比较指码在[31:21]位，寄存器比较指令格式为0x72*，带操作数比较指令格式为0x5e*
  always @(posedge clk or `OR1200_RST_EVENT rst) begin

    if (rst == `OR1200_RST_VALUE) begin
      comp_op <=  4'd0;

    end else if (!ex_freeze & id_freeze | ex_flushpipe)
      comp_op <=  4'd0;

    else if (!ex_freeze)
      // 寄存器比较大小指令编码为0x72*
      comp_op <=  id_insn[24:21];
  end

`ifdef OR1200_FPU_IMPLEMENTED
  // Decode of FPU ops
  assign fpu_op = {(id_insn[31:26] == `OR1200_OR32_FLOAT), id_insn[`OR1200_FPUOP_WIDTH-2:0]};
`else
  assign fpu_op = {`OR1200_FPUOP_WIDTH{1'b0}};
`endif

  //
  // Decode of l.sys
  //
  // l.sys解码
  // l.sys K。即：system-call-exception(K)。执行系统调用指令导致系统调用例外。立即数被用来指定哪个系统调用被请求。
  always @(posedge clk or `OR1200_RST_EVENT rst) begin

    if (rst == `OR1200_RST_VALUE)
      sig_syscall <=  1'b0;

    else if (!ex_freeze & id_freeze | ex_flushpipe)
      sig_syscall <=  1'b0;

    else if (!ex_freeze) begin
`ifdef OR1200_VERBOSE
      // synopsys translate_off
      if (id_insn[31:23] == {`OR1200_OR32_XSYNC, 3'b000})
        $display("Generating sig_syscall");
      // synopsys translate_on
`endif
      //操作码为0x2000，是l.sys指令
      sig_syscall <=  (id_insn[31:23] == {`OR1200_OR32_XSYNC, 3'b000});
    end
  end

  //
  // Decode of l.trap
  // 陷入指令的解析
  // 指令l.trap解码到sig_trap
  // l.trap K。即：if SR[K] = 1 then trap-exception()。
  // 如果特定的bit在SR中被设置，trap指令的执行将导致trap例外。
  // trap例外被用来执行调试服务。立即数被用来选择哪一个SR位被trap指令测试。
  always @(posedge clk or `OR1200_RST_EVENT rst) begin
    if (rst == `OR1200_RST_VALUE)
      sig_trap <=  1'b0;
    else if (!ex_freeze & id_freeze | ex_flushpipe)
      sig_trap <=  1'b0;
    else if (!ex_freeze) begin
`ifdef OR1200_VERBOSE
      // synopsys translate_off
      if (id_insn[31:23] == {`OR1200_OR32_XSYNC, 3'b010})
        $display("Generating sig_trap");
      // synopsys translate_on
`endif
      //操作码0x2100是l.trap指令
      sig_trap <=  (id_insn[31:23] == {`OR1200_OR32_XSYNC, 3'b010})
                   | du_hwbkpt; // 硬件中断
    end
  end

  // Decode destination register address for data cache to check if store ops
  // are being done from the stack register (r1) or frame pointer register (r2)
  // 读写data cache的寄存器地址
`ifdef OR1200_DC_NOSTACKWRITETHROUGH
  always @(posedge clk or `OR1200_RST_EVENT rst)
  begin

     if (rst == `OR1200_RST_VALUE)
       dc_no_writethrough <= 0;

     else if (!ex_freeze)
       dc_no_writethrough <= (id_insn[20:16] == 5'd1) | (id_insn[20:16] == 5'd2);

  end
`else

  assign dc_no_writethrough = 0;

`endif

endmodule
