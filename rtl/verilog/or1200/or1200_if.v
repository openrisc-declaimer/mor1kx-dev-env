//////////////////////////////////////////////////////////////////////
////                                                              ////
////  OR1200's instruction fetch                                  ////
////                                                              ////
////  This file is part of the OpenRISC 1200 project              ////
////  http://www.opencores.org/project,or1k                       ////
////                                                              ////
////  Description                                                 ////
////  PC, instruction fetch, interface to IC.                     ////
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
// $Log: or1200_if.v,v $
// Revision 2.0  2010/06/30 11:00:00  ORSoC
// Major update:
// Structure reordered and bugs fixed.

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "or1200_defines.v"

// if模块从icache中获取指令送到ID阶段中译码。
// 它还提供了到icache的接口。在if模块需要暂停时，暂存指令。
// 在icache出错或没有延迟槽或从例外返回时，输出空操作指令。
module or1200_if
  (
   // Clock and reset
   clk, rst,

   // External i/f to IC
   icpu_dat_i, icpu_ack_i, icpu_err_i, icpu_adr_i, icpu_tag_i,

   // Internal i/f
   if_freeze, if_insn, if_pc, if_flushpipe, saving_if_insn,
   if_stall, no_more_dslot, genpc_refetch, rfe,
   except_itlbmiss, except_immufault, except_ibuserr
  );

  //
  // I/O
  //

  //
  // Clock and reset
  //
  // 时钟和复位信号
  input                   clk;
  input                   rst;

  //
  // External i/f to IC
  //
  // 到icache的外部接口
  input  [31:0]           icpu_dat_i;
  input                   icpu_ack_i;
  input                   icpu_err_i;
  input  [31:0]           icpu_adr_i;
  input  [3:0]            icpu_tag_i;

  //
  // Internal i/f
  //
  // 内部的接口
  input                   if_freeze;
  output  [31:0]          if_insn;  // 从IF中得到的Instruction指令，传递到ID段；
  output  [31:0]          if_pc;
  input                   if_flushpipe;
  output                  saving_if_insn;
  output                  if_stall;
  input                   no_more_dslot;
  output                  genpc_refetch;
  input                   rfe;
  output                  except_itlbmiss;
  output                  except_immufault;
  output                  except_ibuserr;

  //
  // Internal wires and regs
  //
  wire                    save_insn;
  wire                    if_bypass;
  reg                     if_bypass_reg;
  reg  [31:0]             insn_saved;
  reg  [31:0]             addr_saved;
  reg  [2:0]              err_saved;
  reg                     saved;

  // 保存当前指令的条件
  // 1: 从外面读入数据了，ACK应答
  // 2: if_freeze被冻结中
  // 3：先前没有被保存过
  assign save_insn = (icpu_ack_i | icpu_err_i) & 
                      if_freeze & 
                     !saved;
  assign saving_if_insn = !if_flushpipe & save_insn;

  //
  // IF bypass
  //
  // ?????????
  // cache的行支取，一次会取4个字，16个字节吗？
  assign if_bypass = icpu_adr_i[0] ? 1'b0 : if_bypass_reg | if_flushpipe;

  always @(posedge clk or `OR1200_RST_EVENT rst)
    if (rst == `OR1200_RST_VALUE)
      if_bypass_reg <=  1'b0;

    else
      if_bypass_reg <=  if_bypass;

  //
  // IF stage insn
  //
  // IF阶段指令
  // 输出指令到ctrl模块，指令l.rfe表示从例外返回。指令来源于空指令、暂时存储的指令和icache中的数据。
  // 当icache出错或没有延迟槽或从例外返回时，输出空操作指令。
  // icpu_ack_i表示icache应答已送回指令，icpu_dat_i是icache送回的指令数据
  assign if_insn = no_more_dslot | rfe | if_bypass ? {`OR1200_OR32_NOP, 26'h041_0000} :
                        saved ? insn_saved :
                            icpu_ack_i ?
                              icpu_dat_i : // 当icpu_ack_i有效的时候，从icache返回Instruct code.
                              {`OR1200_OR32_NOP, 26'h061_0000};

  // 输出程序计数器地址PC到except模块，指令地址来自暂时存储的地址或icache地址。
  assign if_pc = saved ? addr_saved : {icpu_adr_i[31:2], 2'h0};

  // 输出停止信号到freeze和except模块。
  assign if_stall = !icpu_err_i & !icpu_ack_i & !saved;

  // 输出重支取信号到genpc模块
  assign genpc_refetch = saved & icpu_ack_i;

  // OR1200_ITAG_TE表示TLB失靶例外，输出TLB失靶例外信号
  assign except_itlbmiss  = no_more_dslot ? 1'b0 : saved ? err_saved[0] : 
                                                           icpu_err_i & (icpu_tag_i == `OR1200_ITAG_TE);

  // OR1200_ITAG_PE表示页错误例外，输出页错误例外信号
  assign except_immufault = no_more_dslot ? 1'b0 : saved ? err_saved[1] : 
                                                           icpu_err_i & (icpu_tag_i == `OR1200_ITAG_PE);

  // OR1200_ITAG_BE表示总线错误例外，输出总线错误例外信号
  assign except_ibuserr   = no_more_dslot ? 1'b0 : saved ? err_saved[2] : 
                                                           icpu_err_i & (icpu_tag_i == `OR1200_ITAG_BE);

  //
  // Flag for saved insn/address
  //
  // 分析得到insn/address的暂时存储标识
  always @(posedge clk or `OR1200_RST_EVENT rst)
    if (rst == `OR1200_RST_VALUE)
      saved <=  1'b0;
    
    else if (if_flushpipe)
      // 从except来的刷新流水线信号
      saved <=  1'b0;
    
    else if (save_insn)
      // 在icache有数据输入；且暂停if；且没有暂存指令时
      saved <=  1'b1;
    
    else if (!if_freeze)
      // 运行时不需要暂存指令
      saved <=  1'b0;

  //
  // Store fetched instruction
  //
  // 存储支取的指令
  always @(posedge clk or `OR1200_RST_EVENT rst)
    if (rst == `OR1200_RST_VALUE)
      // 空操作指令
      insn_saved <=  {`OR1200_OR32_NOP, 26'h041_0000};
    else if (if_flushpipe)
      // 刷新流水线
      insn_saved <=  {`OR1200_OR32_NOP, 26'h041_0000};
    else if (save_insn)
      insn_saved <=  icpu_err_i ? {`OR1200_OR32_NOP, 26'h041_0000} : icpu_dat_i;
    else if (!if_freeze)
      // 指令支取没有暂停
      insn_saved <=  {`OR1200_OR32_NOP, 26'h041_0000};

  //
  // Store fetched instruction's address
  //
  // 存储支取的指令的地址以便输出到except模块
  always @(posedge clk or `OR1200_RST_EVENT rst)
    if (rst == `OR1200_RST_VALUE)
      addr_saved <=  32'h00000000;
    else if (if_flushpipe)
      addr_saved <=  32'h00000000;
    else if (save_insn)
      addr_saved <=  {icpu_adr_i[31:2], 2'b00};
    else if (!if_freeze)
      addr_saved <=  {icpu_adr_i[31:2], 2'b00};

  //
  // Store fetched instruction's error tags
  //
  always @(posedge clk or `OR1200_RST_EVENT rst)
  
    if (rst == `OR1200_RST_VALUE)
      err_saved <=  3'b000;
    
    else if (if_flushpipe)
      err_saved <=  3'b000;
    
    else if (save_insn) begin
      err_saved[0] <=  icpu_err_i & (icpu_tag_i == `OR1200_ITAG_TE);
      err_saved[1] <=  icpu_err_i & (icpu_tag_i == `OR1200_ITAG_PE);
      err_saved[2] <=  icpu_err_i & (icpu_tag_i == `OR1200_ITAG_BE);
    end
    
    else if (!if_freeze)
      err_saved <=  3'b000;

endmodule
