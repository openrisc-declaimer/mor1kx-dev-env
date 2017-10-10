//////////////////////////////////////////////////////////////////////
////                                                              ////
////  OR1200's generate PC                                        ////
////                                                              ////
////  This file is part of the OpenRISC 1200 project              ////
////  http://www.opencores.org/project,or1k                       ////
////                                                              ////
////  Description                                                 ////
////  PC, interface to IC.                                        ////
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
// $Log: or1200_genpc.v,v $
// Revision 2.0  2010/06/30 11:00:00  ORSoC
// Major update:
// Structure reordered and bugs fixed.
//
// --------------------------------------------------------------------
//
//  genpc --               -----> IF -----> ID
//         |               |
//         |               |
//         ----> icache ----
//
// --------------------------------------------------------------------

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "or1200_defines.v"

module or1200_genpc
  (
   // Clock and reset
   clk, rst,

   // External i/f to IC
   icpu_adr_o, icpu_cycstb_o, icpu_sel_o, icpu_tag_o,
   icpu_rty_i, icpu_adr_i,

   // Internal i/f
   pre_branch_op, branch_op, except_type, except_prefix,
   id_branch_addrtarget, ex_branch_addrtarget, muxed_b, operand_b,
   flag, flagforw, ex_branch_taken, except_start,
   epcr, spr_dat_i, spr_pc_we, genpc_refetch,
   genpc_freeze, no_more_dslot
  );

  //
  // I/O
  //

  //
  // Clock and reset
  //
  // 时钟和复位信号
  input               clk;
  input               rst;

  //
  // External i/f to IC
  //
  // 外部到IC的接口
  output [31:0]       icpu_adr_o;
  output              icpu_cycstb_o;
  output  [3:0]       icpu_sel_o;
  output  [3:0]       icpu_tag_o;
  input               icpu_rty_i;
  input  [31:0]       icpu_adr_i; // addr input信号包含初始化，
                                  // 或者就是当前的addr地址，但是在IMMU模块中打上一拍了；

  //
  // Internal i/f
  //
  //内部的接口
  input  [`OR1200_BRANCHOP_WIDTH-1:0] pre_branch_op;
  input  [`OR1200_BRANCHOP_WIDTH-1:0] branch_op;
  input  [`OR1200_EXCEPT_WIDTH-1:0]   except_type;
  input                               except_prefix;
  input  [31:2]                       id_branch_addrtarget;
  input  [31:2]                       ex_branch_addrtarget;
  input  [31:0]                       muxed_b;
  input  [31:0]                       operand_b;
  input                               flag;
  input                               flagforw;
  output                              ex_branch_taken;
  input                               except_start;
  input  [31:0]                       epcr;
  input  [31:0]                       spr_dat_i;
  input                               spr_pc_we;
  input                               genpc_refetch;
  input                               genpc_freeze;
  input                               no_more_dslot;

  //
  // Internal wires and regs
  //

  // 这一组和PC相关的寄存器非常重要：
  // pcreg_default
  reg  [31:2]         pcreg_default;
  wire [31:0]         pcreg_boot;
  reg                 pcreg_select;
  // pcreg是PC寄存器的备份，从例外返回时会用到pcreg寄存器
  // 保存着从IF段取码的地址，该指令还没有进入流水线的译码阶段  
  reg  [31:2]         pcreg;   
  // 始终保持着指向下一条指令的地址
  reg  [31:0]         pc;

  // Set in event of jump or taken branch
  reg                 ex_branch_taken;
  reg                 genpc_refetch_r;

  // 建立取指总线周期

  //
  // Address of insn to be fecthed
  //
  // 被支取指令的地址，来自icpu_adr_i或PC寄存器
  assign icpu_adr_o = !no_more_dslot & // 没有延迟槽指令了
                      !except_start &  // 是否在异常的处理中
                      !spr_pc_we &     // 对SPR寄存器的读写中
                      (icpu_rty_i | genpc_refetch) ? // 复位时，icpu_rty_i有效
                        icpu_adr_i :   // 从IMMU中过来的地址
                        {pc[31:2], 1'b0, ex_branch_taken | spr_pc_we};

  //
  // Control access to IC subsystem
  //
  // 控制对IC子系统的访问，表示取指总线周期开始
  // 除了在load/store长周期期间保持已升高的cystb外(即已开始总线周期)，其它情况下工作
  assign icpu_cycstb_o  = ~(genpc_freeze | (|pre_branch_op && !icpu_rty_i));
  assign icpu_sel_o     = 4'b1111; // 片选信号
  // `OR1200_ITAG_NI指令总线的标签为nomal；
  // 标签有：空闲(idle)、正常指令(normal)、总线错误例外(Bus error)、页错误例外(Page fault)、TLB失靶例外(TLB miss)。
  assign icpu_tag_o     = `OR1200_ITAG_NI;

  //
  // genpc_freeze_r
  //
  // 重支取原指令的信号处理
  always @(posedge clk or `OR1200_RST_EVENT rst)
    if (rst == `OR1200_RST_VALUE)
      genpc_refetch_r <=  1'b0;

    else if (genpc_refetch)
      // 重支取
      genpc_refetch_r <=  1'b1;

    else
      genpc_refetch_r <=  1'b0;

  //
  // Async calculation of new PC value. This value is used for addressing the
  // IC.
  //
  // 新PC值的异步计算，这个值用作指令的地址。
  // 根据分支指令计算PC地址。
  always @(pcreg or ex_branch_addrtarget or flag or branch_op or except_type
                 or except_start or operand_b or epcr or spr_pc_we or spr_dat_i or except_prefix)
  begin
    casez ({spr_pc_we, except_start, branch_op}) // synopsys parallel_case
    {2'b00, `OR1200_BRANCHOP_NOP}: begin // 分支操作指令为NOP
      // 在非跳转的情况下，PC=PC+4
      pc = {pcreg + 30'd1, 2'b0};
      ex_branch_taken = 1'b0;
    end

    {2'b00, `OR1200_BRANCHOP_J}: begin
`ifdef OR1200_VERBOSE
      // synopsys translate_off
      $display("%t: BRANCHOP_J: pc <= ex_branch_addrtarget %h", $time, ex_branch_addrtarget);
      // synopsys translate_on
`endif
      // 在跳转的情况下，PC=跳转地址
      pc = {ex_branch_addrtarget, 2'b00}; // 分支偏移地址
      ex_branch_taken = 1'b1; // 通知ctrl分支指令偏移地址已被genpc取走。
    end

    {2'b00, `OR1200_BRANCHOP_JR}: begin // 指令l.jr rB。即：PC <- rB
`ifdef OR1200_VERBOSE
      // synopsys translate_off
      $display("%t: BRANCHOP_JR: pc <= operand_b %h", $time, operand_b);
      // synopsys translate_on
`endif
      // JR，跳转到寄存器指定的地址
      pc = operand_b; // 直接跳转到通用寄存器的值上
      ex_branch_taken = 1'b1;
    end

    {2'b00, `OR1200_BRANCHOP_BF}:
      // 指令l.bf N，即：EA < - exts(Immediate < < 2) + BranchInsnAddr；PC < - EA if SR[F] set
      if (flag) begin
`ifdef OR1200_VERBOSE
        // synopsys translate_off
        $display("%t: BRANCHOP_BF: pc <= ex_branch_addrtarget %h", $time, ex_branch_addrtarget);
        // synopsys translate_on
`endif
        pc = {ex_branch_addrtarget, 2'b00}; // 分支指令地址+分支偏移地址
        ex_branch_taken = 1'b1;
      end

      else begin
`ifdef OR1200_VERBOSE
        // synopsys translate_off
        $display("%t: BRANCHOP_BF: not taken", $time);
        // synopsys translate_on
`endif
        pc = {pcreg + 30'd1, 2'b0}; // 备份pd+4
        ex_branch_taken = 1'b0;
      end

    {2'b00, `OR1200_BRANCHOP_BNF}:
      // 指令l.bnf N；EA < - exts(Immediate < < 2) + BranchInsnAddr；PC < - EA if SR[F] cleared
      if (flag) begin
`ifdef OR1200_VERBOSE
        // synopsys translate_off
        $display("%t: BRANCHOP_BNF: not taken", $time);
        // synopsys translate_on
`endif
        //备份pd+4
        pc = {pcreg + 30'd1, 2'b0};
        ex_branch_taken = 1'b0;
      end

      else begin
`ifdef OR1200_VERBOSE
        // synopsys translate_off
        $display("%t: BRANCHOP_BNF: pc <= ex_branch_addrtarget %h", $time, ex_branch_addrtarget);
        // synopsys translate_on
`endif
        // 分支指令地址+分支偏移地址
        pc = {ex_branch_addrtarget, 2'b00};
        ex_branch_taken = 1'b1;
      end

    {2'b00, `OR1200_BRANCHOP_RFE}: begin
      // 指令l.rfe表示从例外返回，它优先于例外部分恢复处理器状态，它没有延迟槽。即：PC < - EPCR；SR < - ESR
`ifdef OR1200_VERBOSE
      // synopsys translate_off
      $display("%t: BRANCHOP_RFE: pc <= epcr %h", $time, epcr);
      // synopsys translate_on
`endif
      // epcr寄存器中读出pc地址
      pc = epcr;
      ex_branch_taken = 1'b1;
    end

    {2'b01, 3'b???}: begin
      // 从内存中读入pc值，跳转到例外的地址处执行（中断、异常处理）
`ifdef OR1200_VERBOSE
      // synopsys translate_off
      $display("Starting exception: %h.", except_type);
      // synopsys translate_on
`endif
      // SR寄存器的EPH位为高，表示例外向量位于内存0xF0000000开始处，否则，为0开始处
      // except_prefix表示EPH位值，
      pc = {(except_prefix ?
            `OR1200_EXCEPT_EPH1_P : `OR1200_EXCEPT_EPH0_P),
            except_type,
            `OR1200_EXCEPT_V};
      ex_branch_taken = 1'b1;
    end

    default: begin
`ifdef OR1200_VERBOSE
      // synopsys translate_off
      $display("l.mtspr writing into PC: %h.", spr_dat_i);
      // synopsys translate_on
`endif
      // 来自sprs模块特殊寄存器的值。
      pc = spr_dat_i;
      ex_branch_taken = 1'b0;
    end

    endcase
  end

  //
  // PC register
  //
  // 获得pcreg寄存器
  // pcreg寄存器是pc寄存器的备份寄存器，当发生例外时，将pc寄存器的值存储起来，以便例外返回时恢复PC寄存器值。
  always @(posedge clk or `OR1200_RST_EVENT rst)
    // default value
    if (rst == `OR1200_RST_VALUE) begin
      //复位时，从内存中读入pc值
      // SR寄存器EPH（Exception Prefix High）位：
      // 是0表示例外向量位于内存0x0开始的区域。
      // 是1 表示例外向量位于内存0xF0000000开始的区域。
      pcreg_default <=  `OR1200_BOOT_PCREG_DEFAULT; // jb
      pcreg_select  <=  1'b1;// select async. value due to reset state
    end

    // selected value (different from default) is written into FF after
    // reset state
    else if (pcreg_select) begin
      // dynamic value can only be assigned to FF out of reset!
      pcreg_default <=  pcreg_boot[31:2];
      pcreg_select  <=  1'b0;    // select FF value
    end

    else if (spr_pc_we) begin
      // 当sprs模块有spr_pc_we（即sprs输出PC写使能信号）时，从sprs数据线上得到数据。
      pcreg_default <=  spr_dat_i[31:2];
    end

    else if (no_more_dslot | except_start | !genpc_freeze & !icpu_rty_i & !genpc_refetch) begin
      // 在ctrl模块没有数据槽或例外开始等情况下，将pc寄存器的值存储到pcreg备份寄存器中。
      pcreg_default <=  pc[31:2];
    end

  // select async. value for pcreg after reset - PC jumps to the address selected
  // after boot.
  assign  pcreg_boot = `OR1200_BOOT_ADR; // changed JB

  always @(pcreg_boot or pcreg_default or pcreg_select)
    if (pcreg_select)
      // async. value is selected due to reset state
      pcreg = pcreg_boot[31:2];

    else
      // FF value is selected 2nd clock after reset state
      pcreg = pcreg_default ;

endmodule
