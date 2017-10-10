//////////////////////////////////////////////////////////////////////
////                                                              ////
////  OR1200's register file read operands mux                    ////
////                                                              ////
////  This file is part of the OpenRISC 1200 project              ////
////  http://www.opencores.org/project,or1k                       ////
////                                                              ////
////  Description                                                 ////
////  Mux for two register file read operands.                    ////
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
// $Log: or1200_operandmuxes.v,v $
// Revision 2.0  2010/06/30 11:00:00  ORSoC
// Minor update:
// Bugs fixed.

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "or1200_defines.v"

// operandmuxes模块是个操作数多选2的选择器，它从rf模块的数据输出、
// wbmux模块的数据输出、ctrl模块的simm引脚输出中选择2个数据输出，
// 数据输出a到lsu模块作为基地址或作为操作数A给其它模块，数据输出b作为操作数B输出到其它模块。
// alu和mult_mac就是使用来自operandmuxes模块的操作数A和B，并对它们进行运算的。
module or1200_operandmuxes
  (
   // Clock and reset
   // 时钟和复位信号
   clk, rst,

   // Internal i/f
   // 内部接口
   id_freeze, ex_freeze, rf_dataa, rf_datab, ex_forw, wb_forw,
   simm, sel_a, sel_b, operand_a, operand_b, muxed_a, muxed_b
  );

  // ---------------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------------
  parameter width = `OR1200_OPERAND_WIDTH;

  //
  // I/O
  //
  input                          clk;
  input                          rst;
  input                          id_freeze;
  input                          ex_freeze;
  input  [width-1:0]             rf_dataa;
  input  [width-1:0]             rf_datab;
  input  [width-1:0]             ex_forw;
  input  [width-1:0]             wb_forw;
  input  [width-1:0]             simm;
  input  [`OR1200_SEL_WIDTH-1:0] sel_a;
  input  [`OR1200_SEL_WIDTH-1:0] sel_b;
  output  [width-1:0]            operand_a;
  output  [width-1:0]            operand_b;
  output  [width-1:0]            muxed_a;
  output  [width-1:0]            muxed_b;

  //
  // Internal wires and regs
  //
  reg  [width-1:0]               operand_a;
  reg  [width-1:0]               operand_b;
  reg  [width-1:0]               muxed_a;
  reg  [width-1:0]               muxed_b;
  reg                            saved_a;
  reg                            saved_b;

  //
  // Operand A register
  //
  // 控制在操作数A寄存器operand_a上的数据。
  always @(posedge clk or `OR1200_RST_EVENT rst) begin
    if (rst == `OR1200_RST_VALUE) begin
      // 复位
      operand_a <=  32'd0;
      saved_a <=  1'b0;
    end

    // EX级没有暂停且ID级暂停且操作数A没有存到operand_a上。
    else if (!ex_freeze && id_freeze && !saved_a) begin
      operand_a <=  muxed_a; // 得到选择器的数据。
      saved_a <=  1'b1; // 表示数据已存储的信号。
    end

    // saved_a表示操作数A已存到operand_a上
    else if (!ex_freeze && !saved_a) begin
      operand_a <=  muxed_a;
    end

    // ex_freeze 表示EX级暂停
    else if (!ex_freeze && !id_freeze)
      saved_a <=  1'b0;
  end

  //
  // Operand B register
  //
  // 控制在操作数B寄存器operand_b上的数据。
  always @(posedge clk or `OR1200_RST_EVENT rst) begin
    if (rst == `OR1200_RST_VALUE) begin
      operand_b <=  32'd0;
      saved_b <=  1'b0;
    end

    else if (!ex_freeze && id_freeze && !saved_b) begin
      operand_b <=  muxed_b;
      saved_b <=  1'b1;
    end

    else if (!ex_freeze && !saved_b) begin
      operand_b <=  muxed_b;
    end

    else if (!ex_freeze && !id_freeze)
      saved_b <=  1'b0;
  end

  //
  // Forwarding logic for operand A register
  //
  // 操作数A寄存器的转发逻辑，选择输出operand_a上数据源
  always @(ex_forw or wb_forw or rf_dataa or sel_a) begin
`ifdef OR1200_ADDITIONAL_SYNOPSYS_DIRECTIVES
    casez (sel_a)  // synopsys parallel_case infer_mux
`else
    casez (sel_a)  // synopsys parallel_case
`endif
    `OR1200_SEL_EX_FORW:
      // ex_forw表示来自ex级写回寄存器的数据
      muxed_a = ex_forw;

    `OR1200_SEL_WB_FORW:
      // wb_forw表示来自wb级写回寄存器的数据
      muxed_a = wb_forw;

    default:
      // rf_dataa表示来自通用寄存器的数据
      muxed_a = rf_dataa;
    endcase
  end

  //
  // Forwarding logic for operand B register
  //
  // 操作数B寄存器的转发逻辑，选择输出operand_b上数据源
  always @(simm or ex_forw or wb_forw or rf_datab or sel_b) begin
`ifdef OR1200_ADDITIONAL_SYNOPSYS_DIRECTIVES
    casez (sel_b)  // synopsys parallel_case infer_mux
`else
    casez (sel_b)  // synopsys parallel_case
`endif

    `OR1200_SEL_IMM:
      // simm表示有符号立即数，来自指令中
      muxed_b = simm;

    `OR1200_SEL_EX_FORW:
      muxed_b = ex_forw;

    `OR1200_SEL_WB_FORW:
      muxed_b = wb_forw;

    default:
      muxed_b = rf_datab;
    endcase
  end

endmodule
