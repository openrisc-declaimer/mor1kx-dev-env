//////////////////////////////////////////////////////////////////////
////                                                              ////
////  OR1200's Load/Store unit                                    ////
////                                                              ////
////  This file is part of the OpenRISC 1200 project              ////
////  http://opencores.org/project,or1k                           ////
////                                                              ////
////  Description                                                 ////
////  Interface between CPU and DC.                               ////
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
// $Log: or1200_lsu.v,v $
// Revision 2.0  2010/06/30 11:00:00  ORSoC
// Major update:
// Structure reordered and bugs fixed.
//

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "or1200_defines.v"

//
// lsu模块提供了CPU核心到dcache的接口，它将来自dcache的数据装载到通用寄存器中或
// 将通用寄存器的数据存储到dcache中，同时，还根据装载/存储过程中出现的错误产生例外信号，
// 如：产生对齐例外信号except_dtlbmiss、数据MMU错误例外信号except_dmmufault、
// 数据总线错误例外信号except_dbuserr、dcache周期稳定信号dcpu_cycstb_o。
// 它还产生dcache标签输出信号dcpu_tag_o，表示dcache是否正常运行。
//
// lsu模块还计算dcache地址得到地址输出信号dcpu_adr_o，区分装载与存储指令，在Store指令时，
// 设置写dcache使能信号dcpu_we_o为1。
// 由于dcache是按字节分片存储，因此，lsu模块设置dcache片选信号dcpu_sel_o。
//
// 根据Load/Store指令的不同，所选取数据的字节不同，还有指令所要求的符号扩展和0扩展问题，
// 为了解析Load/Store指令操作时通用寄存器与数据高速缓存dcache中的数据字节对应关系，
// lsu模块调用了reg2mem和mem2reg模块，reg2mem模块完成存储指令操作时dcache到通用寄存器的字节转换，
// mem2reg模块完成装载指令操作时通用寄存器到dcache的字节转换。
//
module or1200_lsu
  (
   // Clock and Reset
   clk, rst,

   // Internal i/f
   // 内部接口
   id_addrbase, ex_addrbase, id_addrofs, ex_addrofs, id_lsu_op,
   lsu_datain, lsu_dataout, lsu_stall, lsu_unstall,
   du_stall, except_align, except_dtlbmiss, except_dmmufault, except_dbuserr,
   id_freeze, ex_freeze, flushpipe,

   // External i/f to DC
   // 到dcache的外部接口
   dcpu_adr_o, dcpu_cycstb_o, dcpu_we_o, dcpu_sel_o, dcpu_tag_o, dcpu_dat_o,
   dcpu_dat_i, dcpu_ack_i, dcpu_rty_i, dcpu_err_i, dcpu_tag_i
  );

  // ---------------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------------
  parameter dw = `OR1200_OPERAND_WIDTH;
  parameter aw = `OR1200_REGFILE_ADDR_WIDTH;

  //
  // I/O
  //

  //
  // Clock and reset
  //
  input                   clk;
  input                   rst;

  //
  // Internal i/f
  //
  input  [31:0]           id_addrbase;
  input  [31:0]           ex_addrbase;
  input  [31:0]           id_addrofs;
  input  [31:0]           ex_addrofs;
  input  [`OR1200_LSUOP_WIDTH-1:0] id_lsu_op;
  input  [dw-1:0]         lsu_datain;
  output [dw-1:0]         lsu_dataout;
  output                  lsu_stall;
  output                  lsu_unstall;
  input                   du_stall;
  output                  except_align;
  output                  except_dtlbmiss;
  output                  except_dmmufault;
  output                  except_dbuserr;
  input                   id_freeze;
  input                   ex_freeze;
  input                   flushpipe;

  //
  // External i/f to DC
  //
  output [31:0]           dcpu_adr_o;
  output                  dcpu_cycstb_o;
  output                  dcpu_we_o;
  output [3:0]            dcpu_sel_o;
  output [3:0]            dcpu_tag_o;
  output [31:0]           dcpu_dat_o;
  input  [31:0]           dcpu_dat_i;
  input                   dcpu_ack_i;
  input                   dcpu_rty_i;
  input                   dcpu_err_i;
  input  [3:0]            dcpu_tag_i;

  //
  // Internal wires/regs
  //
  reg    [3:0]                        dcpu_sel_o;

  reg    [`OR1200_LSUOP_WIDTH-1:0]    ex_lsu_op;
  wire   [`OR1200_LSUEA_PRECALC:0]    id_precalc_sum;
  reg    [`OR1200_LSUEA_PRECALC:0]    dcpu_adr_r;
  reg                                 except_align;

  //
  // ex_lsu_op
  //
  always @(posedge clk or `OR1200_RST_EVENT rst) begin
    
    if (rst == `OR1200_RST_VALUE)
      ex_lsu_op <=  `OR1200_LSUOP_NOP;
    
    else if (!ex_freeze & id_freeze | flushpipe)
      ex_lsu_op <=  `OR1200_LSUOP_NOP;
    
    else if (!ex_freeze)
      ex_lsu_op <=  id_lsu_op;
  end

  //
  // Precalculate part of load/store EA in ID stage
  //
  assign id_precalc_sum = id_addrbase[`OR1200_LSUEA_PRECALC-1:0] +
                          id_addrofs[`OR1200_LSUEA_PRECALC-1:0];

  always @(posedge clk or `OR1200_RST_EVENT rst) begin
    if (rst == `OR1200_RST_VALUE)
      dcpu_adr_r <=  {`OR1200_LSUEA_PRECALC+1{1'b0}};
    else if (!ex_freeze)
      dcpu_adr_r <=  id_precalc_sum;
  end

  //
  // Generate except_align in ID stage
  // 不对齐引发的异常
  // 对齐例外=(装载半字指令 & dcache地址0位不为0 | 装载全字 & dcache地址1和0位不为0。
  always @(posedge clk or `OR1200_RST_EVENT rst) begin
    if (rst == `OR1200_RST_VALUE)
      except_align <=  1'b0;
    else if (!ex_freeze & id_freeze | flushpipe)
      except_align <=  1'b0;
    else if (!ex_freeze)
      except_align <=  ((id_lsu_op == `OR1200_LSUOP_SH)  |
                        (id_lsu_op == `OR1200_LSUOP_LHZ) |
                        (id_lsu_op == `OR1200_LSUOP_LHS)) & id_precalc_sum[0]
                    |  ((id_lsu_op == `OR1200_LSUOP_SW)  |
                        (id_lsu_op == `OR1200_LSUOP_LWZ) |
                        (id_lsu_op == `OR1200_LSUOP_LWS)) & |id_precalc_sum[1:0];
  end

  //
  // Internal I/F assignments
  //
  // 内部接口指定
  // lsu_stall表示lsu停止下来接收数据的信号，输出给freeze模块
  assign lsu_stall = dcpu_rty_i & dcpu_cycstb_o; // dcache数据准备好 & 总线周期开始
  assign lsu_unstall = dcpu_ack_i; // dcache应答信号

  // 数据TLB失靶例信号=dcache错误 & dcache标签为TLB失靶。
  assign except_dtlbmiss = dcpu_err_i & (dcpu_tag_i == `OR1200_DTAG_TE);

  // 数据MMU错误例外=dcache错误 & dcache标签为页错误
  assign except_dmmufault = dcpu_err_i & (dcpu_tag_i == `OR1200_DTAG_PE);

  // 数据总线错误例外=dcache错误 & dcache标签为总线错误
  assign except_dbuserr = dcpu_err_i & (dcpu_tag_i == `OR1200_DTAG_BE);

  //
  // External I/F assignments
  //
  // 计算dcache地址
  assign dcpu_adr_o[31:`OR1200_LSUEA_PRECALC] =
                                       ex_addrbase[31:`OR1200_LSUEA_PRECALC] +
                                       (ex_addrofs[31:`OR1200_LSUEA_PRECALC] +
                                        // carry from precalc, pad to 30-bits
                                       {{(32-`OR1200_LSUEA_PRECALC)-1{1'b0}},
                                        dcpu_adr_r[`OR1200_LSUEA_PRECALC]});
  assign dcpu_adr_o[`OR1200_LSUEA_PRECALC-1:0] = dcpu_adr_r[`OR1200_LSUEA_PRECALC-1:0];

  // 如果du停止或从停止中恢复或对齐例外，则dcache周期稳定信号为0(即不稳定)。否则，在lsu有操作指令时，dcache周期稳定信号为1(即为稳定)。
  assign dcpu_cycstb_o = du_stall | lsu_unstall | except_align ? 1'b0 : |ex_lsu_op;

  // 在ctrl中解码Load/Store指令再重编码后，lsu_op[3]为1的指令是Store指令。
  assign dcpu_we_o = ex_lsu_op[3];

  // dcache标签输出=dcpu周期稳定 ? 运行正常标识 : 空闲标识。
  assign dcpu_tag_o = dcpu_cycstb_o ? `OR1200_DTAG_ND : `OR1200_DTAG_IDLE;

  always @(ex_lsu_op or dcpu_adr_o)
    // 根据字节、半字或字对齐选择低2位地址，设置dcache片选信号。
    // dcache中，将字的4个字节分别放在4片dcache中存储。
    casez({ex_lsu_op, dcpu_adr_o[1:0]})
    {`OR1200_LSUOP_SB, 2'b00} : dcpu_sel_o = 4'b1000;
    {`OR1200_LSUOP_SB, 2'b01} : dcpu_sel_o = 4'b0100;
    {`OR1200_LSUOP_SB, 2'b10} : dcpu_sel_o = 4'b0010;
    {`OR1200_LSUOP_SB, 2'b11} : dcpu_sel_o = 4'b0001;
    {`OR1200_LSUOP_SH, 2'b00} : dcpu_sel_o = 4'b1100;
    {`OR1200_LSUOP_SH, 2'b10} : dcpu_sel_o = 4'b0011;
    {`OR1200_LSUOP_SW, 2'b00} : dcpu_sel_o = 4'b1111;
    {`OR1200_LSUOP_LBZ, 2'b00}, {`OR1200_LSUOP_LBS, 2'b00} : dcpu_sel_o = 4'b1000;
    {`OR1200_LSUOP_LBZ, 2'b01}, {`OR1200_LSUOP_LBS, 2'b01} : dcpu_sel_o = 4'b0100;
    {`OR1200_LSUOP_LBZ, 2'b10}, {`OR1200_LSUOP_LBS, 2'b10} : dcpu_sel_o = 4'b0010;
    {`OR1200_LSUOP_LBZ, 2'b11}, {`OR1200_LSUOP_LBS, 2'b11} : dcpu_sel_o = 4'b0001;
    {`OR1200_LSUOP_LHZ, 2'b00}, {`OR1200_LSUOP_LHS, 2'b00} : dcpu_sel_o = 4'b1100;
    {`OR1200_LSUOP_LHZ, 2'b10}, {`OR1200_LSUOP_LHS, 2'b10} : dcpu_sel_o = 4'b0011;
    {`OR1200_LSUOP_LWZ, 2'b00}, {`OR1200_LSUOP_LWS, 2'b00} : dcpu_sel_o = 4'b1111;
    default : dcpu_sel_o = 4'b0000;
    endcase

  //
  // Instantiation of Memory-to-regfile aligner
  // load, 就是将内存的数据搬到寄存器里
  // Memory-to-regfile对准器实例化
  or1200_mem2reg or1200_mem2reg(
    .addr(dcpu_adr_o[1:0]),
    .lsu_op(ex_lsu_op),
    .memdata(dcpu_dat_i),
    .regdata(lsu_dataout)
  );

  //
  // Instantiation of Regfile-to-memory aligner
  // store, 就是将寄存器的数据搬到内存
  // Regfile-to-memory对准器实例化
  or1200_reg2mem or1200_reg2mem(
          .addr(dcpu_adr_o[1:0]),
          .lsu_op(ex_lsu_op),
          .regdata(lsu_datain),
          .memdata(dcpu_dat_o)
  );

endmodule
