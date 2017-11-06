//////////////////////////////////////////////////////////////////////
////                                                              ////
////  OR1200's register file inside CPU                           ////
////                                                              ////
////  This file is part of the OpenRISC 1200 project              ////
////  http://www.opencores.org/project,or1k                       ////
////                                                              ////
////  Description                                                 ////
////  Instantiation of register file memories                     ////
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
// $Log: or1200_rf.v,v $
// Revision 2.0  2010/06/30 11:00:00  ORSoC
// Minor update:
// Bugs fixed, coding style changed.
//

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "or1200_defines.v"

// RF模块描述了通用寄存器堆（或寄存器文件）的实现。
// RF模块是CPU内部的寄存器文件RAM，它实例化两个双端口RAM或通用寄存器文件RAM。
// 并将来自wbmux模块输出的数据和sprs模块输出的数据写入通用寄存器(RAM)，或将通用
// 寄存器的数据输出到operandmuxes模块或sprs模块。
//
//                              数据写入
// 读出操作数A --------------       |       --------------
//          | 双端口 RAM   |       |      | 双端口 RAM   |
// <--------|（实例 rf_a）   |<------------>| （实例 rf_b）  |------->
//          |            |              |            |
//          --------------              --------------
//
//
module or1200_rf
  (
   // Clock and reset
   // 时钟和复位信号
   clk, rst,

   // Write i/f
   // 写接口
   cy_we_i, cy_we_o, supv, wb_freeze, addrw, dataw, we, flushpipe,

   // Read i/f
   // 读接口
   id_freeze, addra, addrb, dataa, datab, rda, rdb,

   // Debug
   // 调试接口
   spr_cs, spr_write, spr_addr, spr_dat_i, spr_dat_o, du_read
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
  // Write i/f
  //
  input                   cy_we_i;
  output                  cy_we_o;
  input                   supv;
  input                   wb_freeze;
  input  [aw-1:0]         addrw;    // 将要写入的地址
  input  [dw-1:0]         dataw;    // 将要写入的数据
  input                   we;       // 写寄存器信号
  input                   flushpipe;

  //
  // Read i/f
  //
  input                   id_freeze;
  input  [aw-1:0]         addra;
  input  [aw-1:0]         addrb;
  output [dw-1:0]         dataa;
  output [dw-1:0]         datab;
  input                   rda;
  input                   rdb;

  //
  // SPR access for debugging purposes
  //
  input                   spr_cs;
  input                   spr_write;
  input  [31:0]           spr_addr;
  input  [31:0]           spr_dat_i;
  output [31:0]           spr_dat_o;
  input                   du_read;

  //
  // Internal wires and regs
  //
  wire  [dw-1:0]          from_rfa;
  wire  [dw-1:0]          from_rfb;
  wire  [aw-1:0]          rf_addra;
  wire  [aw-1:0]          rf_addrw;
  wire  [dw-1:0]          rf_dataw;
  wire                    rf_we;
  wire                    spr_valid;
  wire                    rf_ena;
  wire                    rf_enb;
  reg                     rf_we_allow;

  // Logic to restore output on RFA after debug unit has read out via SPR if.
  // Problem was that the incorrect output would be on RFA after debug unit
  // had read out  - this is bad if that output is relied upon by execute
  // stage for next instruction. We simply save the last address for rf A and
  // and re-read it whenever the SPR select goes low, so we must remember
  // the last address and generate a signal for falling edge of SPR cs.
  // -- Julius

  // Detect falling edge of SPR select
  reg             spr_du_cs;
  wire            spr_cs_fe;
  // Track RF A's address each time it's enabled
  reg  [aw-1:0]   addra_last;


  always @(posedge clk)
    if (rf_ena & !(spr_cs_fe | (du_read & spr_cs)))
      addra_last <= addra;

  always @(posedge clk)
    spr_du_cs <= spr_cs & du_read;

  assign spr_cs_fe = spr_du_cs & !(spr_cs & du_read);


  //
  // SPR access is valid when spr_cs is asserted and
  // SPR address matches GPR addresses
  //
  // 当spr_cs被声明且SPR地址匹配GPR地址时，SPR访问是有效的。
  assign spr_valid = spr_cs & (spr_addr[10:5] == `OR1200_SPR_RF);

  //
  // SPR data output is always from RF A
  //
  // SPR数据输出总是来自通用寄存器A，即来自A双端口RAM。
  assign spr_dat_o = from_rfa;

  //
  // Operand A comes from RF or from saved A register
  //
  // 操作数A来自内部暂存寄存器或存储的A寄存器堆，即来自A双端口RAM。
  assign dataa = from_rfa;

  //
  // Operand B comes from RF or from saved B register
  //
  // 操作数B来自部暂存寄存器或存储的B寄存器堆，即来自B双端口RAM。
  assign datab = from_rfb;

  //
  // RF A read address is either from SPRS or normal from CPU control
  //
  // A双端口RAM读地址来自SPRS或正常来自CPU控制
  assign rf_addra = (spr_valid & !spr_write) ? spr_addr[4:0] :
                                   spr_cs_fe ? addra_last : addra;

  //
  // RF write address is either from SPRS or normal from CPU control
  //
  // RF写地址来自SPRS或正常来自CPU控制
  assign rf_addrw = (spr_valid & spr_write) ? spr_addr[4:0] : addrw;

  //
  // RF write data is either from SPRS or normal from CPU datapath
  //
  // RF写数据来自SPRS或正常来自CPU数据路径
  assign rf_dataw = (spr_valid & spr_write) ? spr_dat_i : dataw;

  //
  // RF write enable is either from SPRS or normal from CPU control
  //
  // RF写使能来自SPRS或正常来自CPU控制
  always @(`OR1200_RST_EVENT rst or posedge clk)
    if (rst == `OR1200_RST_VALUE)
      rf_we_allow <=  1'b1;

    else if (~wb_freeze)
      rf_we_allow <=  ~flushpipe;

  // rf写使能信号=(SPR有效且写使能) | (数据输入写使能且WB级非停止) & rf允许写 & (超级监管模式 | rf写地址不为0)
  assign rf_we = ((spr_valid & spr_write) | 
                  (we & ~wb_freeze)) & 
                  rf_we_allow;

  assign cy_we_o = cy_we_i && ~wb_freeze && rf_we_allow;

  //
  // CS RF A asserted when instruction reads operand A and ID stage
  // is not stalled
  //
  // 当指令读操作数A并且ID阶段还没停止时，读A双端口RAM的A端口使能信号被声明。
  assign rf_ena = (rda & ~id_freeze) | (spr_valid & !spr_write) | spr_cs_fe;

  //
  // CS RF B asserted when instruction reads operand B and ID stage
  // is not stalled
  //
  // 当指令读操作数B并且ID阶段还没停止时，读B双端口RAM的A端口使能信号被声明。
  assign rf_enb = rdb & ~id_freeze;

`ifdef OR1200_RFRAM_TWOPORT
  // 实例化2个双端口RAM，即实例rf_a和rf_b。
  // 这2个实例的A端口用来读出数据(对应操作数A和B)，
  // B端口用来写入数据(2个B端口写入同样的数据)。也就是说2个双端口RAM，同时保存同样的数据。

  //
  // Instantiation of register file two-port RAM A
  //
  // 寄存器文件双端口RAM A的实例化
  or1200_tpram_32x32 rf_a(
    // Port A
    // 从 Port A读出操作数A
    .clk_a(clk),
    .rst_a(rst),
    .ce_a(rf_ena),
    .we_a(1'b0),
    .oe_a(1'b1),
    .addr_a(rf_addra),
    .di_a(32'h0000_0000),
    .do_a(from_rfa),

    // Port B
    // 从Port B写入数据。
    .clk_b(clk),
    .rst_b(rst),
    .ce_b(rf_we),
    .we_b(rf_we),
    .oe_b(1'b0),
    .addr_b(rf_addrw),
    .di_b(rf_dataw),
    .do_b()
  );

  //
  // Instantiation of register file two-port RAM B
  //
  // 寄存器文件双端口RAM B的实例化
  or1200_tpram_32x32 rf_b(
    // Port A
    // 从Port A读出操作数B
    .clk_a(clk),
    .rst_a(rst),
    .ce_a(rf_enb),
    .we_a(1'b0),
    .oe_a(1'b1),
    .addr_a(addrb),
    .di_a(32'h0000_0000),
    .do_a(from_rfb),

    // Port B
    // 从 Port B写入数据
    .clk_b(clk),
    .rst_b(rst),
    .ce_b(rf_we),
    .we_b(rf_we),
    .oe_b(1'b0),
    .addr_b(rf_addrw),
    .di_b(rf_dataw),
    .do_b()
  );

`else // !OR1200_RFRAM_TWOPORT

`ifdef OR1200_RFRAM_DUALPORT
  // 双端口RAM的实例化
  //
  // Instantiation of register file two-port RAM A
  //
     or1200_dpram #
       (
        .aw(5),
        .dw(32)
        )
     rf_a
       (
        // Port A
        .clk_a(clk),
        .ce_a(rf_ena),
        .addr_a(rf_addra),
        .do_a(from_rfa),

        // Port B
        .clk_b(clk),
        .ce_b(rf_we),
        .we_b(rf_we),
        .addr_b(rf_addrw),
        .di_b(rf_dataw)
        );

     //
     // Instantiation of register file two-port RAM B
     //
     or1200_dpram #
       (
        .aw(5),
        .dw(32)
        )
     rf_b
       (
        // Port A
        .clk_a(clk),
        .ce_a(rf_enb),
        .addr_a(addrb),
        .do_a(from_rfb),

        // Port B
        .clk_b(clk),
        .ce_b(rf_we),
        .we_b(rf_we),
        .addr_b(rf_addrw),
        .di_b(rf_dataw)
        );

`else // !OR1200_RFRAM_DUALPORT

`ifdef OR1200_RFRAM_GENERIC
  // 通用基于触发器的寄存器实例化
  //
  // Instantiation of generic (flip-flop based) register file
  //
  or1200_rfram_generic rf_a(
    // Clock and reset
    .clk(clk),
    .rst(rst),

    // Port A
    .ce_a(rf_ena),
    .addr_a(rf_addra),
    .do_a(from_rfa),

    // Port B
    .ce_b(rf_enb),
    .addr_b(addrb),
    .do_b(from_rfb),

    // Port W
    .ce_w(rf_we),
    .we_w(rf_we),
    .addr_w(rf_addrw),
    .di_w(rf_dataw)
  );

`else // ! OR1200_RFRAM_GENERIC

  //
  // RFRAM type not specified
  //
  // RFRAM类型没被定义时，打印信息
  initial begin
    $display("Define RFRAM type.");
    $finish;
  end

`endif // OR1200_RFRAM_GENERIC

`endif // OR1200_RFRAM_DUALPORT

`endif // OR1200_RFRAM_TWOPORT

endmodule
