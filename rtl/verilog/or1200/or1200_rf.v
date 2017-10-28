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
?`ifdef OR1200_RFRAM_GENERIC
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
