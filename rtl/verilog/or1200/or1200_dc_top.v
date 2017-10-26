//////////////////////////////////////////////////////////////////////
////                                                              ////
////  OR1200's Data Cache top level                               ////
////                                                              ////
////  This file is part of the OpenRISC 1200 project              ////
////  http://opencores.org/project,or1k                           ////
////                                                              ////
////  Description                                                 ////
////  Instantiation of all DC blocks.                             ////
////                                                              ////
////  To Do:                                                      ////
////   - Test error during line read or write                     ////
////                                                              ////
////  Author(s):                                                  ////
////      - Damjan Lampret, lampret@opencores.org                 ////
////      - Julius Baxter, julius@opencores.org                   ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
////                                                              ////
//// Copyright (C) 2000, 2010 Authors and OPENCORES.ORG           ////
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
// $Log: or1200_dc_top.v,v $
// Revision 2.0  2010/06/30 11:00:00  ORSoC
// Minor update:
// Bugs fixed.
//

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "or1200_defines.v"

//
// Data cache
//
module or1200_dc_top
  (
	 // Rst, clk and clock control
	 clk, rst,

	 // External i/f
	 dcsb_dat_o, dcsb_adr_o, dcsb_cyc_o, dcsb_stb_o, dcsb_we_o, dcsb_sel_o,
	 dcsb_cab_o, dcsb_dat_i, dcsb_ack_i, dcsb_err_i,

	 // Internal i/f
	 dc_en,
	 dcqmem_adr_i, dcqmem_cycstb_i, dcqmem_ci_i,
	 dcqmem_we_i, dcqmem_sel_i, dcqmem_tag_i, dcqmem_dat_i,
	 dcqmem_dat_o, dcqmem_ack_o, dcqmem_rty_o, dcqmem_err_o, dcqmem_tag_o,

   dc_no_writethrough,

`ifdef OR1200_BIST
	 // RAM BIST
	 mbist_si_i, mbist_so_o, mbist_ctrl_i,
`endif

	 // SPRs
	 spr_cs, spr_write, spr_dat_i, spr_addr, mtspr_dc_done
  );

  // ---------------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------------
  parameter dw = `OR1200_OPERAND_WIDTH;
  parameter aw = `OR1200_OPERAND_WIDTH;

  //
  // I/O
  //

  //
  // Clock and reset
  //
  input				        clk;
  input				        rst;

  //
  // External I/F
  //
  output	[dw-1:0]		dcsb_dat_o; // 输出到写缓存的WB总线数据
  output	[31:0]			dcsb_adr_o; // 输出到写缓存的WB总线地址
  output				      dcsb_cyc_o; // 输出到写缓存的WB总线周期有效信号
  output				      dcsb_stb_o; // 输出到写缓存的WB总线周期选通信号
  output				      dcsb_we_o;  // 输出到写缓存的WB写使能信号
  output	[3:0]			  dcsb_sel_o; // 输出到写缓存的WB字节选择信号
  output				      dcsb_cab_o; // 输出到写缓存的WB连续突发通知信号
  input	[dw-1:0]		  dcsb_dat_i; // 从写缓冲读取的数据信号
  input				        dcsb_ack_i; // 从写缓冲反馈的总线周期应答信号
  input				        dcsb_err_i; // 从写缓冲反馈的总线周期错误信号

  //
  // Internal I/F
  //
  input				        dc_en;            // dcache使能信号
  input	[31:0]			  dcqmem_adr_i;     // 从QMEM输入的WB总线地址信号
  input				        dcqmem_cycstb_i;  // 从QMEM输入的WB总线周期有效和选通信号
  input				        dcqmem_ci_i;      // 从QMEM输入的缓存禁止信号
  input				        dcqmem_we_i;      // 从QMEM输入的WB总线写使能信号
  input	[3:0]			    dcqmem_sel_i;     // 从QMEM输入的WB总线字节选择信号
  input	[3:0]			    dcqmem_tag_i;     // 从QMEM输入的WB总线周期标签信号
  input	[dw-1:0]		  dcqmem_dat_i;     // 从QMEM输入的WB总线数据信号

  output	[dw-1:0]		dcqmem_dat_o;     // 输出到QMEM的WB总线数据信号
  output				      dcqmem_ack_o;     // 输出到QMEM的WB总线操作应答信号
  output				      dcqmem_rty_o;     // 输出到QMEM的WB总线操作重试信号
  output				      dcqmem_err_o;     // 输出到QMEM的WB总线操作错误信号
  output	[3:0]			  dcqmem_tag_o;     // 输出到QMEM的WB总线操作周期标签信号

  input   			      dc_no_writethrough;

`ifdef OR1200_BIST
  //
  // RAM BIST
  //
  input mbist_si_i;
  input [`OR1200_MBIST_CTRL_WIDTH - 1:0] mbist_ctrl_i;
  output mbist_so_o;
`endif

  //
  // SPR access
  //
  input				        spr_cs;         // dcache内部实现的处理器专用寄存器的选择信号
  input				        spr_write;      // dcache内部实现的处理器专用寄存器的写使能信号
  input	[31:0]			  spr_dat_i;      // dcache内部实现的处理器专用寄存器的数据输入信号
  input	[aw-1:0]	    spr_addr;       // dcache内部实现的处理器专用寄存器的地址输入信号
  output  			      mtspr_dc_done;  // 输出倒QMEM的dcache内部实现的处理器专用寄存器的操作完成信号

`ifdef OR1200_NO_DC // 没有使能DACACHE

  // Bypass cache

  // IF to external memory
  assign dcsb_dat_o = dcqmem_dat_i;
  assign dcsb_adr_o = dcqmem_adr_i;
  assign dcsb_cyc_o = dcqmem_cycstb_i;
  assign dcsb_stb_o = dcqmem_cycstb_i;
  assign dcsb_we_o  = dcqmem_we_i;
  assign dcsb_sel_o = dcqmem_sel_i;
  assign dcsb_cab_o = 1'b0;

  // IF to internal memory
  assign dcqmem_dat_o = dcsb_dat_i;
  assign dcqmem_ack_o = dcsb_ack_i;
  assign dcqmem_err_o = dcsb_err_i;
  assign dcqmem_rty_o = ~dcqmem_ack_o;
  assign dcqmem_tag_o = dcqmem_err_o ? `OR1200_DTAG_BE : dcqmem_tag_i;

  assign mtspr_dc_done = 1'b1;

`else

  //
  // Internal wires and regs
  //
  wire				        tag_v;
  wire	[`OR1200_DCTAG_W-2:0]	tag;
  wire    			      dirty;
  wire	[dw-1:0]		  to_dcram;
  wire	[dw-1:0]		  from_dcram;
  wire	[3:0]			    dcram_we;
  wire				        dctag_we;
  wire	[31:0]			  dc_addr;
  wire				        dcfsm_biu_read;
  wire				        dcfsm_biu_write;
  wire                dcfsm_dcram_di_sel;
  wire                dcfsm_biu_do_sel;
  reg				          tagcomp_miss;
  wire	[`OR1200_DCINDXH:`OR1200_DCLS]	dctag_addr;
  wire				        dctag_en;
  wire				        dctag_v;
  wire    			      dctag_dirty;

  wire				        dc_block_invalidate;
  wire 			          dc_block_flush;
  wire 			          dc_block_writeback;
  wire				        dcfsm_first_hit_ack;
  wire				        dcfsm_first_miss_ack;
  wire				        dcfsm_first_miss_err;
  wire				        dcfsm_burst;
  wire				        dcfsm_tag_we;
  wire    			      dcfsm_tag_valid;
  wire    			      dcfsm_tag_dirty;

`ifdef OR1200_BIST
  //
  // RAM BIST
  //
  wire				mbist_ram_so;
  wire				mbist_tag_so;
  wire				mbist_ram_si = mbist_si_i;
  wire				mbist_tag_si = mbist_ram_so;
  assign			mbist_so_o = mbist_tag_so;
`endif

  // Address out to external bus - always from FSM
  // 使用写直达算法，有效地址直接送给OR1200_sb作为将来写主存的地址
  assign dcsb_adr_o = dc_addr;

  //
  // SPR register decodes
  //
`ifdef OR1200_DC_WRITETHROUGH
  // dc_block_invalidate表示使一行无效的信号
  assign dc_block_invalidate = spr_cs & spr_write &
                             ((spr_addr[`OR1200_SPRGRP_DC_ADR_WIDTH-1:0]==`OR1200_SPRGRP_DC_DCBIR) |
                              (spr_addr[`OR1200_SPRGRP_DC_ADR_WIDTH-1:0]==`OR1200_SPRGRP_DC_DCBFR));
  assign dc_block_flush = 0;
  assign dc_block_writeback = 0;
`else
  assign dc_block_invalidate = spr_cs & spr_write &
                              (spr_addr[`OR1200_SPRGRP_DC_ADR_WIDTH-1:0]==`OR1200_SPRGRP_DC_DCBIR);
  assign dc_block_flush = spr_cs & spr_write &
                         (spr_addr[`OR1200_SPRGRP_DC_ADR_WIDTH-1:0]==`OR1200_SPRGRP_DC_DCBFR);
  assign dc_block_writeback = spr_cs & spr_write &
                             (spr_addr[`OR1200_SPRGRP_DC_ADR_WIDTH-1:0]==`OR1200_SPRGRP_DC_DCBWR);
`endif // !`ifdef OR1200_DC_WRITETHROUGH

  // 发现
  assign dctag_we = dcfsm_tag_we | dc_block_invalidate;
  assign dctag_addr = dc_block_invalidate ?
                          spr_dat_i[`OR1200_DCINDXH:`OR1200_DCLS] :
                          dc_addr[`OR1200_DCINDXH:`OR1200_DCLS];
  assign dctag_en = dc_block_invalidate | dc_en;

  assign dctag_v = dc_block_invalidate ? 1'b0 : dcfsm_tag_valid;
  assign dctag_dirty = dc_block_invalidate ? 1'b0 : dcfsm_tag_dirty;

  //
  // Data to BIU is from DCRAM when bursting lines back into memory
  //
  // dcqmem_dat_i在OR1200_qmem_top中被赋值。
  // 当使用CACHE的时候，数据从dcram传递倒BIU；否则从LSU传递到BIU
  assign dcsb_dat_o = dcfsm_biu_do_sel ? from_dcram : dcqmem_dat_i;

  //
  // Bypases of the DC when DC is disabled
  //
  // 当使用CACHE时，使用状态机的读写信号；否则直接使用dcqmem_cycstb_i信号
  assign dcsb_cyc_o = (dc_en) ? dcfsm_biu_read | dcfsm_biu_write : dcqmem_cycstb_i;
  assign dcsb_stb_o = (dc_en) ? dcfsm_biu_read | dcfsm_biu_write : dcqmem_cycstb_i;
  assign dcsb_we_o = (dc_en) ? dcfsm_biu_write : dcqmem_we_i;
  // dcfsm对写缓存的操作总是选中所有的字节
  assign dcsb_sel_o = (dc_en & dcfsm_burst) ? 4'b1111 : dcqmem_sel_i;
  // dcfsm对写缓存的突发操作的握手信号
  assign dcsb_cab_o = dc_en & dcfsm_burst & dcsb_cyc_o;
  // 如果QMEM向dcache发出的操作成功，则重试信号为低，否则重试信号为高
  assign dcqmem_rty_o = ~dcqmem_ack_o;
  // 发送给QMEM的总线周期标签信号，当dcache出错时，送出总线周期异常信号`OR1200_DTAG_BE
  assign dcqmem_tag_o = dcqmem_err_o ? `OR1200_DTAG_BE : dcqmem_tag_i;

  //
  // DC/LSU normal and error termination
  //
  // dcache操作总线周期应答信号
  assign dcqmem_ack_o = dc_en ?
            dcfsm_first_hit_ack | dcfsm_first_miss_ack : dcsb_ack_i;
  // dcache操作总线周期错误应答信号
  assign dcqmem_err_o = dc_en ? dcfsm_first_miss_err : dcsb_err_i;

  //
  // Select between input data generated by LSU or by BIU
  //
  // 当CPU写主存的时候，送到dcram保存的数据来源于CPU的LSU；当进行缓存更新的时候，送到dcram保存的数据来源于主存储
  assign to_dcram = (dcfsm_dcram_di_sel) ? dcsb_dat_i : dcqmem_dat_i;

  //
  // Select between data generated by DCRAM or passed by BIU
  //
  // 当LSU读取缓存失靶或缓存没有使能，则送到LSU的数据dcqmem_dat_o来源于dcsb_dat_i
  assign dcqmem_dat_o = dcfsm_first_miss_ack | !dc_en ? dcsb_dat_i : from_dcram;

  //
  // Tag comparison
  //
  // 比较地址标签以确定缓存是否命中，OR1200_DCTAGL=12
  wire [31:`OR1200_DCTAGL]  dcqmem_adr_i_tag;
  assign dcqmem_adr_i_tag = dcqmem_adr_i[31:`OR1200_DCTAGL];
  always @(tag or dcqmem_adr_i_tag or tag_v) begin
    if ((tag != dcqmem_adr_i_tag) || !tag_v)
      tagcomp_miss = 1'b1;
    else
      tagcomp_miss = 1'b0;
  end

  //
  // Instantiation of DC Finite State Machine
  //
  // dcache住状态机。完成dcache相关的所有操作，包括数据和标签更新以及dcache的读写操作
  or1200_dc_fsm or1200_dc_fsm(
    .clk(clk),
    .rst(rst),
    .dc_en(dc_en),
    .dcqmem_cycstb_i(dcqmem_cycstb_i),
    .dcqmem_ci_i(dcqmem_ci_i),
    .dcqmem_we_i(dcqmem_we_i),
    .dcqmem_sel_i(dcqmem_sel_i),
    .tagcomp_miss(tagcomp_miss),
    .tag(tag),
    .tag_v(tag_v),
    .dirty(dirty),
    .biudata_valid(dcsb_ack_i),
    .biudata_error(dcsb_err_i),
    .lsu_addr(dcqmem_adr_i),
    .dcram_we(dcram_we),
    .biu_read(dcfsm_biu_read),
    .biu_write(dcfsm_biu_write),
    .dcram_di_sel(dcfsm_dcram_di_sel),
    .biu_do_sel(dcfsm_biu_do_sel),
    .first_hit_ack(dcfsm_first_hit_ack),
    .first_miss_ack(dcfsm_first_miss_ack),
    .first_miss_err(dcfsm_first_miss_err),
    .burst(dcfsm_burst),
    .tag_we(dcfsm_tag_we),
    .tag_valid(dcfsm_tag_valid),
    .tag_dirty(dcfsm_tag_dirty),
    .dc_addr(dc_addr),
    .dc_no_writethrough(dc_no_writethrough),
    .dc_block_flush(dc_block_flush),
    .dc_block_writeback(dc_block_writeback),
    .spr_dat_i(spr_dat_i),
    .mtspr_dc_done(mtspr_dc_done),
    .spr_cswe(spr_cs & spr_write)
  );

  //
  // Instantiation of DC main memory
  //
  // 高速缓存的每一行或者块都由16个字节的数据、19位的标签，以及一位的标签有效位组成。
  // 在存储数据时，512X16个字节的数据构成or1200_dc_ram，
  // 而512X20位的标签和标签有效位构成or1200_dc_tag
  or1200_dc_ram or1200_dc_ram(
    .clk(clk),
    .rst(rst),
`ifdef OR1200_BIST
    // RAM BIST
    .mbist_si_i(mbist_ram_si),
    .mbist_so_o(mbist_ram_so),
    .mbist_ctrl_i(mbist_ctrl_i),
`endif
    .addr(dc_addr[`OR1200_DCINDXH:2]),
    .en(dc_en),
    .we(dcram_we),
    .datain(to_dcram),
    .dataout(from_dcram)
  );

  //
  // Instantiation of DC TAG memory
  //
  or1200_dc_tag or1200_dc_tag(
    .clk(clk),
    .rst(rst),
`ifdef OR1200_BIST
    // RAM BIST
    .mbist_si_i(mbist_tag_si),
    .mbist_so_o(mbist_tag_so),
    .mbist_ctrl_i(mbist_ctrl_i),
`endif
    .addr(dctag_addr),
    .en(dctag_en),
    .we(dctag_we),
    .datain({dc_addr[31:`OR1200_DCTAGL], dctag_v, dctag_dirty}),
    .tag_v(tag_v),
    .tag(tag),
    .dirty(dirty)
  );
`endif // !`ifdef OR1200_NO_DC

endmodule
