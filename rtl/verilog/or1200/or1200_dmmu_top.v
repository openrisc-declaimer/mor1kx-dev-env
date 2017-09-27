//////////////////////////////////////////////////////////////////////
////                                                              ////
////  OR1200's Data MMU top level                                 ////
////                                                              ////
////  This file is part of the OpenRISC 1200 project              ////
////  http://www.opencores.org/project,or1k                       ////
////                                                              ////
////  Description                                                 ////
////  Instantiation of all DMMU blocks.                           ////
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
// $Log: or1200_dmmu_top.v,v $
// Revision 2.0  2010/06/30 11:00:00  ORSoC
// Minor update: 
// Bugs fixed. 
//

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "or1200_defines.v"

//
// Data MMU
//

module or1200_dmmu_top
  (
	 // Rst and clk
	 clk, rst,
   
	 // CPU i/f
	 dc_en, dmmu_en, supv, dcpu_adr_i, dcpu_cycstb_i, dcpu_we_i,
	 dcpu_tag_o, dcpu_err_o,
   
	 // SPR access
	 spr_cs, spr_write, spr_addr, spr_dat_i, spr_dat_o,
   
`ifdef OR1200_BIST
	 // RAM BIST
	 mbist_si_i, mbist_so_o, mbist_ctrl_i,
`endif

	 // DC i/f
	 qmemdmmu_err_i, qmemdmmu_tag_i, qmemdmmu_adr_o, qmemdmmu_cycstb_o, qmemdmmu_ci_o
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
  // CPU I/F
  //
  input				        dc_en;          /* SR[3]: Enable Data Cache */
  input				        dmmu_en;        /* SR[5]: Enable Data MMU */
  input				        supv;           /* SR[0]: Super Model */
  input	[aw-1:0]		  dcpu_adr_i;     /* Virtual Address Input*/
  input				        dcpu_cycstb_i;  /* WB BUS */
  input				        dcpu_we_i;
  output	[3:0]			  dcpu_tag_o;
  output				      dcpu_err_o;

  //
  // SPR access
  //
  input				            spr_cs;     /* MMU specify register select */
  input				            spr_write;  /* MMU specify register write enable */
  input	[aw-1:0]		      spr_addr;   /* MMU specify register address */
  input	[31:0]			      spr_dat_i;  /* MMU specify register input data */
  output	[31:0]			    spr_dat_o;  /* MMU specify register output data */

`ifdef OR1200_BIST
  //
  // RAM BIST
  //
  input mbist_si_i;
  input [`OR1200_MBIST_CTRL_WIDTH - 1:0] mbist_ctrl_i;
  output mbist_so_o;
`endif

  //
  // DC I/F
  //
  input				        qmemdmmu_err_i;
  input	[3:0]			    qmemdmmu_tag_i;
  output	[aw-1:0]		qmemdmmu_adr_o;
  output				      qmemdmmu_cycstb_o;
  output				      qmemdmmu_ci_o;  /* Data Cache Disable */

  //
  // Internal wires and regs
  //
  wire				        dtlb_spr_access;
  wire	[31:`OR1200_DMMU_PS]	dtlb_ppn;
  wire				        dtlb_hit;
  wire				        dtlb_uwe;
  wire				        dtlb_ure;
  wire				        dtlb_swe;
  wire				        dtlb_sre;
  wire	[31:0]			  dtlb_dat_o;
  wire				        dtlb_en;
  wire				        dtlb_ci;
  wire				        fault;
  wire				        miss;
`ifdef OR1200_NO_DMMU
`else
  reg				          dtlb_done;
  reg	[31:`OR1200_DMMU_PS]	dcpu_vpn_r;
`endif

  //
  // Implemented bits inside match and translate registers
  //
  // dtlbwYmrX: vpn 31-10  v 0
  // dtlbwYtrX: ppn 31-10  swe 9  sre 8  uwe 7  ure 6
  //
  // dtlb memory width:
  // 19 bits for ppn
  // 13 bits for vpn
  // 1 bit for valid
  // 4 bits for protection
  // 1 bit for cache inhibit

`ifdef OR1200_NO_DMMU // 不使用MMU

  //
  // Put all outputs in inactive state
  //
  assign spr_dat_o = 32'h00000000;          // 特殊寄存器输出0
  assign qmemdmmu_adr_o = dcpu_adr_i;       // 不进行地址翻译，直接输出
  assign dcpu_tag_o = qmemdmmu_tag_i;       // 从QMEM过来的TAG信号，直接送给CPU模块
  assign qmemdmmu_cycstb_o = dcpu_cycstb_i; // CPU输出的总线周期有效信号直接输出给QMEM
  assign dcpu_err_o = qmemdmmu_err_i;       // 从QMEM过来的ERR信号，直接送给CPU模块
  assign qmemdmmu_ci_o = `OR1200_DMMU_CI;   // 在默认的情况下，`OR1200_DMMU_CI=dcpu_adr_i[31]，
                                            // 相当于2GB-4GB的地址是不进行高速缓存的，而2GB以下的空间则进行高速缓存（CACHE）。
`ifdef OR1200_BIST
  assign mbist_so_o = mbist_si_i; //
`endif

`else // !OR1200_NO_DMMU 使用MMU进行地址转换

  //
  // DTLB SPR access
  //
  // 0A00 - 0AFF  dtlbmr w0
  // 0A00 - 0A3F  dtlbmr w0 [63:0]
  //
  // 0B00 - 0BFF  dtlbtr w0
  // 0B00 - 0B3F  dtlbtr w0 [63:0]
  //
  assign dtlb_spr_access = spr_cs; // MMU中特殊寄存器的选择信号

  //
  // Tags:
  //
  // OR1200_DTAG_TE - TLB miss Exception
  // OR1200_DTAG_PE - Page fault Exception
  //
  // 数据MMU总线周期标签信号输出，`OR1200_DTAG_TE表示TLB脱靶例外；`OR1200_DTAG_PE表示页访问出错例外
  assign dcpu_tag_o = miss ? `OR1200_DTAG_TE : 
                              fault ? `OR1200_DTAG_PE : qmemdmmu_tag_i;

  //
  // dcpu_err_o
  //
  // 数据MMU总线周期错误结束信号。其中的qmemdmmu_err_i来源与QMEM模块，而QMEM中相应的信号则来源于dcache信号。
  assign dcpu_err_o = miss | fault | qmemdmmu_err_i;

  //
  // Assert dtlb_done one clock cycle after new address and dtlb_en must be active
  //
  // dtlb_done信号表示DTLB已被操作的信号，其中的dcpu_cycstb_i信号表示LSU读写纵向操作开始，
  // 延迟了1个周期是因为虚拟地址到物理地址的翻译需要一个时钟周期
  always @(posedge clk or `OR1200_RST_EVENT rst)
    if (rst == `OR1200_RST_VALUE)
      dtlb_done <=  1'b0;
    else if (dtlb_en)
      dtlb_done <=  dcpu_cycstb_i;
    else
      dtlb_done <=  1'b0;

  //
  // Cut transfer if something goes wrong with translation. Also delayed signals 
  // because of translation delay.
  // 
  // 如果翻译失败，给出中止传输信号，翻译延迟导致信号延迟。dc_en表示dcache激活，dmmu_en表示dmmu激活，miss表示dtlb失靶，fault表示也出错
  assign qmemdmmu_cycstb_o = (dc_en & dmmu_en) ? 
                             !(miss | fault) & dtlb_done & dcpu_cycstb_i : 
                             !(miss | fault) & dcpu_cycstb_i;

  //
  // Cache Inhibit
  //
  // qmemdmmu_ci_o表示输出给QMEM的cache禁止信号
  assign qmemdmmu_ci_o = dmmu_en ? dtlb_ci : `OR1200_DMMU_CI;

  //
  // Register dcpu_adr_i's VPN for use when DMMU is not enabled but PPN is 
  // expected to come one clock cycle after offset part.
  //
  // dcpu_vpn_r表示为虚拟页号
  always @(posedge clk or `OR1200_RST_EVENT rst)
    if (rst == `OR1200_RST_VALUE)
      dcpu_vpn_r <=  {32-`OR1200_DMMU_PS{1'b0}}; // 即31-13｛1'b0｝
    else
      dcpu_vpn_r <=  dcpu_adr_i[31:`OR1200_DMMU_PS]; // 即dcpu_adr_i[31：13]

  //
  // Physical address is either translated virtual address or
  // simply equal when DMMU is disabled
  //
  // dtlb_ppn是寄存器DTLBWTR[31:13]值，即翻译后的物理页面，dcpu_adr_i[13-1:0]为页内地址。
  // 当DMMU未激活时，将来自CPU核心的虚拟地址dcpu_adr_i送到QMEM
  assign qmemdmmu_adr_o = dmmu_en ? 
                                {dtlb_ppn, dcpu_adr_i[`OR1200_DMMU_PS-1:0]} :
                                dcpu_adr_i;

  //
  // Output to SPRS unit
  //
  // 输出到DMMU特殊寄存器的数据
  assign spr_dat_o = dtlb_spr_access ? dtlb_dat_o : 32'h00000000;

  //
  // Page fault exception logic
  //
  // 页出错例外逻辑
  assign fault = dtlb_done & // DTLB操作完成
                (  (!dcpu_we_i & !supv & !dtlb_ure) // Load in user mode not enabled，用户模式下不允许装载
                || (!dcpu_we_i & supv & !dtlb_sre)  // Load in supv mode not enabled，超级监管者模式下不允许装载
                || (dcpu_we_i & !supv & !dtlb_uwe)  // Store in user mode not enabled，用户模式下不允许存储
                || (dcpu_we_i & supv & !dtlb_swe)); // Store in supv mode not enabled，超级监管者模式模式下不允许存储

  //
  // TLB Miss exception logic
  //
  // DTLB失靶例外逻辑。当DTLB查询完成但没有命中时，产生失靶信号
  assign miss = dtlb_done & !dtlb_hit;

  //
  // DTLB Enable
  //
  // dtlb_en为DTLB使能信号，dcpu_cycstb_i为从CPU传来的总线周期开始信号
  assign dtlb_en = dmmu_en & dcpu_cycstb_i;

  //
  // Instantiation of DTLB
  //
  // DTLB模块实例化
  or1200_dmmu_tlb or1200_dmmu_tlb(
    // Rst and clk
    .clk(clk),
    .rst(rst),

    // I/F for translation
    // 内部接口，快表地址翻译相关信号
    .tlb_en(dtlb_en),
    .vaddr(dcpu_adr_i),
    .hit(dtlb_hit),
    .ppn(dtlb_ppn),
    .uwe(dtlb_uwe),
    .ure(dtlb_ure),
    .swe(dtlb_swe),
    .sre(dtlb_sre),
    .ci(dtlb_ci),

`ifdef OR1200_BIST
    // RAM BIST
    .mbist_si_i(mbist_si_i),
    .mbist_so_o(mbist_so_o),
    .mbist_ctrl_i(mbist_ctrl_i),
`endif

    // SPR access
    // 数据MMU专用寄存器相关信号
    .spr_cs(dtlb_spr_access),
    .spr_write(spr_write),
    .spr_addr(spr_addr),
    .spr_dat_i(spr_dat_i),
    .spr_dat_o(dtlb_dat_o)
  );

`endif // !OR1200_NO_DMMU

endmodule
