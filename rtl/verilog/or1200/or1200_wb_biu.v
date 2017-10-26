//////////////////////////////////////////////////////////////////////
////                                                              ////
////  OR1200's WISHBONE BIU                                       ////
////                                                              ////
////  This file is part of the OpenRISC 1200 project              ////
////  http://opencores.org/project,or1k                           ////
////                                                              ////
////  Description                                                 ////
////  Implements WISHBONE interface                               ////
////                                                              ////
////  To Do:                                                      ////
////   - if biu_cyc/stb are deasserted and wb_ack_i is asserted   ////
////   and this happens even before aborted_r is asssrted,        ////
////   wb_ack_i will be delivered even though transfer is         ////
////   internally considered already aborted. However most        ////
////   wb_ack_i are externally registered and delayed. Normally   ////
////   this shouldn't cause any problems.                         ////
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
// $Log: or1200_wb_biu.v,v $
// Revision 2.0  2010/06/30 11:00:00  ORSoC
// Major update:
// Structure reordered and bugs fixed.
//

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "or1200_defines.v"

module or1200_wb_biu
  (
   // RISC clock, reset and clock control
   clk, rst, clmode,

   // WISHBONE interface
   wb_clk_i, wb_rst_i, wb_ack_i, wb_err_i, wb_rty_i, wb_dat_i,
   wb_cyc_o, wb_adr_o, wb_stb_o, wb_we_o, wb_sel_o, wb_dat_o,
`ifdef OR1200_WB_CAB
   wb_cab_o,
`endif
`ifdef OR1200_WB_B3
   wb_cti_o, wb_bte_o,
`endif

   // Internal RISC bus
   biu_dat_i, biu_adr_i, biu_cyc_i, biu_stb_i, biu_we_i, biu_sel_i, biu_cab_i,
   biu_dat_o, biu_ack_o, biu_err_o
  );

  // ---------------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------------
  parameter dw = `OR1200_OPERAND_WIDTH;
  parameter aw = `OR1200_OPERAND_WIDTH;
  // 目前只叠4突发和叠8突发
  parameter bl = 4; /* Can currently be either 4 or 8 - the two optional line sizes for the OR1200. */

  //
  // RISC clock, reset and clock control
  //
  // 处理器外部Wishbone总线的时钟可以小于处理器的时钟，这样做的目的是降低系统功耗，
  // 这三个信号的作用就是在Wishbone总线时钟与处理器时钟不一致时，协调WB_BIU模块对
  // 处理器内部、外部的Wishbone总线信号，使其正常工作。
  // 其中wb_rst_i、wb_clk_i是外部Wishbone总线的复位信号、时钟信号，
  // clmode是时钟模式选择信号，长度是两位，可以设置如下：
  //    2’b00：外部Wishbone总线时钟wb_clk_i等于处理器时钟clk
  //    2’b01：外部Wishbone总线时钟wb_clk_i等于处理器时钟clk的一半
  //    2’b10：未定义
  //    2’b11：外部Wishbone总线时钟wb_clk_i等于处理器时钟clk的1/4
  input                   clk;        // RISC clock
  input                   rst;        // RISC reset
  input [1:0]             clmode;     // 00 WB=RISC, 01 WB=RISC/2, 10 N/A, 11 WB=RISC/4

  //
  // WISHBONE interface
  //
  // WB_BIU模块对处理器外部的接口名称都是wb_xxx_x的形式
  // 链接到外部的Instruct Bus上
  input                   wb_clk_i;  // clock input
  input                   wb_rst_i;  // reset input
  input                   wb_ack_i;  // normal termination
  input                   wb_err_i;  // termination w/ error
  input                   wb_rty_i;  // termination w/ retry
  input [dw-1:0]          wb_dat_i;  // input data bus
  output                  wb_cyc_o;  // cycle valid output
  output [aw-1:0]         wb_adr_o;  // address bus outputs
  output                  wb_stb_o;  // strobe output
  output                  wb_we_o;   // indicates write transfer
  output [3:0]            wb_sel_o;  // byte select outputs
  output [dw-1:0]         wb_dat_o;  // output data bus
`ifdef OR1200_WB_CAB
  output                  wb_cab_o;  // consecutive address burst
`endif
`ifdef OR1200_WB_B3
  output [2:0]            wb_cti_o;  // cycle type identifier
  output [1:0]            wb_bte_o;  // burst type extension
`endif

  //
  // Internal RISC interface
  //
  // WB_BIU模块对处理器内部的接口名称都是biu_xxx_x的形式
  // 链接到CPU的内部的ICACHE上
  input [dw-1:0]          biu_dat_i;  // input data bus
  input [aw-1:0]          biu_adr_i;  // address bus
  input                   biu_cyc_i;  // WB cycle
  input                   biu_stb_i;  // WB strobe
  input                   biu_we_i;   // WB write enable
  input                   biu_cab_i;  // CAB input, icfsm_burst or dcfsm_burst
  input [3:0]             biu_sel_i;  // byte selects
  output [31:0]           biu_dat_o;  // output data bus
  output                  biu_ack_o;  // ack output
  output                  biu_err_o;  // err output

  //
  // Registers
  //
  wire                    wb_ack;    // normal termination
  reg [aw-1:0]            wb_adr_o;  // address bus outputs
  reg                     wb_cyc_o;  // cycle output
  reg                     wb_stb_o;  // strobe output
  reg                     wb_we_o;   // indicates write transfer
  reg [3:0]               wb_sel_o;  // byte select outputs
`ifdef OR1200_WB_CAB
  reg                     wb_cab_o;  // CAB output
`endif
`ifdef OR1200_WB_B3
  // wb_cti_o是周期类型识别地址标签，wb_bte_o是突发类型扩展地址标签，
  // 两者都是在Wishbone规范B3版中使用的信号，作用是实现Wishbone寄存反馈总线周期，
  // 在处理器内部的Wishbone总线中没有这两个信号，所以WB_BIU模块还起一个不同Wishbone总线规范
  // 之间转换的作用。目前Wishbone已经制定了B4版，如果要使得OR1200支持新版本，只需要
  // 修改WB_BIU模块即可，提高了系统的移植能力。
  // 
  // CTI的定义：
  // CTI=3'b000 : 传统总线周期
  // CTI=3'b001 : 恒定地址突发总线周期
  // CTI=3'b010 : 递增突发总线周期
  // CTI=3'b011 : 预留总线周期
  // CTI=3'b100 : 预留总线周期
  // CTI=3'b101 : 预留总线周期
  // CTI=3'b110 : 预留总线周期
  // CTI=3'b111 : 突发结束
  reg [2:0]               wb_cti_o;  // cycle type identifier
  reg [1:0]               wb_bte_o;  // burst type extension
`endif
`ifdef OR1200_NO_DC
  reg [dw-1:0]            wb_dat_o;  // output data bus
`else
  assign wb_dat_o = biu_dat_i;    // No register on this - straight from DCRAM
`endif

`ifdef OR1200_WB_RETRY
  // 当外部设备回复重试信号时（wb_rty_i为1），WB_BIU模块会再次与外部设备
  // 开始Wishbone总线操作周期，只有当重试次数超过一个阈值（默认是64次）的时候，
  // 才向处理器报告Wishbone总线操作错误，这样可以提高系统的容错能力。
  reg [`OR1200_WB_RETRY-1:0]     retry_cnt;  // Retry counter
`else
  wire   retry_cnt;
  assign retry_cnt = 1'b0;
`endif

`ifdef OR1200_WB_B3
  reg [3:0]               burst_len;    // burst counter
`endif

  reg                     biu_stb_reg;  // WB strobe
  wire                    biu_stb;      // WB strobe
  reg                     wb_cyc_nxt;   // next WB cycle value
  reg                     wb_stb_nxt;   // next WB strobe value
  reg [2:0]               wb_cti_nxt;   // next cycle type identifier value

  reg                     wb_ack_cnt;   // WB ack toggle counter
  reg                     wb_err_cnt;   // WB err toggle counter
  reg                     wb_rty_cnt;   // WB rty toggle counter
  reg                     biu_ack_cnt;  // BIU ack toggle counter
  reg                     biu_err_cnt;  // BIU err toggle counter
  reg                     biu_rty_cnt;  // BIU rty toggle counter
  wire                    biu_rty;      // BIU rty indicator

  // WB_BIU状态机描述
  reg [1:0]               wb_fsm_state_cur;     // WB FSM - surrent state
  reg [1:0]               wb_fsm_state_nxt;     // WB FSM - next state
  // NND这个状态机的定义，真的是匪夷所思啊。。。
  wire [1:0]              WB_FSM_IDLE  = 2'h0;  // WB FSM state - IDLE
  wire [1:0]              WB_FSM_TRANS = 2'h1;  // WB FSM state - normal TRANSFER
  wire [1:0]              WB_FSM_LAST  = 2'h2;  // EB FSM state - LAST transfer

  //
  // WISHBONE I/F <-> Internal RISC I/F conversion
  //
  // assign wb_ack = wb_ack_i;
  // wb_ack是成功的应答信号
  assign wb_ack = wb_ack_i & !wb_err_i & !wb_rty_i;

  //
  // WB FSM - register part
  //
  always @(posedge wb_clk_i or `OR1200_RST_EVENT wb_rst_i) begin
    if (wb_rst_i == `OR1200_RST_VALUE)
      wb_fsm_state_cur <=  WB_FSM_IDLE;
    else
      wb_fsm_state_cur <=  wb_fsm_state_nxt;
    end

  //
  // WB burst tength counter
  //
  always @(posedge wb_clk_i or `OR1200_RST_EVENT wb_rst_i) begin
    if (wb_rst_i == `OR1200_RST_VALUE) begin
      burst_len <= 0;
    end
    else begin
      // burst counter
      if (wb_fsm_state_cur == WB_FSM_IDLE)
        // 最大突发为叠16，故是[3:0]，一共4位就可以了，节省资源
        burst_len <=  bl[3:0] - 2;

      else if (wb_stb_o & wb_ack)
        // 在突发传输过程中 
        burst_len <=  burst_len - 1;
    end
  end

  //
  // WB FSM - combinatorial part
  //
  // 主要用来计算wb_fsm_state_nxt的组合电路
  always @(wb_fsm_state_cur or burst_len or wb_err_i or wb_rty_i or wb_ack or
           wb_cti_o or wb_sel_o or wb_stb_o or wb_we_o or biu_cyc_i or
           biu_stb or biu_cab_i or biu_sel_i or biu_we_i)
  begin
    
    // States of WISHBONE Finite State Machine
    case(wb_fsm_state_cur)
    
    // IDLE
    WB_FSM_IDLE : begin
      // 检测到CYC和STB信号都有效时，在下个时钟周期开始进行数据的传递；
      // 注意：这里时组合电路，实际上还没有开始采样那！
      wb_cyc_nxt = biu_cyc_i & biu_stb;
      wb_stb_nxt = biu_cyc_i & biu_stb;
      // wb_cti_nxt = 3'b010 递增突发总线周期
      // wb_cti_nxt = 3'b111 突发结束
      // 因为我们没有使能ICACHE，故icbiu_cab_o=0；
      // 所以，这里的CTI=3'b111
      wb_cti_nxt = {!biu_cab_i, 1'b1, !biu_cab_i};
      
      if (biu_cyc_i & biu_stb)
        // 收到总线周期信号和总线选通信号后开始数据的传输
        wb_fsm_state_nxt = WB_FSM_TRANS;

      else
        wb_fsm_state_nxt = WB_FSM_IDLE;
    end
    
    // normal TRANSFER
    WB_FSM_TRANS : begin
      // CTI = 3'b111 : 突发结束
      // 当wb_ack有效时，!wb_ack将无效cyc_next和stb_next信号；
      wb_cyc_nxt    = !wb_stb_o | !wb_err_i & !wb_rty_i & !(wb_ack & wb_cti_o == 3'b111);
      wb_stb_nxt    = !wb_stb_o | !wb_err_i & !wb_rty_i & !wb_ack | !wb_err_i & !wb_rty_i & wb_cti_o == 3'b010 ;
      
      // 注意：或的优先级最低（与的优先级总是要比或的高）
      wb_cti_nxt[2] = wb_stb_o & wb_ack & burst_len == 'h0 | wb_cti_o[2];
      wb_cti_nxt[1] = 1'b1  ;
      wb_cti_nxt[0] = wb_stb_o & wb_ack & burst_len == 'h0 | wb_cti_o[0];

      if ((!biu_cyc_i | !biu_stb | !biu_cab_i | biu_sel_i != wb_sel_o | biu_we_i != wb_we_o) & wb_cti_o == 3'b010)
        // 最后的一次传输操作
        wb_fsm_state_nxt = WB_FSM_LAST;
      
      else if ((wb_err_i | wb_rty_i | wb_ack & wb_cti_o==3'b111) & wb_stb_o)
        // 感觉这个里面的wb_ack非常的重要，如果当前是一个STB传输过程
        // 若wb_ack应答了，就结束当前的传输了；
        wb_fsm_state_nxt = WB_FSM_IDLE;
      
      else
        wb_fsm_state_nxt = WB_FSM_TRANS;

    end
    
    // LAST transfer
    WB_FSM_LAST : begin
      wb_cyc_nxt = !wb_stb_o | !wb_err_i & !wb_rty_i & !(wb_ack & wb_cti_o == 3'b111);
      wb_stb_nxt = !wb_stb_o | !wb_err_i & !wb_rty_i & !(wb_ack & wb_cti_o == 3'b111);
      wb_cti_nxt[2] = wb_ack & wb_stb_o | wb_cti_o[2];
      wb_cti_nxt[1] = 1'b1                  ;
      wb_cti_nxt[0] = wb_ack & wb_stb_o | wb_cti_o[0];
      
      if ((wb_err_i | wb_rty_i | wb_ack & wb_cti_o == 3'b111) & wb_stb_o)
        // 本次传输周期结束
        wb_fsm_state_nxt = WB_FSM_IDLE;
      
      else
        wb_fsm_state_nxt = WB_FSM_LAST;
    end
    
    // default state
    default : begin
      wb_cyc_nxt = 1'bx;
      wb_stb_nxt = 1'bx;
      wb_cti_nxt = 3'bxxx;
      wb_fsm_state_nxt = 2'bxx;
    end
    endcase
  end

  //
  // WB FSM - output signals
  //
  // 状态机的输出信号
  always @(posedge wb_clk_i or `OR1200_RST_EVENT wb_rst_i) begin
  
    if (wb_rst_i == `OR1200_RST_VALUE) begin
      wb_cyc_o  <=  1'b0;
      wb_stb_o  <=  1'b0;
      // 在初始化的时候设置为突发终止
      wb_cti_o  <=  3'b111;
      wb_bte_o  <=  (16==bl) ? 2'b11: (bl==8) ? 
                                      2'b10 : 
                                      (bl==4) ? 2'b01 : 2'b00;
`ifdef OR1200_WB_CAB
      wb_cab_o  <=  1'b0;
`endif
      wb_we_o   <=  1'b0;
      wb_sel_o  <=  4'hf;
      wb_adr_o  <=  {aw{1'b0}};
`ifdef OR1200_NO_DC
      wb_dat_o  <=  {dw{1'b0}};
`endif
    end
    
    else begin
      wb_cyc_o  <=  wb_cyc_nxt;

      if (wb_ack & wb_cti_o == 3'b111)
        // 当前已经收到了传输完成应答且是总线突发终止，完成总线传输
        wb_stb_o  <=  1'b0;
        
      else
        wb_stb_o  <=  wb_stb_nxt;
        
`ifndef OR1200_NO_BURSTS
        wb_cti_o  <=  wb_cti_nxt;
`endif
        // BTE = 2'b00 : 线性突发
        // BTE = 2'b01 : 叠4突发
        // BTE = 2'b10 : 叠8突发
        // BTE = 2'b11 : 叠16突发
        wb_bte_o  <=  (16==bl) ? 
                        2'b11 : (bl==8) ? 
                          2'b10 : (bl==4) ? 
                            2'b01 : 2'b00;

`ifdef OR1200_WB_CAB
        wb_cab_o  <=  biu_cab_i;
`endif

      // we and sel - set at beginning of access
      if (wb_fsm_state_cur == WB_FSM_IDLE) begin
        wb_we_o   <=  biu_we_i;
        wb_sel_o  <=  biu_sel_i;
      end

      // adr - set at beginning of access and changed at every termination
      // 这个是重点中的重点
      if (wb_fsm_state_cur == WB_FSM_IDLE) begin
        wb_adr_o  <=  biu_adr_i;
      end
      
      else if (wb_stb_o & wb_ack) begin
        // 注意：这里已经有一次应答了，说明前一次的数据传输已经完成了，
        //       对操作的地址进行加一操作；
        
        // 分析叠4突发和叠8突发的不同
        // 这里比较精彩，一看就明白了
        if (bl==4) begin
          wb_adr_o[3:2]  <=  2'bx; //wb_adr_o[3:2] + 1;
        end

        if (bl==8) begin
          wb_adr_o[4:2]  <=  wb_adr_o[4:2] + 1;
        end

        if (16==bl) begin
          wb_adr_o[5:2]  <=  wb_adr_o[5:2] + 1;
        end
        
      end
      
`ifdef OR1200_NO_DC
      // dat - write data changed after avery subsequent write access
      if (!wb_stb_o) begin
        wb_dat_o   <=  biu_dat_i;
      end
`endif
    end
  end

  //
  // WB & BIU termination toggle counters
  //
  always @(posedge wb_clk_i or `OR1200_RST_EVENT wb_rst_i) begin
    if (wb_rst_i == `OR1200_RST_VALUE) begin
      wb_ack_cnt  <=  1'b0;
      wb_err_cnt  <=  1'b0;
      wb_rty_cnt  <=  1'b0;
    end
    
    else begin
      
      // WB ack toggle counter
      if (wb_fsm_state_cur == WB_FSM_IDLE | !(|clmode))
        wb_ack_cnt  <=  1'b0;

      else if (wb_stb_o & wb_ack)
        wb_ack_cnt  <=  !wb_ack_cnt;

      // WB err toggle counter
      if (wb_fsm_state_cur == WB_FSM_IDLE | !(|clmode))
        wb_err_cnt  <=  1'b0;

      else if (wb_stb_o & wb_err_i)
        wb_err_cnt  <=  !wb_err_cnt;

      // WB rty toggle counter
      if (wb_fsm_state_cur == WB_FSM_IDLE | !(|clmode))
        wb_rty_cnt  <=  1'b0;
      
      else if (wb_stb_o & wb_rty_i)
        wb_rty_cnt  <=  !wb_rty_cnt;
     end

  end

  // biu_stb_reg
  always @(posedge clk or `OR1200_RST_EVENT rst)
  begin
  
    if (rst == `OR1200_RST_VALUE)
      biu_stb_reg  <=  1'b0;
    
    else
      // BIU strobe
      if (biu_stb_i & !biu_cab_i & biu_ack_o)
        biu_stb_reg  <=  1'b0;
      
      else
        biu_stb_reg  <=  biu_stb_i;

  end

  assign biu_stb = biu_stb_i & biu_stb_reg;

  always @(posedge clk or `OR1200_RST_EVENT rst)
  begin
  
    if (rst == `OR1200_RST_VALUE) begin
      biu_ack_cnt  <=  1'b0;
      biu_err_cnt  <=  1'b0;
      biu_rty_cnt  <=  1'b0;
`ifdef OR1200_WB_RETRY
      retry_cnt    <= {`OR1200_WB_RETRY{1'b0}};
`endif
    end
    
    else begin
      
      // BIU ack toggle counter
      if (wb_fsm_state_cur == WB_FSM_IDLE | !(|clmode))
        biu_ack_cnt  <=  1'b0 ;
      
      else if (biu_ack_o)
        biu_ack_cnt  <=  !biu_ack_cnt ;
      
      // BIU err toggle counter
      if (wb_fsm_state_cur == WB_FSM_IDLE | !(|clmode))
        biu_err_cnt  <=  1'b0 ;
      
      else if (wb_err_i & biu_err_o)
        biu_err_cnt  <=  !biu_err_cnt ;
      
      // BIU rty toggle counter
      if (wb_fsm_state_cur == WB_FSM_IDLE | !(|clmode))
        biu_rty_cnt  <=  1'b0 ;
      
      else if (biu_rty)
        biu_rty_cnt  <=  !biu_rty_cnt;
        
`ifdef OR1200_WB_RETRY
      if (biu_ack_o | biu_err_o)
        retry_cnt    <=  {`OR1200_WB_RETRY{1'b0}};
      
      else if (biu_rty)
        retry_cnt    <=  retry_cnt + 1'b1;
`endif
    end
  end

  //
  // Input BIU data bus
  //
  assign  biu_dat_o  = wb_dat_i;

  //
  // Input BIU termination signals
  //
  // RTY产生的条件：
  // 1） 当前在传输的过程中
  // 2） 收到了rty信号
  // 3） 总线选通信号有效
  // 4） 因为clmode=2'b00，故wb_ack_cnt=0,biu_ack_cnt=0; wb_ack_cnt ~^ biu_ack_cnt=1
  //    若clmode!=2'b00的时候，
  // biu_rty_o会采用计数的机制，故：这里使用biu_rty信号，当biu_rty的次数大于预定的值时，触发错误信号；
  assign  biu_rty    = (wb_fsm_state_cur == WB_FSM_TRANS) & 
                        wb_rty_i & wb_stb_o &
                       (wb_rty_cnt ~^ biu_rty_cnt);
  assign  biu_ack_o  = (wb_fsm_state_cur == WB_FSM_TRANS) & 
                        wb_ack & wb_stb_o &
                       (wb_ack_cnt ~^ biu_ack_cnt);
  assign  biu_err_o  = (wb_fsm_state_cur == WB_FSM_TRANS) & 
                        wb_err_i & wb_stb_o &
                       (wb_err_cnt ~^ biu_err_cnt)
`ifdef OR1200_WB_RETRY
                        // nice的代码，非常的精彩
                       | biu_rty & retry_cnt[`OR1200_WB_RETRY-1];
`else
   ;
`endif

endmodule
