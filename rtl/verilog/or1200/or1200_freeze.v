//////////////////////////////////////////////////////////////////////
////                                                              ////
////  OR1200's Freeze logic                                       ////
////                                                              ////
////  This file is part of the OpenRISC 1200 project              ////
////  http://www.opencores.org/project,or1k                       ////
////                                                              ////
////  Description                                                 ////
////  Generates all freezes and stalls inside RISC                ////
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
// $Log: or1200_freeze.v,v $
// Revision 2.0  2010/06/30 11:00:00  ORSoC
// Minor update:
// Bugs fixed.
//

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "or1200_defines.v"

`define OR1200_NO_FREEZE            3'd0
`define OR1200_FREEZE_BYDC          3'd1
`define OR1200_FREEZE_BYMULTICYCLE  3'd2
`define OR1200_WAIT_LSU_TO_FINISH   3'd3
`define OR1200_WAIT_IC              3'd4

//
// Freeze logic (stalls CPU pipeline, ifetcher etc.)
//
// 产生RISC内部所有的暂停和延迟。
// freeze模块根据例外信息、模块停止信息、高速缓存出错信息等产生genpc_freeze, 
// if_freeze, id_freeze, ex_freeze, wb_freeze暂停信号去控制流水线中的各个模块。 
module or1200_freeze
  (
   // Clock and reset
   // 时钟和复位信号
   clk, rst,

   // Internal i/f
   // INPUT
   multicycle, wait_on, flushpipe, extend_flush, lsu_stall, if_stall,
   lsu_unstall, du_stall, mac_stall,
   force_dslot_fetch, abort_ex,
   icpu_ack_i, 
   icpu_err_i,
   
   // OUTPUT
   genpc_freeze,  // PC寄存器stall 
   if_freeze,     // IF/ID寄存器stall 
   id_freeze, 
   ex_freeze, 
   ma_freeze,
   wb_freeze, 
   saving_if_insn,
   fpu_done, 
   mtspr_done
  );

  //
  // I/O
  //
  // INPUT
  input                   clk;
  input                   rst;
  input  [`OR1200_MULTICYCLE_WIDTH-1:0] multicycle;
  input  [`OR1200_WAIT_ON_WIDTH-1:0]    wait_on;
  input                   flushpipe;
  input                   extend_flush;
  input                   lsu_stall;
  input                   if_stall;
  input                   lsu_unstall;
  input                   force_dslot_fetch;
  input                   abort_ex;
  input                   du_stall;
  input                   mac_stall;
  input                   saving_if_insn;
  input                   fpu_done;
  input                   mtspr_done;
  input                   icpu_ack_i;
  input                   icpu_err_i;
  
  // OUTPUT
  output                  genpc_freeze;
  output                  if_freeze;
  output                  id_freeze;
  output                  ex_freeze;
  output                  ma_freeze;
  output                  wb_freeze;

  //
  // Internal wires and regs
  //
  wire                    multicycle_freeze;
  reg  [`OR1200_MULTICYCLE_WIDTH-1:0]  multicycle_cnt;
  reg                     flushpipe_r;
  reg [`OR1200_WAIT_ON_WIDTH-1:0]  waiting_on;

  //
  // Pipeline freeze
  //
  // Rules how to create freeze signals:
  // 1. Not overwriting pipeline stages:
  // Freeze signals at the beginning of pipeline (such as if_freeze) can be
  // asserted more often than freeze signals at the of pipeline (such as
  // wb_freeze). In other words, wb_freeze must never be asserted when ex_freeze
  // is not. ex_freeze must never be asserted when id_freeze is not etc.
  //
  // 2. Inserting NOPs in the middle of pipeline only if supported:
  // At this time, only ex_freeze (and wb_freeze) can be deassrted when id_freeze
  // (and if_freeze) are asserted.
  // This way NOP is asserted from stage ID into EX stage.
  //
  // 1. 不覆盖写流水线的级： 
  // 在流水线开始的freeze信号（如：if_freeze)比在流水线的freeze信号（如：wb_freeze)声明的优先级高。
  // 换句话说，当ex_freeze没声明时，wb_freeze不能声明。当id_freeze没声明时，ex_freeze不能声明。
  //
  // 2. 只有在得到支持的情况下，才能在流水线中间插入NOP： 
  // 这时，当id_freeze (和if_freeze)声明时，仅ex_freeze（和wb_freeze)能被取消声明。
  // 这样，NOP能从ID级声明到EX级。 

  // 流水线暂停
  assign genpc_freeze = (du_stall & !saving_if_insn) | flushpipe_r; // 在du停止或刷新流水线时发出暂停genpc信号。
  
  // WB暂停了 => MA 必须暂停
  // MA暂停了 => EX 必须暂停
  // EX暂停了 => ID 必须暂停
  // ID暂停了 => IF 必须暂停
  
  // 1） 当ID段发生了冻结操作的时候，则IF段也必须要产生冻结操作；
  //    要不从IF来的数据就将会破坏ID段的信号；在id暂停信号或扩展刷新时发出暂停if信号。
  // 2) extend_flush 异常刷新信号
  assign if_freeze = id_freeze | extend_flush; 

  // id暂停信号=(lsu停止 | (lsu没有取停止 & if停止) | 多周期暂停 | 强制延迟槽支取) | du停止 | mac停止
  // ID段产生冻结的原因有：
  // 1） lsu_stall: 在LSU段，准备开始存储数据的时候
  // 2) ~lsu_unstall & if_stall: 当LSU没有应答而且if_stall有效（是不是总线竞争？？）
  assign id_freeze = (lsu_stall | (~lsu_unstall & if_stall) | multicycle_freeze
                   | (|waiting_on) | force_dslot_fetch) | du_stall;
  
  assign ex_freeze = ma_freeze;
  assign ma_freeze = wb_freeze;
  // 在EX阶段发现一个异常abort_ex
  // 问： 那MA和WB的指令是否要写完啊？
  assign wb_freeze = (lsu_stall | (~lsu_unstall & if_stall) | multicycle_freeze
                   | (|waiting_on)) | du_stall | abort_ex;

  //
  // registered flushpipe
  //
  // 注册的flushpipe，刷新命令
  always @(posedge clk or `OR1200_RST_EVENT rst)
    if (rst == `OR1200_RST_VALUE)
      flushpipe_r <=  1'b0;
    
    else if (icpu_ack_i | icpu_err_i) // 指令cache回应或出错
    //  else if (!if_stall)
      flushpipe_r <=  flushpipe;
    
    else if (!flushpipe)
      flushpipe_r <=  1'b0;

  //
  // Multicycle freeze
  //
  // 多周期的暂停
  assign multicycle_freeze = |multicycle_cnt; // 如果multicycle_cnt>0，则赋值为1。

  //
  // Multicycle counter
  //
  // 多周期计数器
  // 这里应该使用移位寄存器来代替减法器，可以提高速度 [<= JFDEBUG]
  always @(posedge clk or `OR1200_RST_EVENT rst)
    
    if (rst == `OR1200_RST_VALUE)
      multicycle_cnt <=  `OR1200_MULTICYCLE_WIDTH'd0;
    
    else if (|multicycle_cnt) // 如果multicycle_cnt>0，此时各位或运算后必为1。
      multicycle_cnt <=  multicycle_cnt - `OR1200_MULTICYCLE_WIDTH'd1; // 递减运算。
    
    else if (|multicycle & !ex_freeze) // 如果multicycle_cnt>0且ex没暂停。
      multicycle_cnt <=  multicycle;   // 来自ctrl模块算出的指令周期数。

  //
  // Waiting on generation
  //
  always @(posedge clk or `OR1200_RST_EVENT rst)
    if (rst == `OR1200_RST_VALUE)
      waiting_on <= 0;
    
    else if ((waiting_on == `OR1200_WAIT_ON_MULTMAC) & !mac_stall)
      waiting_on <= 0;
    
    else if ((waiting_on == `OR1200_WAIT_ON_FPU) & fpu_done)
      waiting_on <= 0;
    
    else if ((waiting_on == `OR1200_WAIT_ON_MTSPR) & mtspr_done)
      waiting_on <= 0;
    
    else if (!ex_freeze)
      waiting_on <= wait_on;

endmodule
