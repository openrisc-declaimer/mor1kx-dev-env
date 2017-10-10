//////////////////////////////////////////////////////////////////////
////                                                              ////
////  OR1200's Exception logic                                    ////
////                                                              ////
////  This file is part of the OpenRISC 1200 project              ////
////  http://www.opencores.org/project,or1k                       ////
////                                                              ////
////  Description                                                 ////
////  Handles all OR1K exceptions inside CPU block.               ////
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
// $Log: or1200_except.v,v $
//
// Revision 2.0  2010/06/30 11:00:00  ORSoC
// Major update:
// Structure reordered and bugs fixed.

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "or1200_defines.v"

`define OR1200_EXCEPTFSM_WIDTH  3
`define OR1200_EXCEPTFSM_IDLE   `OR1200_EXCEPTFSM_WIDTH'd0
`define OR1200_EXCEPTFSM_FLU1   `OR1200_EXCEPTFSM_WIDTH'd1
`define OR1200_EXCEPTFSM_FLU2   `OR1200_EXCEPTFSM_WIDTH'd2
`define OR1200_EXCEPTFSM_FLU3   `OR1200_EXCEPTFSM_WIDTH'd3
`define OR1200_EXCEPTFSM_FLU4   `OR1200_EXCEPTFSM_WIDTH'd4
`define OR1200_EXCEPTFSM_FLU5   `OR1200_EXCEPTFSM_WIDTH'd5

//
// Exception recognition and sequencing
//
// except模块处理OR1200的各种例外逻辑
module or1200_except
  (
   // Clock and reset
   // 时钟及复位信号
   clk, rst,

   // Internal i/f
   // 内部接口
   sig_ibuserr, sig_dbuserr, sig_illegal, sig_align, sig_range, sig_dtlbmiss,
   sig_dmmufault, sig_int, sig_syscall, sig_trap, sig_itlbmiss, sig_immufault,
   sig_tick, ex_branch_taken, genpc_freeze, id_freeze, ex_freeze, wb_freeze,
   if_stall,  if_pc, id_pc, ex_pc, wb_pc, id_flushpipe, ex_flushpipe,
   extend_flush, except_flushpipe, except_type, except_start, except_started,
   except_stop, except_trig, ex_void, abort_mvspr, branch_op, spr_dat_ppc,
   spr_dat_npc, datain, du_dsr, epcr_we, eear_we, esr_we, pc_we, epcr, eear,
   du_dmr1, du_hwbkpt, du_hwbkpt_ls_r, esr, sr_we, to_sr, sr, lsu_addr,
   abort_ex, icpu_ack_i, icpu_err_i, dcpu_ack_i, dcpu_err_i, sig_fp, fpcsr_fpee
  );

  //
  // I/O
  //
  input               clk;
  input               rst;
  input               sig_ibuserr;
  input               sig_dbuserr;
  input               sig_illegal;
  input               sig_align;
  input               sig_range;
  input               sig_dtlbmiss;
  input               sig_dmmufault;
  input               sig_int;
  input               sig_syscall;
  input               sig_trap;
  input               sig_itlbmiss;
  input               sig_immufault;
  input               sig_tick;
  input               sig_fp;
  input               fpcsr_fpee;
  input               ex_branch_taken;
  input               genpc_freeze;
  input               id_freeze;
  input               ex_freeze;
  input               wb_freeze;
  input               if_stall;
  input   [31:0]      if_pc;
  output  [31:0]      id_pc;
  output  [31:0]      ex_pc;
  output  [31:0]      wb_pc;
  input   [31:0]      datain;
  input   [`OR1200_DU_DSR_WIDTH-1:0]   du_dsr;
  input   [24:0]                       du_dmr1;
  input               du_hwbkpt;
  input               du_hwbkpt_ls_r;
  input               epcr_we;
  input               eear_we;
  input               esr_we;
  input               pc_we;
  output  [31:0]      epcr;
  output  [31:0]      eear;
  output  [`OR1200_SR_WIDTH-1:0]  esr;
  input   [`OR1200_SR_WIDTH-1:0]  to_sr;
  input               sr_we;
  input    [`OR1200_SR_WIDTH-1:0]  sr;
  input    [31:0]     lsu_addr;
  
  input               id_flushpipe;
  input               ex_flushpipe;
  output              except_flushpipe;
  output              extend_flush;
  
  output  [`OR1200_EXCEPT_WIDTH-1:0]   except_type;
  output              except_start;
  output              except_started;
  output  [13:0]      except_stop;
  output  [13:0]      except_trig;
  input               ex_void;
  input   [`OR1200_BRANCHOP_WIDTH-1:0] branch_op;
  output  [31:0]      spr_dat_ppc;
  output  [31:0]      spr_dat_npc;
  output              abort_ex;
  output              abort_mvspr;
  input               icpu_ack_i;
  input               icpu_err_i;
  input               dcpu_ack_i;
  input               dcpu_err_i;

  //
  // Internal regs and wires
  //
  reg  [`OR1200_EXCEPT_WIDTH-1:0]  except_type /* verilator public */;
  reg  [31:0]         id_pc /* verilator public */;
  reg                 id_pc_val;
  reg  [31:0]         ex_pc /* verilator public */;
  reg                 ex_pc_val;
  reg  [31:0]         wb_pc /* verilator public */;
  reg  [31:0]         dl_pc;
  reg  [31:0]         epcr;
  reg  [31:0]         eear;
  reg  [`OR1200_SR_WIDTH-1:0]    esr;
  reg  [2:0]          id_exceptflags;
  reg  [2:0]          ex_exceptflags;
  reg  [`OR1200_EXCEPTFSM_WIDTH-1:0]  state;
  reg                 extend_flush;
  reg                 extend_flush_last;
  reg                 ex_dslot /* verilator public */;
  reg                 delayed1_ex_dslot;
  reg                 delayed2_ex_dslot;
  wire                except_started;
  wire                except_flushpipe /* verilator public */;
  reg  [2:0]          delayed_iee;
  reg  [2:0]          delayed_tee;
  wire                int_pending;
  wire                tick_pending;
  wire                fp_pending;
  wire                range_pending;

  reg                 trace_trap;
  reg                 ex_freeze_prev;
  reg                 sr_ted_prev;
  reg                 dsr_te_prev;
  reg                 dmr1_st_prev;
  reg                 dmr1_bt_prev;

  wire dsr_te  = ex_freeze_prev ? dsr_te_prev : du_dsr[`OR1200_DU_DSR_TE];
  wire sr_ted  = ex_freeze_prev ? sr_ted_prev : sr[`OR1200_SR_TED];
  wire dmr1_st = ex_freeze_prev ? dmr1_st_prev: du_dmr1[`OR1200_DU_DMR1_ST] ;
  wire dmr1_bt = ex_freeze_prev ? dmr1_bt_prev: du_dmr1[`OR1200_DU_DMR1_BT] ;

  //
  // Simple combinatorial logic
  //
  // 组合逻辑
  assign except_started = extend_flush & except_start;

  assign except_start = (except_type != `OR1200_EXCEPT_NONE) & extend_flush;

  // 中断挂起=中断信号 & 中断使能 & 延迟中断使能 & EX级没暂停 & 分支地址没被取走 & EX级没有延迟槽 &  sprs输出的SR写使能信号为0
  assign int_pending = sig_int & (sr[`OR1200_SR_IEE] |
                                 (sr_we & to_sr[`OR1200_SR_IEE]))
                     & id_pc_val & delayed_iee[2] & ~ex_freeze & ~ex_branch_taken
                     & ~ex_dslot & ~(sr_we & ~to_sr[`OR1200_SR_IEE]);

  // tick定时器中断挂起=tick信号 & tick定时器中断使能 & ......
  assign tick_pending = sig_tick & (sr[`OR1200_SR_TEE] |
                       (sr_we & to_sr[`OR1200_SR_TEE])) & id_pc_val
                       & delayed_tee[2] & ~ex_freeze & ~ex_branch_taken
                       & ~ex_dslot & ~(sr_we & ~to_sr[`OR1200_SR_TEE]);

  assign fp_pending = sig_fp & fpcsr_fpee & ~ex_freeze & ~ex_branch_taken & ~ex_dslot;

`ifdef OR1200_IMPL_OVE
  assign range_pending =  sig_range & sr[`OR1200_SR_OVE] & ~ex_freeze &
                         ~ex_branch_taken & ~ex_dslot;
`else
  assign range_pending = 0;
`endif

  // Abort write into RF by load & other instructions
  // EX级异常中止=数据总线错误 | 数据MMU错误 | 数据TLB失靶 | 对齐错误 | 不合法指令
  // 这个信号表示异常中止Load指令与其它指令对RF的写。
  assign abort_ex = sig_dbuserr | sig_dmmufault | sig_dtlbmiss | sig_align |
                    sig_illegal | ((du_hwbkpt | trace_trap) & ex_pc_val
                 & !sr_ted & !dsr_te);

  // abort spr read/writes
  assign abort_mvspr  = sig_illegal | ((du_hwbkpt | trace_trap) & ex_pc_val
                     & !sr_ted & !dsr_te) ;

  // 输出到sprs的PPC寄存器的数据，是上一个PC。
  assign spr_dat_ppc = wb_pc;

  // 输出到sprs的NPC寄存器的数据，是下一个PC。
  assign spr_dat_npc = ex_void ? id_pc : ex_pc; // 如果EX级无效，使用id_pc值。

  //
  // Order defines exception detection priority
  // 整体的收集工作
  // 这个次序定义了例外检测优先级。
  // DSR寄存器(调试停止寄存器)定义了哪个例外引起核心停止例外例程的执行，并切换控制到开发接口。
  assign except_trig = {  //触发例外，这时对应DSR寄存器的位必须为0
                          ex_exceptflags[1] & ~du_dsr[`OR1200_DU_DSR_IME],
                          ex_exceptflags[0] & ~du_dsr[`OR1200_DU_DSR_IPFE],
                          ex_exceptflags[2] & ~du_dsr[`OR1200_DU_DSR_BUSEE],
                          sig_illegal       & ~du_dsr[`OR1200_DU_DSR_IIE],
                          sig_align         & ~du_dsr[`OR1200_DU_DSR_AE],
                          sig_dtlbmiss      & ~du_dsr[`OR1200_DU_DSR_DME],
                          sig_trap          & ~du_dsr[`OR1200_DU_DSR_TE],
                          sig_syscall       & ~du_dsr[`OR1200_DU_DSR_SCE] & ~ex_freeze,
                          sig_dmmufault     & ~du_dsr[`OR1200_DU_DSR_DPFE],
                          sig_dbuserr       & ~du_dsr[`OR1200_DU_DSR_BUSEE],
                          range_pending     & ~du_dsr[`OR1200_DU_DSR_RE],
                          fp_pending        & ~du_dsr[`OR1200_DU_DSR_FPE],
                          int_pending       & ~du_dsr[`OR1200_DU_DSR_IE],
                          tick_pending      & ~du_dsr[`OR1200_DU_DSR_TTE]
            };

  wire trace_cond  = !ex_freeze && !ex_void && (1'b0
`ifdef OR1200_DU_DMR1_ST
      ||  dmr1_st
`endif
`ifdef OR1200_DU_DMR1_BT
      ||  ((branch_op != `OR1200_BRANCHOP_NOP) && (branch_op != `OR1200_BRANCHOP_RFE) && dmr1_bt)
`endif
      );

  assign except_stop = {  //停止例外，这时对应DSR寄存器的位必须为1
                          tick_pending      & du_dsr[`OR1200_DU_DSR_TTE],
                          int_pending       & du_dsr[`OR1200_DU_DSR_IE],
                          ex_exceptflags[1] & du_dsr[`OR1200_DU_DSR_IME],
                          ex_exceptflags[0] & du_dsr[`OR1200_DU_DSR_IPFE],
                          ex_exceptflags[2] & du_dsr[`OR1200_DU_DSR_BUSEE],
                          sig_illegal       & du_dsr[`OR1200_DU_DSR_IIE],
                          sig_align         & du_dsr[`OR1200_DU_DSR_AE],
                          sig_dtlbmiss      & du_dsr[`OR1200_DU_DSR_DME],
                          sig_dmmufault     & du_dsr[`OR1200_DU_DSR_DPFE],
                          sig_dbuserr       & du_dsr[`OR1200_DU_DSR_BUSEE],
                          range_pending     & du_dsr[`OR1200_DU_DSR_RE],
                          sig_trap          & du_dsr[`OR1200_DU_DSR_TE],
                          fp_pending        & du_dsr[`OR1200_DU_DSR_FPE],
                          sig_syscall       & du_dsr[`OR1200_DU_DSR_SCE] & ~ex_freeze
      };

  always @(posedge clk or `OR1200_RST_EVENT rst) begin
    if (rst == `OR1200_RST_VALUE) begin
      trace_trap  <=  1'b0 ;
    end

    else if (!(trace_trap && !ex_pc_val)) begin
      trace_trap  <=  trace_cond & !dsr_te & !sr_ted ;
    end
  end

  always @(posedge clk or `OR1200_RST_EVENT rst) begin
    if (rst == `OR1200_RST_VALUE) begin
      ex_freeze_prev  <=  1'b0 ;
      sr_ted_prev     <=  1'b0 ;
      dsr_te_prev     <=  1'b0 ;
      dmr1_st_prev    <=  1'b0 ;
      dmr1_bt_prev    <=  1'b0 ;
    end
    else begin
      ex_freeze_prev  <=  ex_freeze ;
      if (!ex_freeze_prev || ex_void) begin
        sr_ted_prev     <=  sr     [`OR1200_SR_TED    ] ;
        dsr_te_prev     <=  du_dsr [`OR1200_DU_DSR_TE ] ;
        dmr1_st_prev    <=  du_dmr1[`OR1200_DU_DMR1_ST] ;
        dmr1_bt_prev    <=  du_dmr1[`OR1200_DU_DMR1_BT] ;
      end
    end
  end

`ifdef verilator
  // Function to access wb_pc (for Verilator). Have to hide this from
  // simulator, since functions with no inputs are not allowed in IEEE
  // 1364-2001.
  function [31:0] get_wb_pc;
    // verilator public
    get_wb_pc = wb_pc;
  endfunction // get_wb_pc

  // Function to access id_pc (for Verilator). Have to hide this from
  // simulator, since functions with no inputs are not allowed in IEEE
  // 1364-2001.
  function [31:0] get_id_pc;
    // verilator public
    get_id_pc = id_pc;
  endfunction // get_id_pc

  // Function to access ex_pc (for Verilator). Have to hide this from
  // simulator, since functions with no inputs are not allowed in IEEE
  // 1364-2001.
  function [31:0] get_ex_pc;
    // verilator public
    get_ex_pc = ex_pc;
  endfunction // get_ex_pc
  // Function to access except_type[3:0] (for Verilator). Have to hide this from
  // simulator, since functions with no inputs are not allowed in IEEE
  // 1364-2001.
  function [3:0] get_except_type;
    // verilator public
    get_except_type = except_type;
  endfunction // get_except_type
`endif

  //
  // PC and Exception flags pipelines
  //
  // PC和例外标识的流水线，得到id级的例外标识id_exceptflags
  always @(posedge clk or `OR1200_RST_EVENT rst) begin
    if (rst == `OR1200_RST_VALUE) begin
      id_pc <=  32'd0;
      id_pc_val <=  1'b0 ;
      id_exceptflags <=  3'b000;
    end
    else if (id_flushpipe) begin
      id_pc_val <=  1'b0 ;
      id_exceptflags <=  3'b000;
    end
    else if (!id_freeze) begin
      id_pc <=  if_pc;
      id_pc_val <=  1'b1 ;
      id_exceptflags <=  { sig_ibuserr, sig_itlbmiss, sig_immufault };
    end
  end

  //
  // delayed_iee
  //
  // SR[IEE] should not enable interrupts right away
  // when it is restored with l.rfe. Instead delayed_iee
  // together with SR[IEE] enables interrupts once
  // pipeline is again ready.
  //
  // delayed_iee ( Interrupt Exception Enabled)
  // 当SR[IEE]被指令l.rfe恢复时，它不应该马上使能中断。代替的方法是：一旦流水线再次准备好时，delayed_iee与SR[IEE]一起使能中断。
  always @(`OR1200_RST_EVENT rst or posedge clk)
    if (rst == `OR1200_RST_VALUE)
      delayed_iee <=  3'b000;

    else if (!sr[`OR1200_SR_IEE]) // SR寄存器IEE位为0，表示中断不被识别
      delayed_iee <=  3'b000;

    else
      // delayed_iee[2]有效时，才可能使中断挂起。下面移位方法延迟了3个周期才有效。
      delayed_iee <=  {delayed_iee[1:0], 1'b1};

  //
  // delayed_tee
  //
  // SR[TEE] should not enable tick exceptions right away
  // when it is restored with l.rfe. Instead delayed_tee
  // together with SR[TEE] enables tick exceptions once
  // pipeline is again ready.
  //
  // delayed_tee(Tick Timer Exception Enabled)
  //当SR[TEE]被指令l.rfe恢复时，它不应该马上使能中断。代替的方法是：一旦流水线再次准备好时，delayed_tee与SR[TEE]一起使能中断。
  always @(`OR1200_RST_EVENT rst or posedge clk)
    if (rst == `OR1200_RST_VALUE)
      delayed_tee <=  3'b000;

    else if (!sr[`OR1200_SR_TEE]) // SR寄存器TEE位为0，表示Tick Timer Exceptions不被识别
      delayed_tee <=  3'b000;

    else
      delayed_tee <=  {delayed_tee[1:0], 1'b1}; // 延迟3个时钟周期才有效

  //
  // PC and Exception flags pipelines
  //
  // PC和例外标识的流水线，得到ex级例外标识ex_exceptflags
  always @(posedge clk or `OR1200_RST_EVENT rst) begin

    if (rst == `OR1200_RST_VALUE) begin //复位时
      ex_dslot <=  1'b0;
      ex_pc <=  32'd0;
      ex_pc_val <=  1'b0 ;
      ex_exceptflags <=  3'b000;
      delayed1_ex_dslot <=  1'b0;
      delayed2_ex_dslot <=  1'b0;
    end

    else if (ex_flushpipe) begin //刷新流水线
      ex_dslot <=  1'b0;
      ex_pc_val <=  1'b0 ;
      ex_exceptflags <=  3'b000;
      delayed1_ex_dslot <=  1'b0;
      delayed2_ex_dslot <=  1'b0;
    end

    else if (!ex_freeze & id_freeze) begin // EX级运行，ID级暂停
      ex_dslot <=  1'b0;
      ex_pc <=  id_pc;
      ex_pc_val <=  id_pc_val ;
      ex_exceptflags <=  3'b000;
      delayed1_ex_dslot <=  ex_dslot;
      delayed2_ex_dslot <=  delayed1_ex_dslot;
    end

    else if (!ex_freeze) begin // EX级运行，ID级运行
      ex_dslot <=  ex_branch_taken;
      ex_pc <=  id_pc;
      ex_pc_val <=  id_pc_val ;
      ex_exceptflags <=  id_exceptflags;
      delayed1_ex_dslot <=  ex_dslot;
      delayed2_ex_dslot <=  delayed1_ex_dslot;
    end
  end

  //
  // PC and Exception flags pipelines
  //
  // PC和例外标识的流水线，得到wb_pc
  always @(posedge clk or `OR1200_RST_EVENT rst) begin
    if (rst == `OR1200_RST_VALUE) begin
      wb_pc <=  32'd0;
      dl_pc <=  32'd0;
    end
    else if (!wb_freeze) begin
      wb_pc <=  ex_pc;
      dl_pc <=  wb_pc;
    end
  end

  //
  // We have started execution of exception handler:
  //  1. Asserted for 3 clock cycles
  //  2. Don't execute any instruction that is still in pipeline and is not part of exception handler
  //
  // 刷新流水线
  // 我们已启动例外处理的运行：
  //  1. 声明了3个时钟周期。
  //  2. 不执行任何不在流水线且不是例外处理的指令。
  assign except_flushpipe = |except_trig & ~|state;

  //
  // Exception FSM that sequences execution of exception handler
  //
  // except_type signals which exception handler we start fetching in:
  //  1. Asserted in next clock cycle after exception is recognized
  //
  // 例外FSM(有限状态机)，用来使例外处理按次序执行。
  // 设置输出的例外类型、epcr、eear、esr寄存器值
  // 从except_type信号中取得例外处理例程：
  //   1.在例外被识别的下一个时钟周期声明。
  always @(posedge clk or `OR1200_RST_EVENT rst) begin

    if (rst == `OR1200_RST_VALUE) begin // 复位时，置为空闲状态
      state <=  `OR1200_EXCEPTFSM_IDLE; //空闲状态
      // 异常的类型
      except_type  <=  `OR1200_EXCEPT_NONE;
      extend_flush <=  1'b0;
      epcr <=  32'b0;   // 输出到例外编程计数器寄存器(EPCR)的值，是PC的拷贝值。
      eear <=  32'b0;   // 输出到例外有效地址寄存器(EEAR)的值，是EA的拷贝值。

      // 输出到例外超级监管寄存器(ESR)的值，是SR的拷贝值。
      esr <=  {2'h1, {`OR1200_SR_WIDTH-3{1'b0}}, 1'b1};

      extend_flush_last <=  1'b0;
    end

    else begin
`ifdef OR1200_CASE_DEFAULT
      case (state)  // synopsys parallel_case
`else
      case (state)  // synopsys full_case parallel_case
`endif
      `OR1200_EXCEPTFSM_IDLE: // 空闲状态
        if (except_flushpipe) begin // 如果例外刷新流水线
          state <=  `OR1200_EXCEPTFSM_FLU1; // 状态1
          extend_flush <= 1'b1;
          esr <=  sr_we ? to_sr : sr; // 拷贝SR寄存器到ESR寄存器

          // 根据不同的例外，设置例外类型、epcr和eear。
          casez (except_trig) // 例外触发优先级，从高到低。

`ifdef OR1200_EXCEPT_ITLBMISS
          14'b1?_????_????_????: begin
            except_type <=  `OR1200_EXCEPT_ITLBMISS;
            eear <=  ex_dslot ?
                      ex_pc : ex_pc;
                        epcr <=  ex_dslot ?
                          wb_pc : ex_pc;
          end
`endif

`ifdef OR1200_EXCEPT_IPF
        14'b01_????_????_????: begin
        except_type <=  `OR1200_EXCEPT_IPF; // 例外类型
        eear <=  ex_dslot ?
                ex_pc : delayed1_ex_dslot ?
                id_pc : delayed2_ex_dslot ?
                id_pc : id_pc;
        epcr <=  ex_dslot ?   // 将程序计数器地址存入EPCR寄存器
                wb_pc : delayed1_ex_dslot ?
                id_pc : delayed2_ex_dslot ?
                id_pc : id_pc;
        end
`endif

`ifdef OR1200_EXCEPT_BUSERR
          14'b00_1???_????_????: begin  // Insn. Bus Error
          except_type <=  `OR1200_EXCEPT_BUSERR;
          eear <=  ex_dslot ? wb_pc : ex_pc;
          epcr <=  ex_dslot ? wb_pc : ex_pc;
          end
`endif
`ifdef OR1200_EXCEPT_ILLEGAL
          14'b00_01??_????_????: begin
            except_type <=  `OR1200_EXCEPT_ILLEGAL;
            eear <=  ex_pc;
            epcr <=  ex_dslot ? wb_pc : ex_pc;
          end
`endif
`ifdef OR1200_EXCEPT_ALIGN
          14'b00_001?_????_????: begin
            except_type <=  `OR1200_EXCEPT_ALIGN;
            eear <=  lsu_addr;
            epcr <=  ex_dslot ? wb_pc : ex_pc;
          end
`endif
`ifdef OR1200_EXCEPT_DTLBMISS
          14'b00_0001_????_????: begin
            except_type <=  `OR1200_EXCEPT_DTLBMISS;
            eear <=  lsu_addr;
            epcr <=  ex_dslot ? wb_pc : delayed1_ex_dslot ? dl_pc : ex_pc;
          end
`endif
`ifdef OR1200_EXCEPT_TRAP
          14'b00_0000_1???_????: begin
             except_type <=  `OR1200_EXCEPT_TRAP;
             epcr <=  ex_dslot ?
               wb_pc : delayed1_ex_dslot ?
               id_pc : ex_pc;
          end
`endif
`ifdef OR1200_EXCEPT_SYSCALL
          14'b00_0000_01??_????: begin
             except_type <=  `OR1200_EXCEPT_SYSCALL;
             epcr <=  ex_dslot ?
               wb_pc : delayed1_ex_dslot ?
               id_pc : delayed2_ex_dslot ?
               id_pc : id_pc;
          end
`endif
`ifdef OR1200_EXCEPT_DPF
          14'b00_0000_001?_????: begin
             except_type <=  `OR1200_EXCEPT_DPF;
             eear <=  lsu_addr;
             epcr <=  ex_dslot ?
               wb_pc : delayed1_ex_dslot ?
               dl_pc : ex_pc;
          end
`endif
`ifdef OR1200_EXCEPT_BUSERR
          14'b00_0000_0001_????: begin  // Data Bus Error
             except_type <=  `OR1200_EXCEPT_BUSERR;
             eear <=  lsu_addr;
             epcr <=  ex_dslot ?
               wb_pc : delayed1_ex_dslot ?
               dl_pc : ex_pc;
          end
`endif
`ifdef OR1200_EXCEPT_RANGE
          14'b00_0000_0000_1???: begin
             except_type <=  `OR1200_EXCEPT_RANGE;
             epcr <=  ex_dslot ?
               wb_pc : delayed1_ex_dslot ?
               id_pc : delayed2_ex_dslot ?
               id_pc : id_pc;
          end
`endif
`ifdef OR1200_EXCEPT_FLOAT
          14'b00_0000_0000_01??: begin
             except_type <=  `OR1200_EXCEPT_FLOAT;
             epcr <=  id_pc;
          end
`endif
`ifdef OR1200_EXCEPT_INT  // 硬件中断
          14'b00_0000_0000_001?: begin
             except_type <=  `OR1200_EXCEPT_INT;
             epcr <=  id_pc;
          end
`endif
`ifdef OR1200_EXCEPT_TICK
          14'b00_0000_0000_0001: begin
             except_type <=  `OR1200_EXCEPT_TICK;
             epcr <=  id_pc;
          end
`endif
          default:
            except_type <=  `OR1200_EXCEPT_NONE;
        endcase
      end

      else if (pc_we) begin // PC写使能
        state <=  `OR1200_EXCEPTFSM_FLU1; // 进入状态1
        extend_flush <=  1'b1;
      end

      else begin
        if (epcr_we)
          epcr <=  datain;
        if (eear_we)
          eear <=  datain;
        if (esr_we)
          esr <=  {datain[`OR1200_SR_WIDTH-1], 1'b1, datain[`OR1200_SR_WIDTH-3:0]};
      end

      `OR1200_EXCEPTFSM_FLU1:  // 状态1
        if (icpu_ack_i | icpu_err_i | genpc_freeze)
          state <=  `OR1200_EXCEPTFSM_FLU2;

      `OR1200_EXCEPTFSM_FLU2:  // 状态2
`ifdef OR1200_EXCEPT_TRAP // 陷阱例外
        if (except_type == `OR1200_EXCEPT_TRAP) begin
          state <=  `OR1200_EXCEPTFSM_IDLE; // 进入空闲状态
          extend_flush <=  1'b0;
          extend_flush_last <=  1'b0;
          except_type <=  `OR1200_EXCEPT_NONE;
        end
      else
`endif
      state <=  `OR1200_EXCEPTFSM_FLU3;

    `OR1200_EXCEPTFSM_FLU3: begin // 状态3
      state <=  `OR1200_EXCEPTFSM_FLU4;
    end

    `OR1200_EXCEPTFSM_FLU4: begin // 状态4
      state <=  `OR1200_EXCEPTFSM_FLU5;
      extend_flush <=  1'b0;
      extend_flush_last <=  1'b0; // damjan
    end

`ifdef OR1200_CASE_DEFAULT
    default: begin
`else
    `OR1200_EXCEPTFSM_FLU5: begin // 状态5
`endif
      if (!if_stall && !id_freeze) begin
        state <=  `OR1200_EXCEPTFSM_IDLE; // 进入空闲状态
        except_type <=  `OR1200_EXCEPT_NONE;
        extend_flush_last <=  1'b0;
      end
    end
    endcase

    end
  end

endmodule
