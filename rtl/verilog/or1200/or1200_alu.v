//////////////////////////////////////////////////////////////////////
////                                                              ////
////  OR1200's ALU                                                ////
////                                                              ////
////  This file is part of the OpenRISC 1200 project              ////
////  http://www.opencores.org/project,or1k                       ////
////                                                              ////
////  Description                                                 ////
////  ALU                                                         ////
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
// $Log: or1200_alu.v,v $
// Revision 2.0  2010/06/30 11:00:00  ORSoC
// Minor update:
// Defines added, flags are corrected.

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "or1200_defines.v"

// 在ctrl模块中，指令被解码并编码成不同的信号线，如：alu_op，comp_op，shrot_op等，
// 以便于完成不同类型的操作。指令操作码及信号线的编码都定义在Or1200_define.v文件中。
// 算术逻辑运算单元根据alu_op输入16种不同操作码分别实现相应的加、与、异或、跳转、乘、除等基本操作运算。
// 根据输入comp_op信号完成两个操作数之间的比较操作，根据输入shrot_op信号完成操作数的移位旋转操作。
module or1200_alu
  (
    a, b, mult_mac_result, macrc_op,
   alu_op, alu_op2, comp_op,
   cust5_op, cust5_limm,
   result, flagforw, flag_we,
   ovforw, ov_we,
   cyforw, cy_we, carry, flag
  );

  // ---------------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------------
  parameter width = `OR1200_OPERAND_WIDTH;

  //
  // I/O
  //
  input  [width-1:0]      a;
  input  [width-1:0]      b;
  input  [width-1:0]      mult_mac_result;
  input                   macrc_op;
  input  [`OR1200_ALUOP_WIDTH-1:0]    alu_op;
  input  [`OR1200_ALUOP2_WIDTH-1:0]  alu_op2;
  input  [`OR1200_COMPOP_WIDTH-1:0]  comp_op;
  input  [4:0]            cust5_op;
  input  [5:0]            cust5_limm;
  output [width-1:0]      result;
  output                  flagforw;
  output                  flag_we;
  output                  cyforw;
  output                  cy_we;
  output                  ovforw;
  output                  ov_we;
  input                   carry;
  input                   flag;

  //
  // Internal wires and regs
  //
  reg  [width-1:0]        result;
  reg  [width-1:0]        shifted_rotated;
  reg  [width-1:0]        extended;
  reg  [width-1:0]        result_cust5;
  reg                     flagforw;
  reg                     flagcomp;
  reg                     flag_we;
  reg                     cyforw;
  reg                     cy_we;
  reg                     ovforw;
  reg                     ov_we;
  wire  [width-1:0]       comp_a;
  wire  [width-1:0]       comp_b;
  wire                    a_eq_b;
  wire                    a_lt_b;
  wire  [width-1:0]       result_sum;
  wire  [width-1:0]       result_and;
  wire                    cy_sum;
`ifdef OR1200_IMPL_SUB
  wire                    cy_sub;
`endif
  wire                    ov_sum;
  wire    [width-1:0]     carry_in;
  wire    [width-1:0]     b_mux;

  //
  // Combinatorial logic
  //
  //组合逻辑进行了比较、加、与的运算
  // comp_op[3]为1时，即寄存器间比较指令编码大于0x728，指令为有符号整数的比较；
  // 为0时是符号整数比较。a[31]^comp_op[3]表示无符号整数比较时，最高位为0；
  // 当为有符号整数比较且为有符号比较指令时，最高位为1。
  // comp_a和comp_b是用于比较的操作数
  assign comp_a = {a[width-1] ^ comp_op[3] , a[width-2:0]}; // 设置操作数符号位
  assign comp_b = {b[width-1] ^ comp_op[3] , b[width-2:0]};
`ifdef OR1200_IMPL_ALU_COMP1
  assign a_eq_b = (comp_a == comp_b););   // 相等
  assign a_lt_b = (comp_a < comp_b);      // 小于
`endif
`ifdef OR1200_IMPL_ALU_COMP3
  assign a_eq_b = !(|result_sum);
  // signed compare when comp_op[3] is set
  assign a_lt_b = comp_op[3] ? ((a[width-1] & !b[width-1]) |
                                (!a[width-1] & !b[width-1] & result_sum[width-1])|
                                (a[width-1] & b[width-1] & result_sum[width-1])):
                    // a < b if (a - b) subtraction wrapped and a[width-1] wasn't set
                    (result_sum[width-1] & !a[width-1]) |
                    // or if (a - b) wrapped and both a[width-1] and b[width-1] were set
                    (result_sum[width-1] & a[width-1] & b[width-1] );
`endif

`ifdef OR1200_IMPL_SUB
`ifdef OR1200_IMPL_ALU_COMP3
  assign cy_sub =  a_lt_b;
`else
  assign cy_sub = (comp_a < comp_b);
`endif
`endif

`ifdef OR1200_IMPL_ADDC
  assign carry_in = (alu_op==`OR1200_ALUOP_ADDC) ?
        {{width-1{1'b0}},carry} : {width{1'b0}};
`else
  assign carry_in = {width-1{1'b0}};
`endif

`ifdef OR1200_IMPL_ALU_COMP3
`ifdef OR1200_IMPL_SUB
  assign b_mux = ((alu_op==`OR1200_ALUOP_SUB) | (alu_op==`OR1200_ALUOP_COMP)) ? (~b)+1 : b;
`else
  assign b_mux = (alu_op==`OR1200_ALUOP_COMP) ? (~b)+1 : b;
`endif
`else // !`ifdef OR1200_IMPL_ALU_COMP3
`ifdef OR1200_IMPL_SUB
  assign b_mux = (alu_op==`OR1200_ALUOP_SUB) ? (~b)+1 : b;
`else
  assign b_mux = b;
`endif
`endif

  assign {cy_sum, result_sum} = (a + b_mux) + carry_in;
  // Numbers either both +ve and bit 31 of result set
  assign ov_sum = ((!a[width-1] & !b_mux[width-1]) & result_sum[width-1]) |
  // or both -ve and bit 31 of result clear
      ((a[width-1] & b_mux[width-1]) & !result_sum[width-1]);
  assign result_and = a & b; // 与运算

  //
  // Simulation check for bad ALU behavior
  //
`ifdef OR1200_WARNINGS
  // synopsys translate_off
  always @(result) begin
    if (result === 32'bx)
      $display("%t: WARNING: 32'bx detected on ALU result bus. Please check !", $time);
  end
  // synopsys translate_on
`endif

  //
  // Central part of the ALU
  //
  // alu_op信号控制的运算
  always @(alu_op or alu_op2 or a or b or result_sum or result_and or macrc_op
                  or shifted_rotated or mult_mac_result or flag or result_cust5 or carry
`ifdef OR1200_IMPL_ALU_EXT
                  or extended
`endif
  ) begin
`ifdef OR1200_CASE_DEFAULT
    casez (alu_op)    // synopsys parallel_case
`else
    casez (alu_op)    // synopsys full_case parallel_case
`endif
`ifdef OR1200_IMPL_ALU_FFL1
    `OR1200_ALUOP_FFL1: begin
`ifdef OR1200_CASE_DEFAULT
      casez (alu_op2) // synopsys parallel_case
`else
      casez (alu_op2) // synopsys full_case parallel_case
`endif
      0: begin // FF1
        // 指令l.ff1 rD,rA,rB，查找第1个bit位为1的位置。如：首次出现1的位置是bit7，则result为7。
        result = a[0] ? 1 : a[1] ? 2 : a[2] ? 3 : a[3] ? 4 : a[4] ? 5 : a[5] ? 6 : a[6] ? 7 : a[7] ? 8 : a[8] ? 9 : a[9] ? 10 : a[10] ? 11 : a[11] ? 12 : a[12] ? 13 : a[13] ? 14 : a[14] ? 15 : a[15] ? 16 : a[16] ? 17 : a[17] ? 18 : a[18] ? 19 : a[19] ? 20 : a[20] ? 21 : a[21] ? 22 : a[22] ? 23 : a[23] ? 24 : a[24] ? 25 : a[25] ? 26 : a[26] ? 27 : a[27] ? 28 : a[28] ? 29 : a[29] ? 30 : a[30] ? 31 : a[31] ? 32 : 0;
      end

      default: begin // FL1
        result = a[31] ? 32 : a[30] ? 31 : a[29] ? 30 : a[28] ? 29 : a[27] ? 28 : a[26] ? 27 : a[25] ? 26 : a[24] ? 25 : a[23] ? 24 : a[22] ? 23 : a[21] ? 22 : a[20] ? 21 : a[19] ? 20 : a[18] ? 19 : a[17] ? 18 : a[16] ? 17 : a[15] ? 16 : a[14] ? 15 : a[13] ? 14 : a[12] ? 13 : a[11] ? 12 : a[10] ? 11 : a[9] ? 10 : a[8] ? 9 : a[7] ? 8 : a[6] ? 7 : a[5] ? 6 : a[4] ? 5 : a[3] ? 4 : a[2] ? 3 : a[1] ? 2 : a[0] ? 1 : 0 ;
      end

      endcase // casez (alu_op2)

    end // case: `OR1200_ALUOP_FFL1
`endif //`ifdef OR1200_IMPL_ALU_FFL1

`ifdef OR1200_IMPL_ALU_CUST5
    // 用户指令l.cust5 rD,rA,rB,L,K。用户自定义的命令
    `OR1200_ALUOP_CUST5 : begin
      result = result_cust5;
    end
`endif

    `OR1200_ALUOP_SHROT : begin
      // 得到移位旋转结果，算出后存在shifted_rotated寄存器中
      result = shifted_rotated;
    end
`ifdef OR1200_IMPL_ADDC
    `OR1200_ALUOP_ADDC,
`endif
`ifdef OR1200_IMPL_SUB
    `OR1200_ALUOP_SUB, // 指令l.sub
`endif

    `OR1200_ALUOP_ADD : begin
      // 得到l.add加法指令结果，前面逻辑电路算出后存在result_sum寄存器中
      result = result_sum;
    end

    `OR1200_ALUOP_XOR : begin // 指令l.xor
      result = a ^ b;
    end

    `OR1200_ALUOP_OR  : begin // 指令l.or
      result = a | b;
    end
`ifdef OR1200_IMPL_ALU_EXT
    `OR1200_ALUOP_EXTHB  : begin
      result = extended;
    end

    `OR1200_ALUOP_EXTW  : begin
      result = extended;
    end
`endif

    `OR1200_ALUOP_MOVHI : begin
      // 指令l.macrc rD，一旦在MAC流水线完成，MAC的内容被放入通用寄存器rD，且MAC累加器清空。
      // mult_mac_result是从ALU外部输入的32位线。
      if (macrc_op) begin
        result = mult_mac_result;
      end
      else begin
        // 指令l.movhi rD,K；将输入线b上的0扩展16位立即数(即K)左移16位。
        result = b << 16;
      end
    end

`ifdef OR1200_MULT_IMPLEMENTED
`ifdef OR1200_DIV_IMPLEMENTED
    `OR1200_ALUOP_DIV,    // 有符号数除法指令l.div rD,rA,rB，即rD=rA/rB
    `OR1200_ALUOP_DIVU,   // 无符号数除法指令l.divu rD,rA,rB，即rD=rA/rB
`endif
    `OR1200_ALUOP_MUL,    // 有符号乘法指令l.mul rD,rA,rB，即rD=rA*rB
    `OR1200_ALUOP_MULU : begin
      result = mult_mac_result; // ALU外部计算结果由mult_mac_result线输入
    end
`endif
    `OR1200_ALUOP_CMOV: begin
      // 指令l.cmov rD,rA,rB，根据标识SR[F ]确定赋值，即：rD[31:0] <- SR[F] ? rA[31:0] : rB[31:0]
      result = flag ? a : b;
    end

`ifdef OR1200_CASE_DEFAULT
    default: begin
`else
    `OR1200_ALUOP_COMP, `OR1200_ALUOP_AND: begin
`endif
      // ALU外部输入线result_and值
      result=result_and;
    end
    endcase
  end

  //
  // Generate flag and flag write enable
  //
  // 产生标识并设置标识写使能
  always @(alu_op or result_sum or result_and or flagcomp)
  begin
    casez (alu_op)    // synopsys parallel_case
`ifdef OR1200_ADDITIONAL_FLAG_MODIFIERS
`ifdef OR1200_IMPL_ADDC
    `OR1200_ALUOP_ADDC,
`endif
    `OR1200_ALUOP_ADD : begin
      // 算出需要写的标识值，加法结果值是否为0
      flagforw = (result_sum == 32'h0000_0000);
      // 设置标识写使能
      flag_we = 1'b1;
    end

    `OR1200_ALUOP_AND: begin
      flagforw = (result_and == 32'h0000_0000);
      flag_we = 1'b1;
    end
`endif
      `OR1200_ALUOP_COMP: begin
        flagforw = flagcomp;
        flag_we = 1'b1;
      end
      default: begin
        flagforw = flagcomp;
        flag_we = 1'b0;
      end
    endcase
  end

  //
  // Generate SR[CY] write enable
  //
  // 产生SR[CY]写使 能信号，SR[CY]是状态寄存器的carry位(进位标识)
  always @(alu_op or cy_sum
`ifdef OR1200_IMPL_CY
`ifdef OR1200_IMPL_SUB
    or cy_sub // cy_csub进位标识产生于减法运算中
`endif
`endif
  )
  begin

    casez (alu_op)    // synopsys parallel_case
`ifdef OR1200_IMPL_CY
`ifdef OR1200_IMPL_ADDC
    `OR1200_ALUOP_ADDC,
`endif
    `OR1200_ALUOP_ADD : begin
      cyforw = cy_sum;  // 加法进位
      cy_we  = 1'b1;    // 设置SR[CY]写使能信号
    end
`ifdef OR1200_IMPL_SUB
    `OR1200_ALUOP_SUB: begin
      cyforw = cy_sub;
      cy_we = 1'b1;
    end
`endif
`endif
      default: begin
        cyforw = 1'b0;
        cy_we = 1'b0;
      end
    endcase
  end

  //
  // Generate SR[OV] write enable
  //
  always @(alu_op or ov_sum) begin
    casez (alu_op)    // synopsys parallel_case
`ifdef OR1200_IMPL_OV
`ifdef OR1200_IMPL_ADDC
      `OR1200_ALUOP_ADDC,
`endif
`ifdef OR1200_IMPL_SUB
      `OR1200_ALUOP_SUB,
`endif
      `OR1200_ALUOP_ADD : begin
        ovforw = ov_sum;
        ov_we = 1'b1;
      end
`endif
      default: begin
        ovforw = 1'b0;
        ov_we = 1'b0;
      end
    endcase
  end

  //
  // Shifts and rotation
  //
  // 移位和旋转，移位结果放入寄存器shifted_rotated，以便输出
  always @(alu_op2 or a or b) begin

    case (alu_op2)    // synopsys parallel_case
    `OR1200_SHROTOP_SLL :
      // 指令l.sll rD,rA,rB。 rA左向移位rB[4:0]值，低位补0
      shifted_rotated = (a << b[4:0]);

    `OR1200_SHROTOP_SRL :
      // 指令l.sll rD,rA,rB。 rA右向移位rB[4:0]值，高位补0
      shifted_rotated = (a >> b[4:0]);

`ifdef OR1200_IMPL_ALU_ROTATE
    `OR1200_SHROTOP_ROR :
      // 指令l.ror rD,rA,rB。rA右向旋转移位rB[4:0]值
      shifted_rotated = (a << (6'd32-{1'b0,b[4:0]})) | (a >> b[4:0]);
`endif
    default:
      shifted_rotated = ({32{a[31]}} << (6'd32-{1'b0, b[4:0]})) | a >> b[4:0];

    endcase
  end

  //
  // First type of compare implementation
  //
  // 两种比较指令实现方法，在最终电路中中选择一种方法。
  // 第一种方法的比较实现，a_eq_b(相等)和a_lt_b(小于)在前面逻辑电路已实现。
  // 比较结果放在寄存器flagcomp中
`ifdef OR1200_IMPL_ALU_COMP1
  always @(comp_op or a_eq_b or a_lt_b) begin

    case(comp_op[2:0])  // synopsys parallel_case

    `OR1200_COP_SFEQ:     // 等于
      flagcomp = a_eq_b;
    `OR1200_COP_SFNE:     // 不等于
      flagcomp = ~a_eq_b;
    `OR1200_COP_SFGT:     // 大于
      flagcomp = ~(a_eq_b | a_lt_b);
    `OR1200_COP_SFGE:     // 大于或等于
      flagcomp = ~a_lt_b;
    `OR1200_COP_SFLT:     // 小于
      flagcomp = a_lt_b;
    `OR1200_COP_SFLE:     // 小于或等于
      flagcomp = a_eq_b | a_lt_b;

    default:
      flagcomp = 1'b0;
    endcase
  end
`endif

  //
  // Second type of compare implementation
  //
  // 第二种方法的比较实现，在自己的电路中实现比较
`ifdef OR1200_IMPL_ALU_COMP2
  always @(comp_op or comp_a or comp_b) begin

    case(comp_op[2:0])  // synopsys parallel_case
    `OR1200_COP_SFEQ:
      flagcomp = (comp_a == comp_b);
    `OR1200_COP_SFNE:
      flagcomp = (comp_a != comp_b);
    `OR1200_COP_SFGT:
      flagcomp = (comp_a > comp_b);
    `OR1200_COP_SFGE:
      flagcomp = (comp_a >= comp_b);
    `OR1200_COP_SFLT:
      flagcomp = (comp_a < comp_b);
    `OR1200_COP_SFLE:
      flagcomp = (comp_a <= comp_b);
    default:
      flagcomp = 1'b0;
    endcase
  end
`endif //`ifdef OR1200_IMPL_ALU_COMP2

`ifdef OR1200_IMPL_ALU_COMP3
  always @(comp_op or a_eq_b or a_lt_b) begin
    case(comp_op[2:0])  // synopsys parallel_case
    `OR1200_COP_SFEQ:
      flagcomp = a_eq_b;
    `OR1200_COP_SFNE:
      flagcomp = ~a_eq_b;
    `OR1200_COP_SFGT:
      flagcomp = ~(a_eq_b | a_lt_b);
    `OR1200_COP_SFGE:
      flagcomp = ~a_lt_b;
    `OR1200_COP_SFLT:
      flagcomp = a_lt_b;
    `OR1200_COP_SFLE:
      flagcomp = a_eq_b | a_lt_b;
    default:
      flagcomp = 1'b0;
    endcase
  end
`endif

`ifdef OR1200_IMPL_ALU_EXT
  always @(alu_op or alu_op2 or a) begin

    casez (alu_op2)
    `OR1200_EXTHBOP_HS : extended = {{16{a[15]}},a[15:0]};
    `OR1200_EXTHBOP_BS : extended = {{24{a[7]}},a[7:0]};
    `OR1200_EXTHBOP_HZ : extended = {16'd0,a[15:0]};
    `OR1200_EXTHBOP_BZ : extended = {24'd0,a[7:0]};
    default: extended = a; // Used for l.extw instructions
    endcase // casez (alu_op2)

  end
`endif

  //
  // l.cust5 custom instructions
  //
  // l.cust5 用户定制指令
`ifdef OR1200_IMPL_ALU_CUST5
  // Examples for move byte, set bit and clear bit
  //
  // l.cust5定制指令样例：移动字节、设置和清除bit位
  always @(cust5_op or cust5_limm or a or b) begin
  
    casez (cust5_op)    // synopsys parallel_case
    5'h1 : begin
      casez (cust5_limm[1:0]) // 各种字节组合方式
      2'h0: result_cust5 = {a[31:8],  b[7:0]};
      2'h1: result_cust5 = {a[31:16], b[7:0], a[7:0]};
      2'h2: result_cust5 = {a[31:24], b[7:0], a[15:0]};
      2'h3: result_cust5 = {b[7:0],   a[23:0]};
      endcase
    end
    5'h2 :
      result_cust5 = a | (1 << cust5_limm); // 向左移位指令
    5'h3 :
      result_cust5 = a & (32'hffffffff ^ (1 << cust5_limm));
    //
    // *** Put here new l.cust5 custom instructions ***
    //
    // 新的l.cust5定制指令放在这里
    default: begin
      result_cust5 = a;
    end
    endcase
  end // always @ (cust5_op or cust5_limm or a or b)
`endif

endmodule
