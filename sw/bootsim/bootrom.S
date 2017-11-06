//////////////////////////////////////////////////////////////////////
///                                                               ////
/// bootrom                                                       ////
///                                                               ////
/// Assembly programs to be embedded inside system to aid boot    ////
///                                                               ////
/// Jin Fei,   jm8371@163.com                                     ////
///                                                               ////
//////////////////////////////////////////////////////////////////////

// #define TEST_SEG_0
// #define TEST_SEG_MUL
// #define TEST_SEG_BRANCH
#define TEST_SEG_LSU
// #define TEST_SEG_MMU
// #define TEST_SEG_CACHE

#include <or1k-asm.h>
#include "spr-defs.h"
#include "board.h"

// 基本的流水线的测试
#ifdef TEST_SEG_BASE
	// ---------------------------------------------------------------------------
	// Base Arithmetic Instruction
	// ---------------------------------------------------------------------------
	// R0 = 0
	// R1 = 3
	// R2 = 4
	// R3 = 1
	// R5 = R1 + R2   : FINISH
	// R6 = R5 + R3   : WB 
	// R7 = R5 + R1   : MA
	// R8 = R5 + R2   : EX
	// NOP
	// NOP
	// NOP
	// NOP
	// NOP
	// R5 = R1 + R2   : FINISH
	// R6 = R3 + R5   : WB 
	// R7 = R1 + R5   : MA
	// R8 = R2 + R5   : EX
	// NOP
	// NOP
	// NOP
	// NOP
	// NOP
	// ---------------------------------------------------------------------------

	// R0 = 0
	l.movhi r0,0
	l.ori r0,r0,0
	
	// R1 = 3
	l.movhi r1,0
	l.ori r1,r0,3
	
	// R2 = 4
	l.movhi r2,0
	l.ori r2,r0,4
	
	// R3 = 1
	l.movhi r3,0
	l.ori r3,r0,1
	
	// R5 = R1 + R2   : FINISH
	l.add r5,r1,r2
	
	// R6 = R5 + R3   : WB 
	l.add r6,r5,r3
	
	// R7 = R5 + R1   : MA
	l.add r7,r5,r1
	
	// R8 = R5 + R2   : EX
	l.add r8,r5,r2
	
	// 排空流水线
	l.nop
	l.nop
	l.nop
	l.nop
	l.nop
	
	// R5 = R1 + R2   : FINISH
	l.add r5,r1,r2
	
	// R6 = R3 + R5   : WB 
	l.add r6,r3,r5
	
	// R7 = R1 + R5   : MA
	l.add r7,r1,r5
	
	// R8 = R2 + R5   : EX	
	l.add r8,r2,r5

	// 排空流水线
	l.nop
	l.nop
	l.nop
	l.nop
	l.nop
	
#endif /* TEST_SEG_BASE */

// 乘除法指令的测试
#ifdef TEST_SEG_MUL
	// ---------------------------------------------------------------------------
	// MUL Instruction
	// ---------------------------------------------------------------------------
	// R0 = 0
	// R1 = 2
	// R2 = 3
	// R3 = 4
	// R5 = R1 * R2   : FINISH
	// NOP
	// NOP
	// NOP
	// NOP
	// NOP
	// ---------------------------------------------------------------------------
	// R0 = 0
	l.movhi r0,0
	l.ori r0,r0,0
	
	// R1 = 2
	l.movhi r1,0
	l.ori r1,r0,2
	
	// R2 = 3
	l.movhi r2,0
	l.ori r2,r0,3
	
	// R3 = 4
	l.movhi r3,0
	l.ori r3,r0,4
	
	// R5 = R1 * R2
	l.mul r5,r1,r2
	
	// 排空流水线
	l.nop
	l.nop
	l.nop
	l.nop
	l.nop    
#endif /* TEST_SEG_MUL */

// 跳转指令的测试
#ifdef TEST_SEG_BRANCH
	// ---------------------------------------------------------------------------
	// BRANCH Instruction
	// ---------------------------------------------------------------------------
	// R0 = 0
	// R1 = 10
	// R2 = 0
    // R3 = 0
    // 
    // __TEST_LOOP:
    // 
    // if(R2==R1) goto __TEST_EXIT:
    // NOP
    //
	//  R3 = R3 + 1
    //
    // Jump __TEST_LOOP
    // R2 = R2 + 1
    // NO_LOOP_CNT:
    // R3 = R3 + 2
	// R3 = R3 + 2
	// NOP
	// NOP
	// NOP
	// NOP
	// ---------------------------------------------------------------------------
    // R0 = 0
	l.movhi r0,0
	l.ori r0,r0,0
	
	// R1 = 10
	l.movhi r1,0
	l.ori r1,r0,2
	
	// R2 = 0
	l.movhi r2,0
    
	// R3 = 0
	l.movhi r3,0
	
__TEST_LOOP:
    
    l.sfeq R2,R1
    l.bf __TEST_EXIT
    l.nop	 0x0
    
    l.addi r3,r3,0x1
    
    l.j	__TEST_LOOP
    l.addi r2,r2,0x01
	
__TEST_EXIT:
    l.addi r3,r3,0x2
    l.addi r3,r3,0x2
	// 排空流水线
	l.nop
	l.nop
	l.nop
	l.nop
	l.nop
#endif /* TEST_SEG_BRANCH */

// 存储指令的测试
#ifdef TEST_SEG_LSU
	// ---------------------------------------------------------------------------
	// LSU Instruction
	// ---------------------------------------------------------------------------
	// ---------------------------------------------------------------------------

    l.movhi	 r13, 0x0010;	    /* r13:memory pointer     <== r13 = 0x0010_0000 */
    l.movhi	 r15, 0x0020;	    /* r15:max memory address <== r15 = 0x0020_0000 */
    l.movhi r19,0
    l.ori	 r19, r19, 0xAA;	/* r19:=0xAA              <== r19 = r19 | 0xAA */
    l.movhi r21,0
    l.ori	 r21, r21, 0x55;	/* r21:=0x55              <== r21 = r21 | 0xAA */

__ram_selftest_loop:	            /* loop start */

    l.sfeq	 r13, r15;	            /* if (r13 == r15) goto __ram_exit  	 <== if(r13 == r15) then SR[F] = 1 else SR[F] = 0 */
    l.bf	 __ram_selftest_exit;	/* <== if(SR[F] == 1) jump __ram_selftest_exit */
    l.nop	 0x0;	                /* <== Delay Slots */
    l.lbz	 r17, 0(r13);	        /* r17 = *(r13), save orig data  	 <== r17[7:0] = (r13+0), r17[31:8] = 0 */

    l.sb	 0(r13), r19;	        /* *(r13) = r19 = 0xAA, write into ram <== (r13+0) = 0xAA */
    l.lbz	 r23, 0(r13);	        /* r23 = *(r13), read out <== r23[7:0] = (r13+0), r23[31:8] = 0 */
    l.sfeq	 r23, r19;	            /* if (r23 != r19) goto __ram_selftest_fail <== if(0xAA == r23) then SR[F] = 1 else SR[F] = 0 */
    l.bnf	 __ram_selftest_fail;	/* <== if(SR[F]==0) jump __ram_selftest_fail */
    l.nop	 0x0;	                /* <== Delay Slots */

    l.sb	 0(r13), r21;	        /* *(r13) = r19 = 0x55, write into ram <== (r13+0) = 0xAA */
    l.lbz	 r23, 0(r13);	        /* r23 = *(r13), read out <== r23[7:0] = (r13+0), r23[31:8] = 0 */
    l.sfeq	 r23, r21;	            /* if (r23 != r19) goto __ram_selftest_fail <== if(0xAA == r23) then SR[F] = 1 else SR[F] = 0 */
    l.bnf	 __ram_selftest_fail;	/* <== if(SR[F]==0) jump __ram_selftest_fail */
    l.nop	 0x0;	                /* <== Delay Slots */

    l.sb	 0(r13), r17;	 /* recover orig data back <== (r13+0) = r17[7:0]; 恢复以前的数据 */

    l.addi	 r13, r13, 0x1;	 /* ++r13 	 <== r13 = r13 + 1 */
    l.j	 __ram_selftest_loop;	/* jump to loop start <== jump __ram_selftest_loop */
    l.nop	 0x0;	        /* <== Delay Slots */

__ram_selftest_fail:	
    l.addi 	r11, r0, 0xffffffff;	 /* <== r11 = 0xffffffff */
    l.lwz	 r2, 0x0(r1);	/* <== r2 = (r1) */
    l.jr	 r9;	        /* <== jump R9 */
    l.addi	 r1, r1, 0x4;	/* <== r1 = r1 + 4 <== NOTE, it is a delay slot */
    
__ram_selftest_exit:
    l.nop
    l.nop
    l.nop
    l.nop
    l.nop
    l.nop    
#endif /* TEST_SEG_LSU */

// MMU的测试
#ifdef TEST_SEG_MMU
#endif /* TEST_SEG_MMU */

// CACHE的测试的测试
#ifdef TEST_SEG_CACHE
#endif /* TEST_SEG_CACHE */

#ifdef TEST_SEG_SPR
	// ---------------------------------------------------------------------------
	// Base SPR Register Operator
	// ---------------------------------------------------------------------------
	l.movhi r0,0
	l.ori r0,r0,0
	
	/* get SR register */
	l.mfspr r1, r0, SPR_SR
	
	/* get VR register */
	l.mfspr r2,r0,SPR_VR

	/* get UPR register */
	l.mfspr r2, r0, SPR_UPR

	/* get config register */
	l.mfspr r3, r0, SPR_CPUCFGR
	
	/* get config register */
	l.mfspr r4, r0, SPR_CPUCFGR

	l.mfspr r5, r0, SPR_DMMUCFGR
	l.mfspr r6, r0, SPR_IMMUCFGR
	l.mfspr r7, r0, SPR_DCCFGR
	l.mfspr r8, r0, SPR_ICCFGR
	l.mfspr r9, r0, SPR_DCFGR
	l.mfspr r10, r0, SPR_PCCFGR
	l.mfspr r11, r0, SPR_REVIR_BASE
	l.mfspr r12, r0, SPR_REVIR_LAST
	l.mfspr r13, r0, SPR_NPC
	
	l.mfspr r14, r0, SPR_SR
	
	l.mfspr r15, r0, SPR_PPC
	l.mfspr r16, r0, SPR_FPCSR
	l.mfspr r17, r0, SPR_EPCR_BASE
	l.mfspr r18, r0, SPR_EPCR_LAST
	l.mfspr r19, r0, SPR_EEAR_BASE
	l.mfspr r20, r0, SPR_EEAR_LAST
	l.mfspr r21, r0, SPR_ESR_BASE
	l.mfspr r22, r0, SPR_ESR_LAST
	l.mfspr r23, r0, SPR_GPR_BASE
#endif /* TEST_SEG_SPR */

