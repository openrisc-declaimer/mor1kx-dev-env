//////////////////////////////////////////////////////////////////////
///                                                               ////
/// bootrom                                                       ////
///                                                               ////
/// Assembly programs to be embedded inside system to aid boot    ////
///                                                               ////
/// Jin Fei,   jm8371@163.com                                     ////
///                                                               ////
//////////////////////////////////////////////////////////////////////

#define TEST_SEG_0

#include <or1k-asm.h>
#include "spr-defs.h"
#include "board.h"

#ifdef TEST_SEG_0
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
	
#endif /* TEST_SEG_0 */


#ifdef TEST_SEG_1
#endif /* TEST_SEG_1 */

#ifdef TEST_SEG_2
#endif /* TEST_SEG_2 */

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

