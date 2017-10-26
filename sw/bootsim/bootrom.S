//////////////////////////////////////////////////////////////////////
///                                                               ////
/// bootrom                                                       ////
///                                                               ////
/// Assembly programs to be embedded inside system to aid boot    ////
///                                                               ////
/// Jin Fei,   jm8371@163.com                                     ////
///                                                               ////
//////////////////////////////////////////////////////////////////////

#include <or1k-asm.h>
#include "spr-defs.h"
#include "board.h"

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

