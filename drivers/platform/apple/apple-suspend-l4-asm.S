/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Apple Silicon experimental "Level 4" (power-gated core) sleep helpers
 */

#include <linux/linkage.h>
#include <asm/assembler.h>

	.text

/*
 * apple_l4_clean_dcache_all()
 *
 * Clean (not invalidate) every data cache level up to the Level of Coherency
 * by set/way, so that all dirty lines (including the cpu_suspend() stash and
 * the stack that cpu_resume() will use with the MMU off) reach DRAM before the
 * core loses power. Same loop as m1n1's dcsw_op_all(DCCSW) used by the
 * working bare-metal Level 3/4 tests.
 *
 * Clobbers x0-x12.
 */
SYM_FUNC_START(apple_l4_clean_dcache_all)
	dmb	sy
	mrs	x0, clidr_el1
	and	x3, x0, #0x7000000
	lsr	x3, x3, #23			// LoC << 1
	cbz	x3, .Lfinished
	mov	x10, #0				// cache level * 2
.Lloop1:
	add	x2, x10, x10, lsr #1
	lsr	x1, x0, x2
	and	x1, x1, #7
	cmp	x1, #2
	b.lt	.Lskip				// no data cache at this level
	msr	csselr_el1, x10
	isb
	mrs	x1, ccsidr_el1
	and	x2, x1, #7
	add	x2, x2, #4			// log2(line size in bytes)
	mov	x4, #0x3ff
	and	x4, x4, x1, lsr #3		// max way number
	clz	w5, w4				// way field shift
	mov	x7, #0x7fff
	and	x7, x7, x1, lsr #13		// max set number
.Lloop2:
	mov	x9, x4
.Lloop3:
	lsl	x6, x9, x5
	lsl	x12, x7, x2
	orr	x11, x10, x6
	orr	x11, x11, x12
	dc	csw, x11
	subs	x9, x9, #1
	b.ge	.Lloop3
	subs	x7, x7, #1
	b.ge	.Lloop2
.Lskip:
	add	x10, x10, #2
	cmp	x3, x10
	b.gt	.Lloop1
.Lfinished:
	mov	x10, #0
	msr	csselr_el1, x10
	dsb	sy
	isb
	ret
SYM_FUNC_END(apple_l4_clean_dcache_all)

/*
 * apple_l4_final_wfi(): the WFI that, with ACC_OVRD.DEEP_SLEEP and
 * CYC_OVRD deep WFI set, power-gates the core. Returns only if the core was
 * woken without being gated.
 */
SYM_FUNC_START(apple_l4_final_wfi)
	dsb	sy
	isb
	wfi
	isb
	ret
SYM_FUNC_END(apple_l4_final_wfi)
