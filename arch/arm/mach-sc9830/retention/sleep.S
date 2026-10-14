/* SPDX-License-Identifier: GPL-2.0-only */

	.arm
	.section .rodata.sc9830_retention,"a"
	.balign 4
	.global sc9830_retention_start, sc9830_retention_end
	.global sc9830_retention_reset, sc9830_retention_resume
	.global sc9830_retention_status, sc9830_retention_limit
sc9830_retention_start:
	cpsid aif
	mov r4, #0x10
	bl sc9830_retention_record
	ldr r0, sc9830_retention_access_set
	mov r1, #1
	str r1, [r0]
	dsb sy
	mrc p15, 0, r0, c1, c0, 0
	orr r0, r0, #0x1000
	mcr p15, 0, r0, c1, c0, 0
	isb
	bl sc9830_retention_ready
	dsb sy
	isb
	mov r1, #0
5:
	cmp r1, #1
	wfieq
	movne r2, #1
	moveq r2, #0x30000
6:
	subs r2, r2, #1
	bne 6b
	add r1, r1, #1
	cmp r1, #1
	beq 5b
	dsb sy
	isb
	mov r4, #1
	b sc9830_retention_return
sc9830_retention_reset:
	cpsid aif
	mov r4, #2
	bl sc9830_retention_delay
sc9830_retention_return:
	bl sc9830_retention_ready
	ldr r0, sc9830_retention_access_clear
	mov r1, #1
	str r1, [r0]
	dsb sy
	mrc p15, 0, r0, c1, c0, 0
	bic r0, r0, #0x1000
	mcr p15, 0, r0, c1, c0, 0
	isb
	bl sc9830_retention_record
	ldr r0, sc9830_retention_resume
	bx r0
sc9830_retention_ready:
	ldr r0, sc9830_retention_pmu
	ldr r1, sc9830_retention_phy
	ldr r2, sc9830_retention_limit
1:
	ldr r3, [r0]
	ldr r5, [r0]
	orr r3, r3, r5
	tst r3, #0x0f000000
	bne 2f
	ldr r3, [r1]
	tst r3, #1
	bxne lr
2:
	subs r2, r2, #1
	bne 1b
	orr r4, r4, #0x80
	bl sc9830_retention_record
3:
	wfi
	b 3b
sc9830_retention_delay:
	mov r2, #0x100
4:
	subs r2, r2, #1
	bne 4b
	bx lr
sc9830_retention_record:
	adr r0, sc9830_retention_status
	str r4, [r0]
	dsb sy
	bx lr
sc9830_retention_access_set:
	.word 0x30021064
sc9830_retention_access_clear:
	.word 0x30022064
sc9830_retention_pmu:
	.word 0x402b00c4
sc9830_retention_phy:
	.word 0x30010184
sc9830_retention_resume:
	.word 0
sc9830_retention_limit:
	.word 0x100000
sc9830_retention_status:
	.word 0
sc9830_retention_end:
