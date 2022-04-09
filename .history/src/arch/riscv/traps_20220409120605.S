/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 * Copyright 2015, 2016 Hesham Almatary <heshamelmatary@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <config.h>/*QT 路径是kernel/include/config.h 宏定义一些及其特性*/
#include <machine/assembler.h>/*QT 路径是kernel/include/machine/assembler.h 段相关的汇编宏*/
#include <arch/machine/hardware.h>/*QTkernel/include/arch/riscv/arch/machine/hardware.h 内存管理（MMU）RISCV平台相关的一些宏定义和操作*/
#include <arch/api/syscall.h>/*QT build下生成*/
#include <arch/machine/registerset.h>/*QT /kernel/include/arch/riscv/arch/machine/registerset.h 寄存器宏定义，消息相关宏定义，fpu等*/
#include <util.h>/*QT kernel/include/util.h 目前用到的是一些位运算操作的宏定义，属性函数等相关宏定义*/
/*QT CONFIG_WORD_SIZE在build目录下指定为64，即字的大小64位，8Bytes*/
#define REGBYTES (CONFIG_WORD_SIZE / 8)

.section .text/*QT 位于.text段*/

.global trap_entry
.extern c_handle_syscall
.extern c_handle_fastpath_reply_recv
.extern c_handle_fastpath_call
.extern c_handle_interrupt
.extern c_handle_exception
/*QT 这些extern函数均来自 include/arch/riscv/arch/kernel/traps.h src/arch/riscv/c_traps.c*/
trap_entry:

#ifdef ENABLE_SMP_SUPPORT
/* The sscratch contains the stack for the current core */
  csrrw sp, sscratch, sp /*QT 复制sscratch值到sp，再把sp的值复制到sscratch，为什么？*/
/* Now we have a valid kernel stack */
  STORE t0, (-2*REGBYTES)(sp)/*QT t0寄存器中存放的值，保存到sp寄存器值加上立即数imm值相加结果的内存地址的 内存单元*/
  LOAD  t0, (-1*REGBYTES)(sp)/*QT 将sp寄存器值加上立即数imm值相加结果的内存地址的 内存单元 的值，存入t0寄存器中*/
#else
  csrrw t0, sscratch, t0/*QT 用户线程运行期间，sscratch保留着特权模式线程上下文的指针，在trap处理时，它与用户寄存器交换内容*/
#endif
  STORE ra, (0*REGBYTES)(t0)
#ifndef ENABLE_SMP_SUPPORT
  STORE sp, (1*REGBYTES)(t0)
#endif
  STORE gp, (2*REGBYTES)(t0)
  STORE tp, (3*REGBYTES)(t0)
  STORE t1, (5*REGBYTES)(t0)
  STORE t2, (6*REGBYTES)(t0)
  STORE s0, (7*REGBYTES)(t0)
  STORE s1, (8*REGBYTES)(t0)
  STORE a0, (9*REGBYTES)(t0)
  STORE a1, (10*REGBYTES)(t0)
  STORE a2, (11*REGBYTES)(t0)
  STORE a3, (12*REGBYTES)(t0)
  STORE a4, (13*REGBYTES)(t0)
  STORE a5, (14*REGBYTES)(t0)
  STORE a6, (15*REGBYTES)(t0)
  STORE a7, (16*REGBYTES)(t0)
  STORE s2, (17*REGBYTES)(t0)
  STORE s3, (18*REGBYTES)(t0)
  STORE s4, (19*REGBYTES)(t0)
  STORE s5, (20*REGBYTES)(t0)
  STORE s6, (21*REGBYTES)(t0)
  STORE s7, (22*REGBYTES)(t0)
  STORE s8, (23*REGBYTES)(t0)
  STORE s9, (24*REGBYTES)(t0)
  STORE s10, (25*REGBYTES)(t0)
  STORE s11, (26*REGBYTES)(t0)
  STORE t3, (27*REGBYTES)(t0)
  STORE t4, (28*REGBYTES)(t0)
  STORE t5, (29*REGBYTES)(t0)
  STORE t6, (30*REGBYTES)(t0)
  /* save t0 value */
#ifdef ENABLE_SMP_SUPPORT
  LOAD  x1, (-2*REGBYTES)(sp)
#else
  csrr  x1, sscratch
#endif
  STORE    x1, (4*REGBYTES)(t0)

  csrr x1, sstatus
  STORE x1, (32*REGBYTES)(t0)

  csrr s0, scause
  STORE s0, (31*REGBYTES)(t0)

  la gp, __global_pointer$

#ifdef ENABLE_SMP_SUPPORT
  /* save the user sp */
  csrr  x1, sscratch
  STORE x1, (1*REGBYTES)(t0)
  /* restore the sscratch */
  csrw  sscratch, sp
#else
  /* Load kernel's stack address */
  la sp, (kernel_stack_alloc + BIT(CONFIG_KERNEL_STACK_BITS))
#endif

  /* Save exception PC */
  csrr x1,  sepc
  STORE   x1, (33*REGBYTES)(t0)

  /* Check if it's an interrupt */
  bltz s0, interrupt

  /* ratified priv has value 8 for ecall from U-mode exception */
  li   s4, 8
  bne  s0, s4, exception

handle_syscall:
  /* Set the return address to sepc + 4 in the case of a system/environment call */
  addi x1, x1, 4
  /* Save NextIP */
  STORE   x1, (34*REGBYTES)(t0)

#ifdef CONFIG_FASTPATH
  li t3, SYSCALL_CALL
  beq a7, t3, c_handle_fastpath_call

  li t3, SYSCALL_REPLY_RECV
#ifdef CONFIG_KERNEL_MCS
  /* move reply to 3rd argument */
  mv a2, a6
#endif
  beq a7, t3, c_handle_fastpath_reply_recv
#endif

  /* move syscall number to 3rd argument */
  mv a2, a7

  j c_handle_syscall

/* Not an interrupt or a syscall */
exception:
  /* Save NextIP */
  STORE   x1, (34*REGBYTES)(t0)
  j c_handle_exception

interrupt:
  /* Save NextIP */
  STORE   x1, (34*REGBYTES)(t0)
  j c_handle_interrupt
