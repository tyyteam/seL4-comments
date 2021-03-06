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
/*QT CONFIG_WORD_SIZE在build目录下指定为64，即字的大小64位。此处定义寄存器大小是8Bytes*/
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
  csrrw sp, sscratch, sp /*QT 交换sp和sscratch的值，sscratch通常是指针，指向当前任务的上下文。此操作将sp设置为当前上下文指针，sscratch为当前栈指针*/
/* Now we have a valid kernel stack */
  STORE t0, (-2*REGBYTES)(sp)/*QT t0寄存器中的值，保存到sp寄存器-2位置处，应该是上个上下文的faultIP*/
  LOAD  t0, (-1*REGBYTES)(sp)/*QT 将nextIP的值取出，存入t0寄存器中*/
#else
  csrrw t0, sscratch, t0/*QT 交换t0和sscratch的值，sscratch通常是指针，指向当前任务的上下文。此操作将t0设置为指向内存可用区域的指针*/
#endif
  STORE ra, (0*REGBYTES)(t0)/*QT 将各个寄存器存入t0指向的区域+偏移，ra是x1的别名，是返回地址寄存器*/
#ifndef ENABLE_SMP_SUPPORT
  STORE sp, (1*REGBYTES)(t0)
#endif
  STORE gp, (2*REGBYTES)(t0)/*QT 这些寄存器的定义位置在 /kernel/include/arch/riscv/arch/machine/registerset.h*/
  STORE tp, (3*REGBYTES)(t0)/*QT 这些寄存器基本上是seL4_UserContext结构体的内容，libsel4/arch_include/riscv/sel4/arch/types.h*/
  STORE t1, (5*REGBYTES)(t0)/*见官方riscv-CN手册28页的寄存器顺序*/
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
  csrr  x1, sscratch/*QT 读sscratch的值到x1中。sscratch的值原本是t0的值。x1寄存器，是ra寄存器的别名，之前已经保存*/
#endif
  STORE    x1, (4*REGBYTES)(t0)/*x1存入到t0的位置，此操作实现了原本t0的值的保存*/

  csrr x1, sstatus/*读sstatus到x1中*/
  STORE x1, (32*REGBYTES)(t0)/*保存控制状态寄存器sstatus到该位置*/

  csrr s0, scause
  STORE s0, (31*REGBYTES)(t0)/*保存异常来源寄存器scause到t6位置*/
/*QT gp全局指针寄存器 https://www.cnblogs.com/wahahahehehe/p/15140813.html*/
  la gp, __global_pointer$/*将__global_pointer$符号的地址加载到gp中*/

#ifdef ENABLE_SMP_SUPPORT
  /* save the user sp */
  csrr  x1, sscratch
  STORE x1, (1*REGBYTES)(t0)
  /* restore the sscratch */
  csrw  sscratch, sp
#else
  /* Load kernel's stack address */
  la sp, (kernel_stack_alloc + BIT(CONFIG_KERNEL_STACK_BITS))/*加载地址到sp*/
#endif

  /* Save exception PC */
  csrr x1,  sepc/*QT sepc：对于中断，它指向中断处理后应该返回的位置；对于异常，它指向导致异常的指令*/
  STORE   x1, (33*REGBYTES)(t0)

  /* Check if it's an interrupt */
  /*QT 上面将scause保存到了s0。异步中断，scause首位是1，即负数表示中断。首位是0，表示例外。见手册图10.3*/
  bltz s0, interrupt/*s0小于0，则跳转到下文的interrupt分支。*/

  /* ratified priv has value 8 for ecall from U-mode exception */
  /*QT 在RISCV中，通过ecall指令可以从低特权切换到高特权，在U模式下执行就切换到S模式，
  在S模式下调用就切换到M模式。用户模式的ecall调用的代码是8。*/
  li   s4, 8/*QT 立即数8加载到s4*/
  bne  s0, s4, exception/*QT s0不等于s4，跳转到exception。如果不是用户态主动切换到s态，跳转到exception*/

handle_syscall:/*QT 如果是用户态调用ecall的异常*/
  /* Set the return address to sepc + 4 in the case of a system/environment call */
  addi x1, x1, 4/*QT [x1]=[x1]+4。x1即ra，返回地址寄存器。*/
  /* Save NextIP */
  STORE   x1, (34*REGBYTES)(t0)/*将返回地址保存*/

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
  mv a2, a7/*QT a7是异常类型号*/

  j c_handle_syscall/*QT 处理系统调用，三个参数：a0=word_t cptr, a1=word_t msgInfo, a2=syscall_t syscall(系统调用号),a0也是返回值*/

/* Not an interrupt or a syscall */
exception:
  /* Save NextIP */
  STORE   x1, (34*REGBYTES)(t0)/*QT 根据上文sepc，保存异常指令的返回地址(=异常指令地址)。*/
  j c_handle_exception

interrupt:
  /* Save NextIP */
  STORE   x1, (34*REGBYTES)(t0)/*QT 根据上文sepc，保存中断指令的返回地址(=中断返回地址)。*/
  j c_handle_interrupt
//单核时，cpu寄存器t0中始终保留这sscratch的值，指向当前上下文的起始处。
//smp时，cpu中sscratch-2的位置存当前t0，sscratch-1的位置存当前上下文的指针，sscratch存可用区域？