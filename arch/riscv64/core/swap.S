/*
 * Copyright (c) 2014-2015 Wind River Systems, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define _ASMLANGUAGE

#include "swap_macros.h"

/**
 *
 * @brief Initiate a cooperative context switch
 *
 * The _Swap() routine is invoked by various nanokernel services to effect
 * a cooperative context switch.  Prior to invoking _Swap(), the caller
 * disables interrupts via nanoCpuIntLock() and the return 'key' is passed as a
 * parameter to _Swap(). The key is in fact the value stored in the register
 * operand of a CLRI instruction.
 *
 * It stores the intlock key parameter into current->intlock_key.

 * Given that _Swap() is called to effect a cooperative context switch,
 * the caller-saved integer registers are saved on the stack by the function
 * call preamble to _Swap(). This creates a custom stack frame that will be
 * popped when returning from _Swap(), but is not suitable for handling a return
 * from an exception. Thus, the fact that the thread is pending because of a
 * cooperative call to _Swap() has to be recorded via the _CAUSE_COOP code in
 * the relinquish_cause of the thread's tTCS. The _IrqExit()/_FirqExit() code
 * will take care of doing the right thing to restore the thread status.
 *
 * When _Swap() is invoked, we know the decision to perform a context switch or
 * not has already been taken and a context switch must happen.
 *
 * @return may contain a return value setup by a call to fiberRtnValueSet()
 *
 * C function prototype:
 *
 * unsigned int _Swap (unsigned int key);
 *
 */

.section ".text"
.globl __start
__start:
	jal _Cstart;

.section ".text"
.globl _Swap
_Swap:
	_save_context(tp)
	la s0, _kernel
	ld s1,_kernel_offset_to_current(s0)     	# s1 = _kernel.current
	addi s2,s1,_thread_offset_to_callee_saved 	# s2 = &next_thread->callee_saved
	sd a0,_callee_saved_offset_to_key(s2)        	# _kernel.current->callee_saved.key = fl

        ## Populate default return value
	lw a1,_k_neg_eagain
	sd a1, _callee_saved_offset_to_retval(s2)

	## This thread was switched preemptively
	li a1,0
	sd a1,_callee_saved_offset_to_preemptive(s2)

	jal _get_next_ready_thread
	mv s1,a0
	sd s1,_kernel_offset_to_current(s0)     	# _kernel.current = _get_next_ready_thread()
	addi tp,s1,_thread_offset_to_callee_saved 	# tp = &next_thread->callee_saved

	ld a0,_callee_saved_offset_to_key(tp)        	# a0 = callee_saved.key
        jal _arch_irq_unlock                        	# _arch_irq_unlock(callee_saved.key)

	_restore_context(tp)
	ld t4,_callee_saved_offset_to_preemptive(tp)
	bnez t4,__swap_preemptive
__swap_cooperative:
	ret
__swap_preemptive:
	ld t4,COOP_REG_MEPC(tp)
	jalr t4

##
#  _IsrWrapper(int idx, void *)
#       a0 store IRQ index,
#       a1 equals NULL
#
.globl _IsrWrapper
_IsrWrapper:
        addi sp,sp,-32
        sd ra,0(sp)
        sd s0,8(sp)
        sd s1,16(sp)
        sd s2,24(sp)

	# Grab a reference to _kernel in r10 so we can determine the
	# current irq stack pointer
	#
	la a1,_kernel

	# Stash a copy of thread's sp in r12 so that we can put it on the IRQ
	# stack
	#
	mv a2,sp

	# Switch to interrupt stack
	ld sp,_kernel_offset_to_irq_stack(a1)

	# Store thread stack pointer onto IRQ stack
	addi sp,sp,-8
	sd a2,0(sp)

	call run_isr_handler

	ld sp,0(sp)

	# Check reschedule condition 
	jal _get_next_ready_thread
	beqz a0,_IsrExit_ending         # _get_next_ready_thread() == 0 goto ending

	## Do not reschedule coop threads (threads that have negative prio)
	ld s1, _thread_offset_to_prio(a0)
	blt  s1,zero,_IsrExit_ending

	## Do not reschedule if scheduler is locked
	ld s1, _thread_offset_to_sched_locked(a0)
	bne s1,zero,_IsrExit_ending

	## Call into the kernel to see if a scheduling decision is necessary
	mv s1,a0				# s1 = next_thread
	jal _is_next_thread_current
	bne a0,zero,_IsrExit_ending

	## Flag current thread that it was switched preemptively
	la s0, _kernel
	ld s0,_kernel_offset_to_current(s0)     	# s0 = _kernel.current
	addi s2,s0,_thread_offset_to_callee_saved 	# s2 = &next_thread->callee_saved
	li a0,1
	sd a0,_callee_saved_offset_to_preemptive(s2)

	# Store IRQ key
        jal _arch_irq_lock_state        	# a0 = _arch_irq_lock_state()
        sd a0,_callee_saved_offset_to_key(s2)

	##
	# Switch to the new thread.
	#
	addi tp,s1,_thread_offset_to_callee_saved 	# tp = &next_thread->callee_saved

	ld a0,_callee_saved_offset_to_key(tp)        	# a0 = callee_saved.key
        jal _arch_irq_unlock                        	# _arch_irq_unlock(callee_saved.key)

	ld t4,_callee_saved_offset_to_preemptive(tp)
	bnez t4,_IsrExit_ending

	## Next thread was switched cooperative so,set MEPC to ra
	ld a0,COOP_REG_RA(tp)
	sd a0,COOP_REG_MEPC(tp)


_IsrExit_ending:
        ld s2,24(sp)
        ld s1,16(sp)
        ld s0,8(sp)
        ld ra,0(sp)
        addi sp,sp,32
	ret
