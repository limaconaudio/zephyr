/*
 * Copyright (c) 2010-2015 Wind River Systems, Inc.
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

/**
 * @file
 * @brief Nanokernel thread support primitives
 *
 * This module provides core nanokernel fiber related primitives for the IA-32
 * processor architecture.
 */


#include <kernel.h>
#include <nano_internal.h>
#include <kernel_structs.h>
#include <wait_q.h>
#include <string.h>
#include <misc/printk.h>
#include "swap_macros.h"

#ifdef _WIN32
extern int LIBH_create_thread(char *pStackMem,
                              unsigned stackSize,
                              int priority,
                              unsigned options);
#endif

#if defined(CONFIG_THREAD_MONITOR)
/*
 * Add a thread to the kernel's list of active threads.
 */
static ALWAYS_INLINE void thread_monitor_init(struct k_thread *thread)
{
	unsigned int key;

	key = irq_lock();
	thread->next_thread = _kernel.threads;
	_kernel.threads = thread;
	irq_unlock(key);
}
#else
#define thread_monitor_init(thread) \
	do {/* do nothing */     \
	} while ((0))
#endif /* CONFIG_THREAD_MONITOR */


struct init_stack_frame {
	/* top of the stack / most recently pushed */

	/* Used by _thread_entry_wrapper. pulls these off the stack and
	 * into argument registers before calling _thread_entry()
	 */
	_thread_entry_t entry_point;
	void *arg1;
	void *arg2;
	void *arg3;

	/* least recently pushed */
};


/**
 *
 * @brief Create a new kernel execution thread
 *
 * This function is utilized to create execution threads for both fiber
 * threads and kernel tasks.
 */
extern void _new_thread(char *pStack, size_t stackSize,
			void (*pEntry)(void *, void *, void *),
			void *p1, void *p2, void *p3,
			int prio, unsigned options)
{


	_ASSERT_VALID_PRIO(priority, thread_func);

	struct k_thread *thread;
	unsigned long *pInitialCtx;

#ifdef CONFIG_INIT_STACKS
	memset(pStack, 0xaa, stackSize);
#endif

	/* Initialize various struct k_thread members */
	thread = (struct k_thread *)pStack;
	pInitialCtx =
		(unsigned long *)(pStack + stackSize);


	_init_thread_base(&thread->base, prio, K_PRESTART, options);

	/* static threads overwrite it afterwards with real value */
	thread->init_data = NULL;
	thread->fn_abort = NULL;

#ifdef CONFIG_THREAD_CUSTOM_DATA
	/* Initialize custom data field (value is opaque to kernel) */
	thread->custom_data = NULL;
#endif
	thread->callee_saved.key = 0;
	thread->callee_saved.preemptive = 0;
	/* Leave the rest of thread->callee_saved junk */

	thread_monitor_init(thread);


    //printk("\nInitial context SP = 0x%x\n", (unsigned long)pStack);
    thread->callee_saved.r[COOP_REG_RA/sizeof(uint64_t)] = (uint64_t)_thread_entry;
    thread->callee_saved.r[COOP_REG_MEPC/sizeof(uint64_t)] = (uint64_t)_thread_entry;
    thread->callee_saved.r[COOP_REG_A0/sizeof(uint64_t)] = (uint64_t)pEntry;
    thread->callee_saved.r[COOP_REG_A1/sizeof(uint64_t)] = (uint64_t)p1;
    thread->callee_saved.r[COOP_REG_A2/sizeof(uint64_t)] = (uint64_t)p2;
    thread->callee_saved.r[COOP_REG_A3/sizeof(uint64_t)] = (uint64_t)p3;
    thread->callee_saved.r[COOP_REG_SP/sizeof(uint64_t)] =
        (uint64_t)(pStack + stackSize);
#ifdef _WIN32
    LIBH_create_thread(pStack, (unsigned int)stackSize, prio, options);
#endif
}
