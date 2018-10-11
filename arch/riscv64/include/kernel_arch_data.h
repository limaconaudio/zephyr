/*
 * Copyright (c) 2016 Gnss Sensor Ltd
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

#ifndef __KERNEL_ARCH__DATA_H__
#define __KERNEL_ARCH__DATA_H__

#include <stdint.h>
#include <toolchain.h>
#include <sections.h>
#include <arch/cpu.h>

#ifndef _ASMLANGUAGE
#include <kernel.h>
#include <nano_internal.h>
#include <stdint.h>
#include <misc/util.h>
#include <misc/dlist.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

struct _caller_saved {
	/*
	 * Nothing here, the exception code puts all the caller-saved
	 * registers onto the stack.
	 */
    int empty_;
};

typedef struct _caller_saved _caller_saved_t;

struct _callee_saved {
    uint64_t r[32];
	/* IRQ status before irq_lock() and call to _Swap() */
	uint64_t key;

	/* Return value of _Swap() */
	uint64_t retval;

    /* I actually do not understand where preemptive tasks switch occurs,
     * so use this flag stores the latest method of switch 0=coop; 1=preemp.
     * It allows properly switch cooperative/preemtive threads independently
     * of its priority
     */
    uint64_t preemptive;
};

typedef struct _callee_saved _callee_saved_t;

struct _thread_arch {
	/* nothing for now */
    unsigned int empty_;
};

typedef struct _thread_arch _thread_arch_t;

struct _kernel_arch {
	/* nothing for now */
    int empty_;
};

typedef struct _kernel_arch _kernel_arch_t;


#ifdef __cplusplus
}
#endif

#endif /* __KERNEL_ARCH__DATA_H__ */
