/* swap_macros.h - helper macros for context switch */

/*
 * Copyright (c) 2014 Wind River Systems, Inc.
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

#ifndef _SWAP_MACROS__H_
#define _SWAP_MACROS__H_

#define tNANO_FIBER_OFFSET   0
#define tNANO_TASK_OFFSET    8
#define tNANO_CURRENT_OFFSET 16

#define _kernel_offset_to_irq_stack    8
#define _kernel_offset_to_current      16

#define _timeout_sizeof                48 
#define _thread_offset_to_prio         24
#define _thread_offset_to_sched_locked 32
#define _thread_offset_to_callee_saved 96  //_thread_base_sizeof+caller_saved

#define _callee_saved_offset_to_key        256  //(32*sizeof(uint64_t))
#define _callee_saved_offset_to_retval     264  //(33*sizeof(uint64_t))
#define _callee_saved_offset_to_preemptive 272  //(34*sizeof(uint64_t))

#define TCS_LINK_OFFSET         0
#define TCS_FLAGS_OFFSET        8
#define TCS_INTLOCK_OFFSET      16
#define TCS_COOP_REGS_OFFSET    24

/**
 * Saved by callee function registers:
 *      s0..s11, sp, tp
 */

/** Return address */
#define COOP_REG_RA         0//(0*sizeof(uint64_t))
/** Saved registers */
#define COOP_REG_S0         8//(1*sizeof(uint64_t))
#define COOP_REG_S1         16//(2*sizeof(uint64_t))
#define COOP_REG_S2         24//(3*sizeof(uint64_t))
#define COOP_REG_S3         32//(4*sizeof(uint64_t))
#define COOP_REG_S4         40//(5*sizeof(uint64_t))
#define COOP_REG_S5         48//(6*sizeof(uint64_t))
#define COOP_REG_S6         56//(7*sizeof(uint64_t))
#define COOP_REG_S7         64//(8*sizeof(uint64_t))
#define COOP_REG_S8         72//(9*sizeof(uint64_t))
#define COOP_REG_S9         80//(10*sizeof(uint64_t))
#define COOP_REG_S10        88//(11*sizeof(uint64_t))
#define COOP_REG_S11        96//(12*sizeof(uint64_t))
/** Stack pointer */
#define COOP_REG_SP         104//(13*sizeof(uint64_t))
/** Thread pointer */
#define COOP_REG_TP         112//(14*sizeof(uint64_t))
#define COOP_REG_MEPC       COOP_REG_TP//(14*sizeof(uint64_t))
/** Return values */
#define COOP_REG_V0         120//(15*sizeof(uint64_t))
#define COOP_REG_V1         128//(16*sizeof(uint64_t))
/** Function Arguments */
#define COOP_REG_A0         136//(17*sizeof(uint64_t))
#define COOP_REG_A1         144//(18*sizeof(uint64_t))
#define COOP_REG_A2         152//(19*sizeof(uint64_t))
#define COOP_REG_A3         160//(20*sizeof(uint64_t))
#define COOP_REG_A4         168//(21*sizeof(uint64_t))
#define COOP_REG_A5         176//(22*sizeof(uint64_t))
#define COOP_REG_A6         184//(23*sizeof(uint64_t))
#define COOP_REG_A7         192//(24*sizeof(uint64_t))
/** Temporary registers */
#define COOP_REG_T0         200//(25*sizeof(uint64_t))
#define COOP_REG_T1         208//(26*sizeof(uint64_t))
#define COOP_REG_T2         216//(27*sizeof(uint64_t))
#define COOP_REG_T3         224//(28*sizeof(uint64_t))
#define COOP_REG_T4         232//(29*sizeof(uint64_t))
/** Global pointer */
#define COOP_REG_GP         240//(30*sizeof(uint64_t))

#define COOP_REGS_TOTAL     32
#define COOP_STACKFRAME_SIZE (COOP_REGS_TOTAL*sizeof(uint64_t))


#define _save_context(TO) \
  sd ra, COOP_REG_RA(TO); \
  sd s0, COOP_REG_S0(TO); \
  sd s1, COOP_REG_S1(TO); \
  sd s2, COOP_REG_S2(TO); \
  sd s3, COOP_REG_S3(TO); \
  sd s4, COOP_REG_S4(TO); \
  sd s5, COOP_REG_S5(TO); \
  sd s6, COOP_REG_S6(TO); \
  sd s7, COOP_REG_S7(TO); \
  sd s8, COOP_REG_S8(TO); \
  sd s9, COOP_REG_S9(TO); \
  sd s10, COOP_REG_S10(TO); \
  sd s11, COOP_REG_S11(TO); \
  sd sp, COOP_REG_SP(TO); \
  sd x16, COOP_REG_V0(TO); \
  sd x17, COOP_REG_V1(TO); \
  sd a0, COOP_REG_A0(TO); \
  sd a1, COOP_REG_A1(TO); \
  sd a2, COOP_REG_A2(TO); \
  sd a3, COOP_REG_A3(TO); \
  sd a4, COOP_REG_A4(TO); \
  sd a5, COOP_REG_A5(TO); \
  sd a6, COOP_REG_A6(TO); \
  sd a7, COOP_REG_A7(TO); \
  sd t0, COOP_REG_T0(TO); \
  sd t1, COOP_REG_T1(TO); \
  sd t2, COOP_REG_T2(TO); \
  sd t3, COOP_REG_T3(TO); \
  sd t4, COOP_REG_T4(TO); \
  sd gp, COOP_REG_GP(TO);


#define _restore_context(FROM) \
  ld ra, COOP_REG_RA(FROM); \
  ld s0, COOP_REG_S0(FROM); \
  ld s1, COOP_REG_S1(FROM); \
  ld s2, COOP_REG_S2(FROM); \
  ld s3, COOP_REG_S3(FROM); \
  ld s4, COOP_REG_S4(FROM); \
  ld s5, COOP_REG_S5(FROM); \
  ld s6, COOP_REG_S6(FROM); \
  ld s7, COOP_REG_S7(FROM); \
  ld s8, COOP_REG_S8(FROM); \
  ld s9, COOP_REG_S9(FROM); \
  ld s10, COOP_REG_S10(FROM); \
  ld s11, COOP_REG_S11(FROM); \
  ld sp, COOP_REG_SP(FROM); \
  ld x16, COOP_REG_V0(FROM); \
  ld x17, COOP_REG_V1(FROM); \
  ld a0, COOP_REG_A0(FROM); \
  ld a1, COOP_REG_A1(FROM); \
  ld a2, COOP_REG_A2(FROM); \
  ld a3, COOP_REG_A3(FROM); \
  ld a4, COOP_REG_A4(FROM); \
  ld a5, COOP_REG_A5(FROM); \
  ld a6, COOP_REG_A6(FROM); \
  ld a7, COOP_REG_A7(FROM); \
  ld t0, COOP_REG_T0(FROM); \
  ld t1, COOP_REG_T1(FROM); \
  ld t2, COOP_REG_T2(FROM); \
  ld t3, COOP_REG_T3(FROM); \
  ld t4, COOP_REG_T4(FROM); \
  ld gp, COOP_REG_GP(FROM);


#endif /*  _SWAP_MACROS__H_ */
