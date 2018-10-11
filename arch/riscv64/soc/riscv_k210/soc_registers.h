/*
 * Copyright (c) 2016, GNSS Sensor Ltd.
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
 * @file SoC configuration macros for the Atmel SAM3 family processors.
 *
 * Refer to the datasheet for more information about these registers.
 */

#ifndef _RISCV_GNSS_SOC_REGS_H_
#define _RISCV_GNSS_SOC_REGS_H_

typedef struct MasterDescrWordBits {
    uint32_t descrsize : 8;
    uint32_t descrtype : 2;
    uint32_t rsrv : 14;
    uint32_t master_idx : 8;
} MasterDescrWordBits;

typedef union MasterDescrWord {
    MasterDescrWordBits bits;
    uint32_t val;
} MasterDescrWord;

typedef struct MasterConfigType {
    MasterDescrWord descr;
    uint16_t did;
    uint16_t vid;
} MasterConfigType;

typedef struct SlaveDescrWordBits {
    uint32_t descrsize : 8;
    uint32_t descrtype : 2;
    uint32_t bar_total : 2;
    uint32_t rsrv1 : 4;
    uint32_t irq_idx : 8;
    uint32_t rsrv2 : 8;
} SlaveDescrWordBits;

typedef union SlaveDescrWord {
    SlaveDescrWordBits bits;
    uint32_t val;
} SlaveDescrWord;

typedef struct SlaveConfigType {
    SlaveDescrWord descr;
    uint16_t did;
    uint16_t vid;
    uint32_t xmask;
    uint32_t xaddr;
} SlaveConfigType;

typedef struct pnp_map {
    volatile uint32_t hwid;         /// Read only HW ID
    volatile uint32_t fwid;         /// Read/Write Firmware ID
    volatile uint32_t tech;         /// Read only technology index
    volatile uint32_t rsrv1;        /// 
    volatile uint64_t idt;          /// 
    volatile uint64_t malloc_addr;  /// debuggind memalloc pointer
    volatile uint64_t malloc_size;  /// debugging memalloc size
    volatile uint64_t fwdbg1;       /// FW debug register
    volatile uint64_t rsrv[2];
    uint8_t cfg_table[(1 << 12) - 0x40];    /// 0xfffff040: RO: PNP configuration
} pnp_map;

typedef struct uart_map {
    volatile uint32_t status;
    volatile uint32_t scaler;
    uint32_t rsrv[2];
    volatile uint32_t data;
} uart_map;


typedef struct irqctrl_map {
    volatile uint32_t irq_mask;     // 0x00: [RW] 1=disable; 0=enable
    volatile uint32_t irq_pending;  // 0x04: [RW]
    volatile uint32_t irq_clear;    // 0x08: [WO]
    volatile uint32_t irq_rise;     // 0x0C: [WO]
    volatile uint64_t isr_table;    // 0x10: [RW]
    volatile uint64_t dbg_cause;    // 0x18: 
    volatile uint64_t dbg_epc;      // 0x20: 
    volatile uint32_t irq_lock;     // 0x28: interrupts wait while lock=1
    volatile uint32_t irq_cause_idx;// 0x2c: 
} irqctrl_map;


typedef struct gptimer_type {
    volatile uint32_t control;      // [0] = count_ena; [1] = irq_ena
    volatile uint32_t rsv1;
    volatile uint64_t cur_value;
    volatile uint64_t init_value;
} gptimer_type;

typedef struct gptimers_map {
    volatile uint64_t highcnt;
    volatile uint32_t pending;
    uint32_t rsv1[13];
    gptimer_type tmr[2];
} gptimer_map;

#endif /* _RISCV_GNSS_SOC_REGS_H_ */
