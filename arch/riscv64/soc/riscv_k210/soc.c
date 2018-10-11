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
 * @file
 * @brief System/hardware module for GNSS RISC-V family processor
 *
 * This module provides routines to initialize and support board-level hardware
 * for the GNSS RISC-V family processor.
 */

#include <stdint.h>
#include <errno.h>
#include <nanokernel.h>
#include <device.h>
#include <init.h>
#include <soc.h>
#include <misc/printk.h>
#include <misc/shell.h>
#include <sw_isr_table.h>
#include <memaccess.h>

#define SHELL_SOC "soc"

extern void _IsrWrapper(int idx, void *arg);

typedef union DescriptorTableType {
    union DescriptorItemType {
        MasterConfigType mst;
        SlaveConfigType slv;
    } *item;
    uint8_t *buf;
} DescriptorTableType;

const char *const VENDOR_NAME = "GNSS Sensor Ltd.";

const char *const MST_DID_EMPTY_NAME = "Empty master slot";
const char *const SLV_DID_EMPTY_NAME = "Empty slave slot";

const char *const UNKOWN_ID_NAME = "Unknown";

static const char *const GNSS_SENSOR_MST_DEVICE_NAMES[] = {
    "Rocket Cached TileLink",           // 0x500
    "Rocket Uncached TileLink",         // 0x501
    "Gaisler Ethernet MAC with DMA",    // 0x502
    "Gaisler Ethernet EDCL with DMA",   // 0x503
    "Reserved",                         // 0x504
    "RISC-V River CPU",                 // 0x505
};

static const char *const GNSS_SENSOR_SLV_DEVICE_NAMES[] = {
    "GNSS Engine stub",         // 0x68
    "Reserved",                 // 0x69
    "Reserved",                 // 0x6a
    "Reserved",                 // 0x6b
    "Reserved",                 // 0x6c
    "Reserved",                 // 0x6d
    "Reserved",                 // 0x6e
    "Reserved",                 // 0x6f
    "Reserved",                 // 0x70
    "Boot ROM",                 // 0x71
    "FW Image ROM",             // 0x72
    "Internal SRAM",            // 0x73
    "Plug'n'Play support",      // 0x74
    "SD Controller",            // 0x75
    "Generic GPIO",             // 0x76
    "RF front-end controller",  // 0x77
    "GNSS Engine",              // 0x78
    "GPS FSE",                  // 0x79
    "Generic UART",             // 0x7a
    "Accelerometer",            // 0x7b
    "Gyroscope",                // 0x7c
    "Interrupt Controller",     // 0x7d
    "Reserved",                 // 0x7e
    "Ethernet MAC",             // 0x7f
    "Debug Support Unit (DSU)", // 0x80
    "GP Timers"                 // 0x81
};

/**
 * @brief Get technology name
 */
const char *const get_tech_name(uint32_t tech)
{
    switch (tech) {
    case TECH_INFERRED: return "inferred";
    case TECH_VIRTEX6: return "Virtex6";
    case TECH_KINTEX7: return "Kintex7";
    default:;
    }
    return "unknown";
}

/**
 * @brief Get device Vendor name by its ID
 */
static const char *get_vendor_name(uint16_t vid)
{
    if (vid != VENDOR_GNSSSENSOR) {
        return UNKOWN_ID_NAME;
    }
    return VENDOR_NAME;
}

/**
 * @brief Get device Name by Vendor ID and Device ID
 */
static const char *get_device_name(uint16_t vid, uint16_t did)
{
    if (vid != VENDOR_GNSSSENSOR) {
        return UNKOWN_ID_NAME;
    }
    if (did == MST_DID_EMPTY) {
        return MST_DID_EMPTY_NAME;
    }
    if (did == SLV_DID_EMPTY) {
        return SLV_DID_EMPTY_NAME;
    }
    if (did >= GNSSSENSOR_ENGINE_STUB && did <= GNSSSENSOR_GPTIMERS) {
        return GNSS_SENSOR_SLV_DEVICE_NAMES[did - GNSSSENSOR_ENGINE_STUB];
    }
    if (did >= RISCV_CACHED_TILELINK && did <= RISCV_RIVER_CPU) {
        return GNSS_SENSOR_MST_DEVICE_NAMES[did - RISCV_CACHED_TILELINK];
    }
    return UNKOWN_ID_NAME;
}

/**
 * @brief Print Plug'n'Play information
 *
 * This function reads information from the PNP slave device that is mapped
 * into hardcoded address __PNP (0xFFFFF000).
 */
static int shell_cmd_soc_info(int argc, char *argv[])
{
    printk("RISC-V synthesizable SoC platform available for download at:\n");
    printk("    https://github.com/sergeykhbr/riscv_vhdl\n\n");
    printk("Author: Sergey Khabarov - sergeykhbr@gmail.com\n");
    return 0;
}

/**
 * @brief Print Plug'n'Play information
 *
 * This function reads information from the PNP slave device that is mapped
 * into hardcoded address __PNP (0xFFFFF000).
 */
static int shell_cmd_soc_pnp(int argc, char *argv[])
{
    uint32_t slaves_total, tech, hwid;
    uint16_t vid, did;
    adr_type xaddr, xmask, xsize;
    volatile uint32_t *iter;
    MasterDescrWord mcfg;
    int mst_cnt=0, slv_cnt=0;

    tech = READ32(&__PNP->tech);
    slaves_total = (tech >> 8) & 0xff;
    printk("# RISC-V:  Rocket-Chip demonstration design\n");
    hwid = READ32(&__PNP->hwid);
    printk("# HW id:   0x%x\n", hwid);
    hwid = READ32(&__PNP->fwid);
    printk("# FW id:   0x%x\n", hwid);
    printk("# Target technology: %s\n", get_tech_name(tech & 0xFF));

    iter = (volatile uint32_t *)__PNP->cfg_table;
    mcfg.val = READ32(iter);
    while (mcfg.bits.descrtype != PNP_CFG_TYPE_INVALID) {
        if (mcfg.bits.descrtype == PNP_CFG_TYPE_MASTER) {
            MasterConfigType *pmst = (MasterConfigType *)iter;
            vid = READ16(&pmst->vid);
            did = READ16(&pmst->did);
            printk("# AXI4: mst%d: %s    %s\n", mst_cnt++, 
                get_vendor_name(vid), get_device_name(vid, did));
        } else {
            SlaveConfigType *pslv = (SlaveConfigType *)iter;
            vid = READ16(&pslv->vid);
            did = READ16(&pslv->did);
            printk("# AXI4: slv%d: %s    %s\n", slv_cnt++, 
                get_vendor_name(vid), get_device_name(vid, did));

            xaddr = READ32(&pslv->xaddr);
            xmask = READ32(&pslv->xmask);
            xmask ^= 0xFFFFFFFF;
            xsize = xmask + 1;


            printk("#    %x...%x, size = ",
                (unsigned long)xaddr, (unsigned long)(xaddr + xmask));
            if (xsize < 1024) {
                printk("%d bytes\n", (int)xsize);
            } else if (xsize < 1024*1024) {
                printk("%d KB\n", (int)(xsize >> 10));
            } else {
                printk("%d MB\n", (int)(xsize >> 20));
            }
        }

        iter = (volatile uint32_t *)((uint8_t *)iter + mcfg.bits.descrsize);
        mcfg.val = READ32(iter);
    }
    return 0;
}

/**
 * @brief Run Dhrystone 2.1 benchmark
 */
extern int shell_cmd_soc_dhry(int argc, char *argv[]);

/**
 * @brief Check hardware target configuration is it inferred or not.
 *
 * inferred hardware target is used for RTL simulation of the whole SOC design.
 */
uint32_t soc_is_rtl_simulation()
{
    uint32_t tech = READ32(&__PNP->tech);
    return (tech & 0xff) == TECH_INFERRED ? 1: 0;
}

/**
 *
 * @brief perform basic hardware initialization
 *
 * Hardware initialized:
 * - interrupt unit
 *
 * RETURNS: N/A
 */
static int riscv_gnss_soc_init(struct device *arg)
{
    ARG_UNUSED(arg);
    WRITE64(&__IRQCTRL->isr_table, (uint64_t)_IsrWrapper);
    WRITE32(&__IRQCTRL->irq_lock, 0);
    return 0;
}

struct shell_cmd soc_commands[] = {
    { "info", shell_cmd_soc_info, "Show SoC RTL download link" },
    { "pnp", shell_cmd_soc_pnp, "Show Plug'n'Play information" },
    { "dhry", shell_cmd_soc_dhry, "Run Dhrystone 2.1. benchmark" },
    { NULL, NULL }
};

SYS_INIT(riscv_gnss_soc_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

SHELL_REGISTER(SHELL_SOC, soc_commands);
