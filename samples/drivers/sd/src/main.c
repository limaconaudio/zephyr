/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <stdio.h>
#include <fs.h>
#include <disk_access.h>

void main(void)
{
	disk_access_init("SDHC");	
}
