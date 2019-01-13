#include "sdcard.h"
#include <stdio.h>
#include <zephyr.h>
#include <misc/printk.h>

static int sdcard_test(void);

int main(void)
{
    if(sdcard_test())
    {
        printk("SD card err\n");
        return -1;
    }

    return 0;
}

static int sdcard_test(void)
{
    uint8_t status;

    printk("/******************sdcard test*****************/\n");
    status = sd_init();
    printk("sd init %d\n", status);
    if (status != 0)
    {
        return status;
    }

    printk("card info status %d\n", status);
    printk("CardCapacity:%ld\n", cardinfo.CardCapacity);
    printk("CardBlockSize:%d\n", cardinfo.CardBlockSize);
    return 0;
}
