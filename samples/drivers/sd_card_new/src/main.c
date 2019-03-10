#include "sdcard.h"
#include "ff.h"
#include <stdio.h>
#include <zephyr.h>
#include <misc/printk.h>

static int sdcard_test(void);
static int fs_test(void);

int main(void)
{
    if(sdcard_test())
    {
        printk("SD card err\n");
        return -1;
    }

    if(fs_test())
    {
        printk("FAT32 err\n");
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
    printk("CardCapacity:%llu\n", cardinfo.CardCapacity);
    printk("CardBlockSize:%d\n", cardinfo.CardBlockSize);
    return 0;
}

struct device *dma_dev;

static int fs_test(void)
{
    static FATFS sdcard_fs;
    FRESULT status;
    DIR dj;
    FILINFO fno;

    printk("/********************fs test*******************/\n");
    status = f_mount(&sdcard_fs, _T("0:"), 1);
    printk("mount sdcard:%d\n", status);
    if (status != FR_OK)
        return status;

    status = f_findfirst(&dj, &fno, _T("0:"), _T("*"));
    while (status == FR_OK && fno.fname[0]) {
        if (fno.fattrib & AM_DIR)
            printk("dir:%s\n", fno.fname);
        else
            printk("file:%s\n", fno.fname);
        status = f_findnext(&dj, &fno);
    }
    f_closedir(&dj);
    return 0;
}

