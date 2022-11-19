/*
 * sdCard.h
 *
 *  Created on: Nov. 18, 2022
 *      Author: Rene
 */

#ifndef SDCARD_H_
#define SDCARD_H_

#include <file.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>   //usleep()

#include <ti/drivers/SD.h>

#include "ti/drivers/SDFatFS.h"
#include "third_party/fatfs/ff.h"
#include <third_party/fatfs/ffcio.h>
#include <third_party/fatfs/diskio.h>
/* Driver configuration */
#include "ti_drivers_config.h"


/* Buffer size used for the file copy process */
#ifndef CPY_BUFF_SIZE
#define CPY_BUFF_SIZE       2048
#endif

/* String conversion macro */
#define STR_(n)             #n
#define STR(n)              STR_(n)

/* Drive number used for FatFs */
#define DRIVE_NUM           0

FATFS fs;
FIL fil;
FRESULT fresult;
char buffer[1024];
UINT br, bw;
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;




int32_t fatfs_getFatTime(void);

#endif /* SDCARD_H_ */
