// SPDX-License-Identifier: GPL-2.0
// (c) 2022 Yunjae Lim <launius@gmail.com> 

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#define LEDPAD_ERROR            1
#define LEDPAD_DEBUG            1
#define LEDPAD_INFO             0

#define print_error(fmt, ...) \
        do { if (LEDPAD_ERROR) fprintf(stderr, "%s: " fmt, \
                                __func__, ##__VA_ARGS__); } while (0)

#define print_debug(fmt, ...) \
        do { if (LEDPAD_DEBUG) fprintf(stderr, "%s: " fmt, \
                                __func__, ##__VA_ARGS__); } while (0)

#define print_info(fmt, ...) \
        do { if (LEDPAD_INFO) fprintf(stderr, "%s: " fmt, \
                                __func__, ##__VA_ARGS__); } while (0)

#define DEV_MISC_SPI			"/dev/misc_spi_ledpad"
