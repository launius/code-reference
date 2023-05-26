# Copyright (C) 2023, Yunjae Lim <launius@gmail.com>
# Released under the MIT license (see COPYING.MIT for the terms)

SUMMARY = "I2C firmware update recipe"
DESCRIPTION = "Custom recipe to build i2cfw.c application"
LICENSE = "CLOSED"

PR = "r0"

SRC_URI = " \
	file://src \
	file://LT6911UXC.bin \
"

# Where to keep downloaded source files (in tmp/work/...)
S = "${WORKDIR}/src"

# Pass arguments to linker
TARGET_CC_ARCH += "${LDFLAGS}"

# Cross-compile source code
do_compile() {
	${CC} -Wall -O2 -o i2cflash i2cflash.c lt6911uxc.c
}

# Create /usr/bin in rootfs and copy program to it
do_install() {
	install -d ${D}${bindir}
	install -m 0755 i2cfw ${D}${bindir}
	install -m 0644 ${WORKDIR}/LT6911UXC.bin ${D}${bindir}
}
