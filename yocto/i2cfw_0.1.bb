# Copyright (C) 2023, launius@gmail.com
# Released under the MIT license (see COPYING.MIT for the terms)

SUMMARY = "I2C firmware update recipe"
DESCRIPTION = "Custom recipe to build i2cfw.c application"
LICENSE = "CLOSED"

PR = "r0"

SRC_URI = " \
	file://i2cfw.c \
	file://Firmware.bin \
"

# Where to keep downloaded source files (in tmp/work/...)
S = "${WORKDIR}"

# Pass arguments to linker
TARGET_CC_ARCH += "${LDFLAGS}"

# Cross-compile source code
do_compile() {
	${CC} -Wall -O2 -o i2cfw i2cfw.c
}

# Create /usr/bin in rootfs and copy program to it
do_install() {
	install -d ${D}${bindir}
	install -m 0755 i2cfw ${D}${bindir}
	install -m 0644 Firmware.bin ${D}${bindir}
}
