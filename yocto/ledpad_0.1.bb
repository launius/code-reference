# (c) 2022 Yunjae Lim <launius@gmail.com>

SUMMARY = "LED pad recipe"
DESCRIPTION = "Custom recipe to build ledpad.c application"
LICENSE = "CLOSED"

inherit update-rc.d

PR = "r0"

SRC_URI = " \
	file://ledpad.c \
	file://ledpad_service \
	"

# Where to keep downloaded source files (in tmp/work/...)
S = "${WORKDIR}"

# Pass arguments to linker
TARGET_CC_ARCH += "${LDFLAGS}"

# Cross-compile source code
do_compile() {
	${CC} -Wall -O2 -o ledpad ledpad.c
}

# Create /usr/bin in rootfs and copy program to it
do_install() {
	install -d ${D}${bindir}
	install -m 0755 ledpad ${D}${bindir}

	install -d ${D}${sysconfdir}/init.d
	install -m 0755 ledpad_service ${D}${sysconfdir}/init.d
}

INITSCRIPT_NAME = "ledpad_service"
INITSCRIPT_PACKAGES = "${PN}"
RDEPENDS_${PN} = "initscripts"
