# (c) 2023 Yunjae Lim <launius@gmail.com>

SUMMARY = "A package group to support audio functionality"
PR = "r1"

#
# packages which content depend on MACHINE_FEATURES need to be MACHINE_ARCH
#
PACKAGE_ARCH = "${MACHINE_ARCH}"

inherit packagegroup

RDEPENDS:${PN} = " \
	alsa-utils \
	alsa-state \
	jack-server \
	jack-utils \
	pulseaudio-module-dbus-protocol \
	pulseaudio \
	pulseaudio-server \
	pulseaudio-misc \
"
