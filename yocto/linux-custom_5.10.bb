# Copyright (C) 2023 Yunjae Lim <launius@gmail.com>
# Released under the MIT license (see COPYING.MIT for the terms)

require recipes-kernel/linux/linux-yocto.inc
require linux-chipvendor.inc

inherit freeze-rev local-git

FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI = " \
	git://${TOPDIR}/../tmp/linux-dev;protocol=file;usehead=1 \
	file://sound.cfg \
"

SRCREV = "${AUTOREV}"
KBRANCH = "HEAD"

LIC_FILES_CHKSUM = "file://COPYING;md5=6bc538ed5bd9a7fc9398086aedcd7e46"

KERNEL_VERSION_SANITY_SKIP = "1"
LINUX_VERSION ?= "5.10"
