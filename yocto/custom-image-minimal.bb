# (c) 2023 Yunjae Lim <launius@gmail.com>

SUMMARY = "A small image just capable of allowing to boot"

IMAGE_INSTALL = "packagegroup-core-boot ${CORE_IMAGE_EXTRA_INSTALL}"

IMAGE_LINGUAS = " "

LICENSE = "MIT"

inherit core-image
