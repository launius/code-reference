# (c) 2023 Yunjae Lim <launius@gmail.com>

SUMMARY = "Production image that fully supports the target device"

IMAGE_FEATURES += "splash"

LICENSE = "MIT"

inherit core-image

IMAGE_INSTALL += "cpuset-audio-scripts audio-mixer-app"

CORE_IMAGE_EXTRA_INSTALL += " \
    packagegroup-audio \
    util-linux \
    ntpdate \
"
