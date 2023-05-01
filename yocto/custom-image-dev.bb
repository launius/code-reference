# (c) 2023 Yunjae Lim <launius@gmail.com>

SUMMARY = "Development image"

inherit core-image
require custom-image.bb

IMAGE_FEATURES += "tools-debug debug-tweaks"

CORE_IMAGE_EXTRA_INSTALL += "\
	ethtool \
	evtest \
	fbset \
	i2c-tools \
	memtester \
	tinymembench \
	pciutils \
	openssh \
	iperf3 \
	i2cfw \
"
