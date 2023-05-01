SUMMARY = "Basic packagegroup"
PR = "r1"

#
# packages which content depend on MACHINE_FEATURES need to be MACHINE_ARCH
#
PACKAGE_ARCH = "${MACHINE_ARCH}"

inherit packagegroup

PACKAGES = ' \
            ${PN} \
            \
            ${@bb.utils.contains("DISTRO_FEATURES", "ipv6", "${PN}-ipv6", "", d)} \
            '

#
# rcv-packagegroup-base contain stuff needed for base system (machine related)
#
RDEPENDS:${PN} = "\
    packagegroup-distro-base \
    packagegroup-machine-base \
    dbus \
    module-init-tools \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ipv6', '${PN}-ipv6', '',d)} \
    "

RRECOMMENDS:${PN} = "\
    kernel-module-nls-utf8 \
    kernel-module-input \
    kernel-module-uinput \
    kernel-module-rtc-dev \
    kernel-module-rtc-proc \
    kernel-module-rtc-sysfs \
    kernel-module-unix"

#
# packages added by distribution
#
SUMMARY:packagegroup-distro-base = "${DISTRO} extras"
DEPENDS_packagegroup-distro-base = "${DISTRO_EXTRA_DEPENDS}"
RDEPENDS:packagegroup-distro-base = "${DISTRO_EXTRA_RDEPENDS}"
RRECOMMENDS:packagegroup-distro-base = "${DISTRO_EXTRA_RRECOMMENDS}"

#
# packages added by machine config
#
SUMMARY:packagegroup-machine-base = "${MACHINE} extras"
SUMMARY:packagegroup-machine-base = "Extra packages required to fully support ${MACHINE} hardware"
RDEPENDS:packagegroup-machine-base = "${MACHINE_EXTRA_RDEPENDS}"
RRECOMMENDS:packagegroup-machine-base = "${MACHINE_EXTRA_RRECOMMENDS}"

SUMMARY:${PN}-ipv6 = "IPv6 support"
RDEPENDS:${PN}-ipv6 = "\
    "

RRECOMMENDS:${PN}-ipv6 = "\
    kernel-module-ipv6 "
