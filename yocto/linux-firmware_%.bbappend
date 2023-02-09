user@user-VirtualBox:~/git/meta-launius-bsp$ git show HEAD
commit 558847ca503674264917b9b0efce3cf7a8c1d275 (HEAD -> device-honister, origin/device-honister)
Author: Yunjae Lim <launius@gmail.com>
Date:   Thu Feb 9 09:36:54 2023 +1100

    Add Realtek 8125b-2 firmware to rootfs

diff --git a/conf/machine/device-rk3588.conf b/conf/machine/device-rk3588.conf
index c12ea08..04ba2c1 100644
--- a/conf/machine/device-rk3588.conf
+++ b/conf/machine/device-rk3588.conf
@@ -16,3 +16,11 @@ RK_WIFIBT_FIRMWARES = " \
        rkwifibt-firmware-ap6275p-bt \
        brcm-tools \
 "
+
+RTL_NIC_FIRMWARES = " \
+       linux-firmware-rtl8125b-2 \
+"
+
+MACHINE_EXTRA_RRECOMMENDS:append = " \
+       ${RTL_NIC_FIRMWARES} \
+"
diff --git a/recipes-kernel/linux-firmware/files/firmware/rtl_nic/rtl8125b-2.fw b/recipes-kernel/linux-firmware/files/firmware/rtl_nic/rtl8125b-2.fw
new file mode 100644
index 0000000..dc753b5
Binary files /dev/null and b/recipes-kernel/linux-firmware/files/firmware/rtl_nic/rtl8125b-2.fw differ
diff --git a/recipes-kernel/linux-firmware/linux-firmware_%.bbappend b/recipes-kernel/linux-firmware/linux-firmware_%.bbappend
new file mode 100644
index 0000000..caaf827
--- /dev/null
+++ b/recipes-kernel/linux-firmware/linux-firmware_%.bbappend
@@ -0,0 +1,21 @@
+# Copyright (C) 2023, launius@gmail.com
+# Released under the MIT license (see COPYING.MIT for the terms)
+
+FILESEXTRAPATHS:prepend := "${THISDIR}/files:"
+
+# Install addition firmwares
+do_install:append() {
+       cp -r ${WORKDIR}/firmware ${D}${nonarch_base_libdir}/
+}
+
+SRC_URI:append = " \
+       file://firmware/rtl_nic/rtl8125b-2.fw \
+"
+
+PACKAGES:prepend = " \
+       ${PN}-rtl8125b-2 \
+"
+
+FILES:${PN}-rtl8125b-2 = " \
+       ${nonarch_base_libdir}/firmware/rtl_nic/rtl8125b-2.fw \
+"

