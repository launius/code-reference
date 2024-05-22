"""
Copyright (c) 2024, Yunjae Lim <launius@gmail.com>

This is a Python program for Wi-Fi/Bluetooth functionality tests
using USB Human Interface Device (HID) APIs. To run this application,

1. Install the HID package:
# pip install hid

2. Allow read-write permission to the Linux HID device file:
# sudo chmod 666 /dev/hidraw#

3. Get vendor ID and product ID from 'lsusb' output:
user@user-MINIPC-PN50-E1:~/hidtest$ lsusb -tv
/:  Bus 05.Port 1: Dev 1, Class=root_hub, Driver=xhci_hcd/4p, 480M
    ID 1d6b:0002 Linux Foundation 2.0 root hub
    |__ Port 2: Dev 2, If 0, Class=Human Interface Device, Driver=usbhid, 1.5M
        ID 046d:c077 Logitech, Inc. M105 Optical Mouse
/:  Bus 03.Port 1: Dev 1, Class=root_hub, Driver=xhci_hcd/4p, 480M
    ID 1d6b:0002 Linux Foundation 2.0 root hub
    |__ Port 2: Dev 2, If 1, Class=Human Interface Device, Driver=usbhid, 1.5M
        ID 046d:c31d Logitech, Inc. Media Keyboard K200
    |__ Port 2: Dev 2, If 0, Class=Human Interface Device, Driver=usbhid, 1.5M
        ID 046d:c31d Logitech, Inc. Media Keyboard K200

4. Usage of HID APIs:
                                 Send OUT Report             Read IN Report
    - enable Wi-Fi:              0x 01 57 4e ('W' 'N')       0x02 'A' (if enabled) or 0x02 'N' (Not OK)
    - disable Wi-Fi:             0x 01 'W' 'F'               0x02 'A' (if disabled) or 0x02 'N' (Not OK)
    - scan Wi-Fi networks:       0x 01 'W' 'S'               0x02 'A' #(the number of scanned)
    - set SSID:                  0x 01 'W' 'A' "SSID"        0x02 'A' (OK)
    - set password:              0x 01 'W' 'B' "PASSWD"      0x02 'A' (OK)
    - connect to Wi-Fi:          0x 01 'W' 'C'               0x02 'A' (if connected)
    - disconnect from Wi-Fi:     0x 01 'W' 'D'               0x02 'A' (if disconnected)
    - query Wi-Fi state:         0x 01 'W' 'Q'               0x02 'A' (if enabled) or 0x02 'N' (if disabled)
    - read Wi-Fi conn state:     0x 01 'W' 'R'               0x02 'A' (if connected) or 0x02 'N' (if disconnected)
    - check bitrate:             0x 01 'W' 'W' #(server IP)  0x02 'A' #(bitrate)
    - get Wi-Fi MAC:             0x 01 'W' 'M'               0x02 'A' 12 34 56 78 9a bc (6bytes MAC)

    - get BT MAC address:        0x 01 'B' 'A'               0x02 'A' 12 34 56 78 9a bc (6bytes MAC)
    - enable BT:                 0x 01 'B' 'B'               0x02 'A' (if enabled) or 0x02 'N' (Not OK)
    - disable BT:                0x 01 'B' 'C'               0x02 'A' (if disabled)
    - enable BT discoverability  0x 01 'B' 'D'               0x02 'A' (if enabled)
    - disable BT discoverability 0x 01 'B' 'E'               0x02 'A' (if disabled)

"""

import hid
import time
import logging

class WifiDevice:
    def __init__(self) -> None:
        self.vendor_id = 0x12ab
        self.product_id = 0x003c

        self.hdev = hid.Device(self.vendor_id, self.product_id)
        logging.info("HID device (%s)", self.hdev.product)
        
    def hid_get_mac_addr(self) -> str:
        buf = [0x01, 0x57, 0x4d] + [0]*61
        self.hdev.write(bytes(buf))

        data = self.hdev.read(64)
        if data[1] == ord('A'):
            mac = ':'.join(format(x, '02x') for x in data[2:8])
            logging.info("%s", mac)
            return mac

        logging.error("error! %s %s", hex(data[0]), hex(data[1]))
        return ""

    def hid_enable_wifi(self) -> bool:
        buf = [0x01, 0x57, 0x4e] + [0]*61
        self.hdev.write(bytes(buf))

        data = self.hdev.read(64)
        if data[1] == ord('A'):
            logging.info("enabled")
            return True

        logging.error("error! %s %s", hex(data[0]), hex(data[1]))
        return False

    def hid_disable_wifi(self) -> bool:
        buf = [0x01, 0x57, 0x46] + [0]*61
        self.hdev.write(bytes(buf))

        data = self.hdev.read(64)
        if data[1] == ord('A'):
            logging.info("disabled")
            return True

        logging.error("error! %s %s", hex(data[0]), hex(data[1]))
        return False

    def hid_scan_networks(self) -> int:
        buf = [0x01, 0x57, 0x53] + [0]*61
        self.hdev.write(bytes(buf))

        data = self.hdev.read(64)
        if data[1] == ord('A'):
            logging.info("%d scanned", data[2])
            return data[2]

        logging.error("error! %s %s", hex(data[0]), hex(data[1]))
        return 0

    def hid_set_ssid(self) -> bool:
        buf = [0x01, 0x57, 0x41] + [0]*61
        ssid = "dlink-5GHz-5A"
        for i, char in enumerate(ssid):
            buf[i + 3] = ord(char)

        self.hdev.write(bytes(buf))
        data = self.hdev.read(64)
        if data[1] == ord('A'):
            logging.info("%s", ssid)
            return True

        logging.error("error! %s %s", hex(data[0]), hex(data[1]))
        return False

    def hid_set_passwd(self) -> bool:
        buf = [0x01, 0x57, 0x42] + [0]*61
        passwd = "password"
        for i, char in enumerate(passwd):
            buf[i + 3] = ord(char)

        self.hdev.write(bytes(buf))
        data = self.hdev.read(64)

        if data[1] == ord('A'):
            logging.info("%s", passwd)
            return True

        logging.error("error! %s %s", hex(data[0]), hex(data[1]))
        return False

    def hid_connect_to_wifi(self) -> bool:
        buf = [0x01, 0x57, 0x43] + [0]*61
        self.hdev.write(bytes(buf))

        data = self.hdev.read(64)
        if data[1] == ord('A'):
            logging.info("connected")
            return True

        logging.error("error! %s %s", hex(data[0]), hex(data[1]))
        return False

    def hid_disconnect_from_wifi(self) -> bool:
        buf = [0x01, 0x57, 0x44] + [0]*61
        self.hdev.write(bytes(buf))

        data = self.hdev.read(64)
        if data[1] == ord('A'):
            logging.info("disconnected")
            return True

        logging.error("error! %s %s", hex(data[0]), hex(data[1]))
        return False

    def hid_get_bandwidth(self) -> int:
        buf = [0x01, 0x57, 0x57] + [0]*61
        buf[3] = 192
        buf[4] = 168
        buf[5] = 1
        buf[6] = 9
        self.hdev.write(bytes(buf))

        data = self.hdev.read(64)
        if data[1] == ord('A'):
            logging.info("%d Mbits/sec", data[2])
            return data[2]

        logging.error("error! %s %s", hex(data[0]), hex(data[1]))
        return 0

class BluetoothController:
    def __init__(self) -> None:
        self.vendor_id = 0x12ab
        self.product_id = 0x003c

        self.hdev = hid.Device(self.vendor_id, self.product_id)
        logging.info("HID device (%s)", self.hdev.product)

    def hid_get_address(self) -> str:
        buf = [0x01, 0x42, 0x41] + [0]*61
        self.hdev.write(bytes(buf))

        data = self.hdev.read(64)
        if data[1] == ord('A'):
            mac = ':'.join(format(x, '02x') for x in data[2:8])
            logging.info("%s", mac)
            return mac

        logging.error("error! %s %s", hex(data[0]), hex(data[1]))
        return ""

    def hid_enable_bt(self) -> bool:
        buf = [0x01, 0x42, 0x42] + [0]*61
        self.hdev.write(bytes(buf))

        data = self.hdev.read(64)
        if data[1] == ord('A'):
            logging.info("enabled")
            return True

        logging.error("error! %s %s", hex(data[0]), hex(data[1]))
        return False

    def hid_disable_bt(self) -> bool:
        buf = [0x01, 0x42, 0x43] + [0]*61
        self.hdev.write(bytes(buf))

        data = self.hdev.read(64)
        if data[1] == ord('A'):
            logging.info("disabled")
            return True

        logging.error("error! %s %s", hex(data[0]), hex(data[1]))
        return False

    def hid_enable_discoverability(self) -> bool:
        buf = [0x01, 0x42, 0x44] + [0]*61
        self.hdev.write(bytes(buf))

        data = self.hdev.read(64)
        if data[1] == ord('A'):
            logging.info("enabled")
            return True

        logging.error("error! %s %s", hex(data[0]), hex(data[1]))
        return False

    def hid_disable_discoverability(self) -> bool:
        buf = [0x01, 0x42, 0x45] + [0]*61
        self.hdev.write(bytes(buf))

        data = self.hdev.read(64)
        if data[1] == ord('A'):
            logging.info("disabled")
            return True

        logging.error("error! %s %s", hex(data[0]), hex(data[1]))
        return False


if __name__ == "__main__":
    logging.basicConfig(format='%(funcName)s: %(message)s', level=logging.INFO)

    logging.info(hid.enumerate())

    # Wi-Fi functionality tests by HID APIs
    wifi = WifiDevice()

    wifi.hid_get_mac_addr()

    wifi.hid_disable_wifi()

    time.sleep(10)

    wifi.hid_enable_wifi()

    time.sleep(10)

    wifi.hid_scan_networks()

    wifi.hid_set_ssid()

    wifi.hid_set_passwd()

    wifi.hid_disconnect_from_wifi()

    time.sleep(5)

    wifi.hid_connect_to_wifi()

    time.sleep(5)

    wifi.hid_get_bandwidth()

    # Bluetooth functionality tests by HID APIs
    bt = BluetoothController()

    bt.hid_get_address()

    bt.hid_enable_bt()

    time.sleep(5)

    bt.hid_enable_discoverability()

    time.sleep(5)

    bt.hid_disable_discoverability()

    time.sleep(5)

    bt.hid_disable_bt()

