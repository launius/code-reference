/*
 * Copyright (c) 2023, Yunjae Lim <launius@gmail.com>
 *
 * This is a test program for Wi-Fi network connection monitoring using the wpa_client library
 * and an example for assigning static network IP addresses on Linux systems.
 *
 
 * How to cross-compile libwpa_client.so from wpa_supplicant-2.10
	1. $ cd wpa_supplicant-2.10/wpa_supplicant
	2. $ cp defconfig .config
	3. $ source /opt/poky/3.1.6/environment-setup-aarch64-poky-linux
	4. edit Makefile in wpa_supplicant-2.10/wpa_supplicant/
		CC=aarch64-poky-linux-gcc -fstack-protector-strong -D_FORTIFY_SOURCE=2 -Wformat -Wformat-security -Werror=format-security --sysroot=/opt/poky/3.1.6/sysroots/aarch64-poky-linux
	5. $ make
	6. $ make libwpa_client.so
	7. $ make clean

 * How to build the network monitoring test app
	1. run $ make in the source directory

 *
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <wpa_ctrl.h>

#define CTRL_INTERFACE_DIR  "/var/run/wpa_supplicant/wlan0"

class NetworkMonitor
{
public:
    std::string getFileAsStringUsingFp(std::string path);
	std::string getFileAsStringUsingStream(std::string path);

	bool setStaticConfig();
	bool monitorWpa();
	
private:
	const std::string netConfig = "./interfaces";
	const std::string ip = "192.168.0.100";
	const std::string netmask = "255.255.255.0";
	const std::string gateway = "192.168.0.1";
};

std::string NetworkMonitor::getFileAsStringUsingFp(std::string path)
{
    std::ifstream ifs(path);
    std::string text;

    if (!ifs.is_open())	{
        std::cout << __func__ << ": file open error!\n";
        return std::string();
    }

    ifs.seekg(0, std::ios::end);
    std::ifstream::pos_type size = ifs.tellg();
    if (size < 0) {
        std::cout << __func__ << ": file size error!\n";
        return std::string();
	}

    text.resize(static_cast<size_t>(size));
    ifs.seekg(0, std::ios::beg);
    ifs.read(&text[0], size);

    ifs.close();

    return text;
}

std::string NetworkMonitor::getFileAsStringUsingStream(std::string path)
{
    std::ifstream in(path);
    std::ostringstream sstr;

    sstr << in.rdbuf();
    return sstr.str();
}

bool NetworkMonitor::setStaticConfig()
{
    std::string netFile;

    netFile = getFileAsStringUsingFp(netConfig);
	if (netFile.empty())
		return false;

    std::string from = "eth0 inet dhcp";
    std::string to = "eth0 inet static\n\taddress " + ip
                                   + "\n\tnetmask " + netmask
                                   + "\n\tgateway " + gateway;

    size_t pos = netFile.find(from);
    if (pos == std::string::npos) {
        std::cout << __func__ << ": no dhcp settings\n";
        return true;
    }

    netFile.replace(pos, from.length(), to);

    std::cout << __func__ << ": file text (\n" << netFile << ")\n";
	
	return true;
}

bool NetworkMonitor::monitorWpa()
{
	struct wpa_ctrl *monitor_conn = NULL;
    int conn_fd = -1;
    fd_set rfds;

    char buf[256];
    size_t len = sizeof(buf) - 1;

    struct timeval tv = {
        .tv_sec = 5,
        .tv_usec = 0
	};

	monitor_conn = wpa_ctrl_open(CTRL_INTERFACE_DIR);
	if (monitor_conn == NULL) {
		fprintf(stderr, "Open monitor conn failed");
		return false;
	}

	if (wpa_ctrl_attach(monitor_conn) != 0) {
		fprintf(stderr, "Attach monitor conn failed");
		return false;
	}

    if ((conn_fd = wpa_ctrl_get_fd(monitor_conn)) < 0) {
        fprintf(stderr, "Monitor conn get failed");
        return false;
    }

    while (monitor_conn) {
        FD_ZERO(&rfds);
        FD_SET(conn_fd, &rfds);
        if(select(conn_fd + 1, &rfds, NULL, NULL, &tv) < 0)
            break;

        if (FD_ISSET(conn_fd, &rfds)) {
            len = sizeof(buf) - 1;
            if (wpa_ctrl_recv(monitor_conn, buf, &len) < 0)
                break;
            
            buf[len] = '\0';
            printf("%s\n", buf);
        }
    }

	return true;
}

int main()
{
	NetworkMonitor mon;
	std::string netFile;

	// 1. network config file read test
	netFile = mon.getFileAsStringUsingFp("./interfaces");
    std::cout << __func__ << ": file text (\n" << netFile << ")\n";

	// 2. network config file read test
	netFile = mon.getFileAsStringUsingStream("./interfaces");
    std::cout << __func__ << ": file text (\n" << netFile << ")\n";

	// 3. set static IP test
	mon.setStaticConfig();

	// 4. Wi-Fi network connection monitoring
	mon.monitorWpa();

    return EXIT_SUCCESS;
}
