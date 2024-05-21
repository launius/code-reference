/*
 * Copyright (c) 2024, Yunjae Lim <launius@gmail.com>
 *
 * This is part of a program that implements USB Human Interface Device (HID) APIs
 * for Wi-Fi/Bluetooth functionality tests. The program employs 'nmcli' and
 * 'bluetoothctl' commands to execute the tests.
 */

/*
 * How to test HID command APIs
 *
 * 1. Download HID API tool: https://github.com/todbot/hidpytoy
 *
 * 2. Connect to a USB HID device with vendor ID and product ID. The VID/PID are shown in 'lsusb'

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

 *
 * 3. Send report and Read. For example,

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
 *
 */

class HidComponent : public juce::Component
{
public:
    HidComponent();
    ~HidComponent() override;

private:
    bool readWifiStatus(juce::String str);
    bool readWifiConnStatus();
    int scanWifiNetworks();
    bool setWifiConfig(const uint8_t apConfig[], juce::String& config);
    bool connectToWifi();
    bool disconnectFromWifi();
    int runWifiIperf3(const uint8_t serverIp[]);
    juce::StringArray getWifiMacAddress();
    juce::StringArray getBTMacAddress();
    bool runBTControl(juce::String str);
    
    juce::String ssid;
    juce::String passwd;
    hidInterface* hidInterface;
};

HidComponent::HidComponent()
{
    hidInterface = new hidInterface();
    hidInterface->Init();
}

HidComponent::~HidComponent()
{
    delete hidInterface;
}

void HidComponent::handleHidMessage(const hidInterface::Message *msg)
{
    uint8_t response[64];

    int result;
    juce::StringArray tokens;

    switch (msg->getReport()[0]) {
        case 'W':   // Wi-Fi Testing
            switch (msg->getReport()[1]) {
                case 'N':   // oN
                    result = system("nmcli radio wifi on");
                    readWifiStatus("enabled") ? response[0] = 'A' : response[0] = 'N';
                    break;
                case 'F':   // ofF
                    result = system("nmcli radio wifi off");
                    readWifiStatus("disabled") ? response[0] = 'A' : response[0] = 'N';
                    break;
                case 'A':   // set ssid
                    setWifiConfig(msg->getReport()+2, ssid) ? response[0] = 'A' : response[0] = 'N';
                    break;
                case 'B':   // set password
                    setWifiConfig(msg->getReport()+2, passwd) ? response[0] = 'A' : response[0] = 'N';
                    break;
                case 'C':   // Connect
                    if (connectToWifi())
                        readWifiConnStatus() ? response[0] = 'A' : response[0] = 'N';
                    else    response[0] = 'N';
                    break;
                case 'D':   // Disconnect
                    if (disconnectFromWifi())
                        readWifiConnStatus() ? response[0] = 'N' : response[0] = 'A';
                    else    response[0] = 'N';
                    break;
                case 'Q':   // Query Wi-Fi state
                    readWifiStatus("enabled") ? response[0] = 'A' : response[0] = 'N';
                    break;
                case 'R':   // Read connection state
                    readWifiConnStatus() ? response[0] = 'A' : response[0] = 'N';
                    break;
                case 'S':   // Scan
                    result = scanWifiNetworks();
                    if (result > 0) {
                        response[0] = 'A';
                        response[1] = (uint8_t)result;
                    }
                    else    response[0] = 'N';
                    break;
                case 'W':   // Bandwidth
                    result = runWifiIperf3(msg->getReport()+2);
                    if (result > 0) {
                        response[0] = 'A';
                        response[1] = (uint8_t)result;
                    }
                    else
                        response[0] = 'N';
                    break;
                case 'M':   // Mac address
                    tokens = getWifiMacAddress();
                    if (tokens.size() == 6) {
                        response[0] = 'A';
                        for (int i = 0 ; i < tokens.size() ; i++)
                            response[i + 1] = (uint8_t)tokens[i].getHexValue32();
                    }
                    else    response[0] = 'N';
                    break;
                default:
                    printf("Wi-Fi HID Message not recognised\n");
                    break;
            }
            break;
        case 'B':   // BT Testing
            switch (msg->getReport()[1]) {
                case 'A':   // get Address
                    tokens = getBTMacAddress();
                    if (tokens.size() == 6) {
                        response[0] = 'A';
                        for (int i = 0 ; i < tokens.size() ; i++)
                            response[i + 1] = (uint8_t)tokens[i].getHexValue32();
                    }
                    else    response[0] = 'N';
                    break;
                case 'B':
                    runBTControl("power on") ? response[0] = 'A' : response[0] = 'N';
                    break;
                case 'C':
                    runBTControl("power off") ? response[0] = 'A' : response[0] = 'N';
                    break;
                case 'D':
                    runBTControl("discoverable on") ? response[0] = 'A' : response[0] = 'N';
                    break;
                case 'E':
                    runBTControl("discoverable off") ? response[0] = 'A' : response[0] = 'N';
                    break;
                default:
                    printf("BT HID Message not recognised\n");
                    break;
            }
            break;
        default:
            printf("HID Message not recognised\n");
            break;
    }
    
    hidInterface->WriteReport(response);
}

bool HidComponent::readWifiStatus(juce::String str)
{
    char buf[10];
    juce::String status;

    FILE *fp = popen("nmcli radio wifi", "r");
    fgets(buf, 10, fp);
    pclose(fp);

    status = buf;
    status = status.trim();

    std::cout << __func__ << ": (" << status << ") \n";
    return (status.compare(str) == 0);
}

bool HidComponent::readWifiConnStatus()
{
    char buf[128] = "";
    bool ret = false;
    juce::String command;

    if (ssid.isEmpty())
        return ret;

    command << "nmcli con show --active " << ssid;

    FILE *fp = popen(command.toStdString().c_str(), "r");
    if (fgets(buf, 128, fp) != NULL) {
        ret = true;
        std::cout << __func__ << ": (connected)\n";
    }
    else
        std::cout << __func__ << ": (disconnected)\n";
    pclose(fp);

    return ret;
}

int HidComponent::scanWifiNetworks()
{
    int count = 0;
    char buf[128];
    juce::String status;

    FILE *fp = popen("nmcli dev wifi list", "r");
    while (fgets(buf, 128, fp) != NULL) {
        count++;
        std::cout << __func__ << ": " << count << " " << buf;
    }
    pclose(fp);

    return (count - 1);
}

bool HidComponent::setWifiConfig(const uint8_t apConfig[], juce::String& config)
{
    config = juce::String(reinterpret_cast<const char *>(apConfig));
    std::cout << __func__ << ": ssid or passwd (" << config << ") " << '\n';

    return true;
}

bool HidComponent::connectToWifi()
{
    int result;
    juce::String command;

    if (ssid.isEmpty() || passwd.isEmpty())
        return false;

    command << "nmcli device wifi connect " << ssid << " password " << passwd;
    std::cout << __func__ << ": " << command << "\n";
    result = system(command.toStdString().c_str());

    return true;
}

bool HidComponent::disconnectFromWifi()
{
    int result;
    juce::String command;

    if (ssid.isEmpty())
        return false;

    command << "nmcli connection delete id " << ssid;
    std::cout << __func__ << ": " << command << "\n";
    result = system(command.toStdString().c_str());

    return true;
}

int HidComponent::runWifiIperf3(const uint8_t serverIp[])
{
    char buf[128] = "";
    int bitrate = 0;
    juce::String command;
    
    command << "iperf3 -c " << serverIp[0] << "." << serverIp[1]
         << "." << serverIp[2] << "." << serverIp[3] << " -f m";
    std::cout << __func__ << ": running " << command << " ...\n";

    FILE *fp = popen(command.toStdString().c_str(), "r");
    while (fgets(buf, 128, fp) != NULL) {
        std::cout << __func__ << ": " << buf;

        if (strstr(buf, "receiver"))
            sscanf(buf, "%*s %*s %*s %*s %*s %*s %d", &bitrate);
    }
    pclose(fp);

    if (bitrate > 0xff)
        bitrate = 0xff;     // set limit to uint8_t

    std::cout << __func__ << ": bitrate 0x" << bitrate << '\n';
    return bitrate;
}

juce::StringArray HidComponent::getWifiMacAddress()
{
    juce::File macFile("/sys/class/net/wlan0/address");

    juce::String macString("");
    if (macFile.exists()) {
        macString = macFile.loadFileAsString();
        macString = macString.trim();
    }

    juce::StringArray macTokens;
    macTokens.addTokens(macString, ":", "\"");

    std::cout << __func__ << ": mac ";
    for (int i = 0 ; i < macTokens.size() ; i++)
        std::cout << std::setw(2) << std::hex << macTokens[i].getHexValue32() << ":";
    std::cout << '\n';

    return macTokens;
}

juce::StringArray HidComponent::getBTMacAddress()
{
    char buf[128] = {0};
    juce::String macString("");
    juce::StringArray strTokens, macTokens;

    FILE *fp = popen("bluetoothctl list", "r");
    fgets(buf, 128, fp);
    pclose(fp);

    macString = buf;
    strTokens.addTokens(macString, " ", "\"");

    macString = strTokens[1];
    macTokens.addTokens(macString, ":", "\"");

    std::cout << __func__ << ": mac ";
    for (int i = 0 ; i < macTokens.size() ; i++)
        std::cout << std::setw(2) << std::hex << macTokens[i].getHexValue32() << ":";
    std::cout << '\n';

    return macTokens;
}

bool HidComponent::runBTControl(juce::String str)
{
    char buf[128];
    bool ret = false;
    juce::String command;
    
    command << "bluetoothctl " << str;
    FILE *fp = popen(command.toStdString().c_str(), "r");

    while (fgets(buf, 128, fp) != NULL) {
        if (strstr(buf, "succeeded")) {
            std::cout << __func__ << ": " << buf;
            ret = true;
            break;
        }
    }

    pclose(fp);
    return ret;
}
