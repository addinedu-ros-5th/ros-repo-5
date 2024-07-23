#ifndef CONFIG_H
#define CONFIG_H

// Define byte if not already defined
#ifndef byte
typedef unsigned char byte;
#endif

class Config {
public:
    // WiFi info
    const char* ssid = "yohan";
    const char* password =  "12345678";

    // const char* ap_ssid = "yohan"; 
    // const char* ap_password = "12345678";

    // Server info
    const char* tcp_server = "192.168.99.216";
    const int tcp_port = 8080;

    // const char* ros_server = "192.168.0.39";
    // const int ros_port = 12347;

  

    // Define a structure to hold a target UID with an index
    struct IndexedUID {
        int index;
        byte uid[4];
    };

    // Define the target UID structures as a dictionary-like array
    const IndexedUID targetUIDs[7] = {
        {0, {0x00, 0x00, 0x00, 0x00}},  // Empty UID for index 0
        {1, {0xC3, 0x49, 0x6F, 0x96}},  // targetUID1
        {2, {0x43, 0xA6, 0x88, 0xA9}},  // targetUID2
        {3, {0xb2, 0x42, 0x93, 0x1d}},  // targetUID3
        {4, {0x61, 0x63, 0x72, 0x8f}},  // targetUID4
        {5, {0xe9, 0xf7, 0x97, 0x24}},  // targetUID5
        {6, {0x03, 0xfa, 0x85, 0x0f}}   // targetUID6
    };

    // Number of UIDs (including the empty one)
    const int NUM_UIDS = sizeof(targetUIDs) / sizeof(targetUIDs[0]);

    // Method to get the target UID by index
    IndexedUID getTargetUID(int index) const {
        if (index < 0 || index >= NUM_UIDS) {
            return targetUIDs[0]; // Return the empty UID if the index is out of range
        }
        return targetUIDs[index];
    }
};

#endif // CONFIG_H
