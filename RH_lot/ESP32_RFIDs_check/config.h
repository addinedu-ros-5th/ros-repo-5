#ifndef CONFIG_H
#define CONFIG_H

// Define byte if not already defined
#ifndef byte
typedef unsigned char byte;
#endif

// WiFi info
const char* ssid = "addinedu_class_2 (2.4G)";
const char* password = "addinedu1";
const char* ssid2 = "your_ssid2";
const char* password2 = "your_password2";

// Server info
const char* server = "192.168.0.37";
const int port = 12346;
const char* server2 = "192.168.0.38";
const int port2 = 12347;

// Define a structure to hold a target UID with an index
struct IndexedUID {
    int index;
    byte uid[4];
};

// Define the target UID structures as a dictionary-like array
const IndexedUID targetUIDs[] = {
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

#endif