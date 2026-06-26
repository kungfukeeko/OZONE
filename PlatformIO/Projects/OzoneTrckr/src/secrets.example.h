#pragma once
// Template for secrets.h — copy this file to secrets.h and fill in your values.
// secrets.h is gitignored so your real credentials are never committed.
#define SECRET_WIFI_SSID "YOUR_WIFI_SSID"
#define SECRET_WIFI_PASS "YOUR_WIFI_PASSWORD"
#define SECRET_PC_HOST   "192.168.1.100"   // computer's IP address (TCP listener)
#define SECRET_PC_PORT   5000              // port the listener waits on

// Optional: static IP for the device (no DHCP lease to expire). Comment out for DHCP.
// Pick an address on your subnet that the router's DHCP pool won't also assign.
//#define SECRET_STATIC_IP "192.168.1.50"
//#define SECRET_GATEWAY   "192.168.1.1"
//#define SECRET_SUBNET    "255.255.255.0"
//#define SECRET_DNS       "192.168.1.1"
