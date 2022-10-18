// System and network info
#pragma once

#include <Arduino.h>
#include <map>

extern String WifiMode, WifiIP, WifiSSID;
extern String host_name, macAddress;
extern String Model;


void getNetInfo(Stream & s);
void getSysInfo(Stream & s);
int getCpuAvg(int core);
void getN2kMsgs(Stream & s);

