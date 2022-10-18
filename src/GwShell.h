// Shell header

#pragma once

#include <Arduino.h>
#include <map>

extern String WifiMode, WifiIP, WifiSSID;
extern String host_name, macAddress;


void initGwShell();
void setShellSource(Stream * stream);
void handleShell();

void ListDevices(Stream & stream, bool force);
void displayBoat(Stream & stream);

extern void disconnect();

extern Stream * Console;