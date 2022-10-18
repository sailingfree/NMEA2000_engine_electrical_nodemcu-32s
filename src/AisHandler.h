// AIS handler

#pragma once

#ifndef defined_NMEA2000
#define defined_NMEA2000 1   // Dont duplicate the definition
#endif

#include <N2kMsg.h>
#include <N2kTypes.h>
#include <NMEA2000.h>

extern tNMEA2000 &NMEA2000;

#include <N2kMessages.h>
extern tNMEA2000 &NMEA2000;


#include <NMEA0183.h>
#include <NMEA0183Messages.h>
#include <NMEA0183Msg.h>
#include "NMEA0183AIStoNMEA2000.h"  // Contains class, global variables and code !!!


#define MAX_NMEA0183_MESSAGE_SIZE 150  // For AIS
#define ENABLE_DEBUG_LOG 1

extern tNMEA0183 AIS_NMEA0183;

void handleAis();