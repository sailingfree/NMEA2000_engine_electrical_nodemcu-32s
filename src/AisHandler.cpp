// Naiad AIS Handler

#include <AisHandler.h>


MyAisDecoder decoder;               // Create decoder object
AIS::DefaultSentenceParser parser;  // Create parser object


// Handle the AIS messages on Serial2
void handleAis() {
    tNMEA0183Msg AIS_NMEA0183Msg;
    char AisBuff[MAX_NMEA0183_MESSAGE_SIZE];


    if (AIS_NMEA0183.GetMessage(AIS_NMEA0183Msg)) {  // Get AIS NMEA sentences from serial2
        size_t i = 0;

        AIS_NMEA0183Msg.GetMessage(AisBuff, MAX_NMEA0183_MESSAGE_SIZE);  // send to buffer
        strcat(AisBuff, "\n");                                       // Decoder expects that.

#if ENABLE_DEBUG_LOG == 1
        Serial.println(AisBuff);
#endif
        // decode the messages.
        do {
            i = decoder.decodeMsg(AisBuff, strlen(AisBuff), i, parser);  // Decode AIVDM message.
        } while (i != 0);                                                // To be called until return value is 0
    }
}
