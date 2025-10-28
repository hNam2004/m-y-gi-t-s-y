#ifndef READBYTE_HPP
#define READBYTE_HPP

#include <Arduino.h>
#include <PubSubClient.h> 

struct MachineData
{
    int temperature;
    int warningCount;
    long totalCoins;
    long coinsInBox;
    int runCount;
    char modelName[20];
    bool isValid;
};


extern PubSubClient client;
extern const char *mqtt_topic_cmd;

extern MachineData machineInfo; 

unsigned int ModRTU_CRC(byte buf[], int len);
void printByteAsBinary(byte data);
void send_to_serial(byte _msg[], int _len);
void sendMobus(const char *commandType, const char *commandValue);
uint16_t calculateCRC_Response(const byte *data, int len);
bool parseResponse(const byte *buffer, int len);
int getMachineStatus();
bool readAndParseMachineData();

#endif 