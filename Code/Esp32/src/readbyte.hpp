#ifndef READBYTE_HPP
#define READBYTE_HPP

#include <Arduino.h>
#include <PubSubClient.h> // Cần để khai báo 'extern client'

// --- Định nghĩa cấu trúc dữ liệu máy ---
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

// --- KHAI BÁO BIẾN TOÀN CỤC (EXTERN) ---
// Báo cho trình biên dịch biết rằng các biến này tồn tại ở một file .cpp khác (main.cpp)
extern PubSubClient client;
extern const char *mqtt_topic_cmd;

// Biến này sẽ được ĐỊNH NGHĨA trong readbyte.cpp, nhưng main.cpp cần thấy nó
extern MachineData machineInfo; 

// --- KHAI BÁO CÁC HÀM ---
// (Các hàm này sẽ được định nghĩa trong readbyte.cpp)

unsigned int ModRTU_CRC(byte buf[], int len);
void printByteAsBinary(byte data);
void send_to_serial(byte _msg[], int _len);
void sendMobus(const char *commandType, const char *commandValue);
uint16_t calculateCRC_Response(const byte *data, int len);
bool parseResponse(const byte *buffer, int len);
int getMachineStatus();
bool readAndParseMachineData();

#endif // READBYTE_HPP