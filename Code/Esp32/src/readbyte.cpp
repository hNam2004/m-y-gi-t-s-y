#include "readbyte.hpp"
#include <PubSubClient.h> // <<! QUAN TRỌNG: Bạn cần include thư viện MQTT ở đây

// =================================================================
// ==== CÁC BIẾN VÀ HÀM PHỤ THUỘC TỪ BÊN NGOÀI ====
// =================================================================
// File .cpp này cần truy cập 'client' và 'mqtt_topic_cmd'
// Chúng ta khai báo chúng là 'extern' để báo cho trình biên dịch biết
// rằng chúng sẽ được định nghĩa ở file .ino chính.

extern PubSubClient client;
extern const char *mqtt_topic_cmd;

// =================================================================
// ==== ĐỊNH NGHĨA BIẾN VÀ HẰNG SỐ NỘI BỘ ====
// =================================================================

// --- Khai báo các lệnh Modbus cố định ---
byte requestReadInfo[] = {0x01, 0x03, 0x03, 0x20, 0x00, 0x46, 0xC5, 0xB6};
byte requestReadStatus[] = {0x01, 0x01, 0x00, 0x00, 0x00, 0xA0, 0x3C, 0x72};

// --- Buffer và Struct cho dữ liệu máy ---
// Định nghĩa các biến đã được khai báo 'extern' trong .hpp
byte responseBuffer[RESPONSE_BUFFER_SIZE];
MachineData machineInfo;

// --- KHUNG TRUYỀN MẪU (Nội bộ của file .cpp này) ---
static const byte frameTemplate_C[] = {0x01, 0x10, 0x01, 0x2E, 0x00, 0x01, 0x02}; // Start/Stop/Next...
static const byte frameTemplate_M[] = {0x01, 0x10, 0x01, 0x31, 0x00, 0x01, 0x02}; // Giả lập coin
static const byte frameTemplate_P[] = {0x01, 0x10, 0x01, 0x32, 0x00, 0x01, 0x02}; // Chọn chương trình

// =================================================================
// ==== CÁC HÀM NỘI BỘ (Private Helper Functions) ====
// =================================================================

// HÀM GỬI DỮ LIỆU QUA CỔNG SERIAL (Nội bộ)
static void send_to_serial(byte _msg[], int _len)
{
    while (Serial.available())
        Serial.read(); // Xóa bộ đệm nhận trước khi gửi
    Serial.write(_msg, _len);
    Serial.flush(); // Đợi cho đến khi tất cả dữ liệu được gửi đi
}

// HÀM PHÂN TÍCH PHẢN HỒI (Nội bộ)
// (Tôi đã sửa lại hàm này để thực sự KIỂM TRA CRC)
static bool parseResponse(const byte *buffer, int len)
{
    if (len < 5)
        return false;
    if (buffer[0] != requestReadInfo[0] || buffer[1] != requestReadInfo[1])
        return false;

    int dataByteCount = buffer[2];
    if (len != 3 + dataByteCount + 2)
        return false;

    // Kiểm tra CRC
    uint16_t receivedCRC = (buffer[len - 2] << 8) | buffer[len - 1];
    uint16_t calculatedCRC = ModRTU_CRC((byte *)buffer, len - 2);

    if (receivedCRC != calculatedCRC)
    {
        Serial.println("[MODBUS RX] Error: CRC mismatch!");
        return false; // CRC KHÔNG KHỚP
    }

    // --- Phân tích dữ liệu ---
    machineInfo.temperature = (buffer[3 + 2] << 8) | buffer[3 + 3];
    machineInfo.warningCount = (buffer[3 + 8] << 8) | buffer[3 + 9];
    machineInfo.totalCoins = (long)(buffer[3 + 25] << 24) | (long)(buffer[3 + 26] << 16) | (long)(buffer[3 + 27] << 8) | buffer[3 + 28];
    machineInfo.coinsInBox = (long)(buffer[3 + 68] << 24) | (long)(buffer[3 + 69] << 16) | (long)(buffer[3 + 70] << 8) | buffer[3 + 71];
    machineInfo.runCount = (buffer[3 + 72] << 8) | buffer[3 + 73];

    int modelStartIndex = 3 + 80;
    int modelLen = 0;
    while (modelLen < 19 && (modelStartIndex + modelLen) < (len - 2) && buffer[modelStartIndex + modelLen] != 0)
    {
        machineInfo.modelName[modelLen] = (char)buffer[modelStartIndex + modelLen];
        modelLen++;
    }
    machineInfo.modelName[modelLen] = '\0';

    machineInfo.isValid = true;
    return true;
}

// =================================================================
// ==== THỰC THI CÁC HÀM CÔNG KHAI (Public Functions) ====
// =================================================================

// --- HÀM TÍNH TOÁN MODBUS CRC-16 ---
unsigned int ModRTU_CRC(byte buf[], int len)
{
    unsigned int crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++)
    {
        crc ^= (unsigned int)buf[pos];
        for (int i = 8; i != 0; i--)
        {
            if ((crc & 0x0001) != 0)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void printByteAsBinary(byte data)
{
    // Lặp từ bit 7 (MSB) về bit 0 (LSB)
    for (int i = 7; i >= 0; i--)
    {
        Serial.print(bitRead(data, i));
        if (i == 4)
        {
            Serial.print(" ");
        }
    }
}

/**
 * @brief Xây dựng và gửi một khung truyền Modbus RTU để ghi 1 thanh ghi (Function Code 16)
 */
void sendMobus(const char *commandType, const char *commandValue)
{
    const byte *templateFrame = NULL;
    size_t templateSize = 0;
    uint16_t dataValue = 0;

    uint16_t valueFromStr = atoi(commandValue);

    if (strcmp(commandType, "c") == 0)
    {
        templateFrame = frameTemplate_C;
        templateSize = sizeof(frameTemplate_C);
        dataValue = valueFromStr;
    }
    else if (strcmp(commandType, "m") == 0)
    {
        templateFrame = frameTemplate_M;
        templateSize = sizeof(frameTemplate_M);
        dataValue = valueFromStr;
    }
    else if (strcmp(commandType, "p") == 0)
    {
        templateFrame = frameTemplate_P;
        templateSize = sizeof(frameTemplate_P);
        dataValue = valueFromStr;
    }
    else
    {
        Serial.printf("Loi: Loai lenh khong xac dinh '%s'\n", commandType);
        return;
    }

    const size_t finalFrameSize = templateSize + 4;
    byte finalFrame[finalFrameSize];

    memcpy(finalFrame, templateFrame, templateSize);
    finalFrame[templateSize] = dataValue >> 8;
    finalFrame[templateSize + 1] = dataValue & 0xFF;

    unsigned int crc = ModRTU_CRC(finalFrame, templateSize + 2);
    finalFrame[templateSize + 2] = crc & 0xFF; // Gửi Low byte first
    finalFrame[templateSize + 3] = crc >> 8;  // Gửi High byte last

    Serial.print("[MODBUS TX]: ");
    for (size_t i = 0; i < finalFrameSize; i++)
    {
        Serial.printf("%02X ", finalFrame[i]);
    }
    Serial.println();

    send_to_serial(finalFrame, finalFrameSize); // Sử dụng hàm nội bộ

    // Publish MQTT
    char jsonMsg[50];
    sprintf(jsonMsg, "{\"t\":\"%s\", \"v\":\"%s\"}", commandType, commandValue);
    client.publish(mqtt_topic_cmd, jsonMsg);
}

<<<<<<< HEAD
uint16_t calculateCRC_Response(const byte *data, int len)
{
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++)
    {
        crc ^= (uint16_t)data[pos];
        for (int i = 8; i != 0; i--)
        {
            if ((crc & 0x0001) != 0)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return (crc << 8) | (crc >> 8);
}

bool parseResponse(const byte *buffer, int len)
{
    if (len < 145)
        return false;
    if (buffer[0] != requestReadInfo[0] || buffer[1] != requestReadInfo[1])
        return false;

    int dataByteCount = buffer[2];
    if (dataByteCount != 140)
        return false;

    if (len != 3 + dataByteCount + 2)
        return false;

    uint16_t receivedCRC = (buffer[len - 2] << 8) | buffer[len - 1];
    uint16_t calculatedCRC = calculateCRC_Response(buffer, len - 2);
    if (calculatedCRC != receivedCRC)
    {
        return false;
    }

    machineInfo.temperature = (buffer[3 + 4] << 8) | buffer[3 + 5];

    machineInfo.warningCount = (buffer[3 + 10] << 8) | buffer[3 + 11];

    uint16_t minutes = (buffer[3 + 24] << 8) | buffer[3 + 25];
    uint16_t seconds = (buffer[3 + 26] << 8) | buffer[3 + 27];

    machineInfo.totalCoins = (buffer[3 + 30] << 8) | buffer[3 + 31];
    machineInfo.coinsInBox = (buffer[3 + 70] << 8) | buffer[3 + 71];

    machineInfo.runCount = (buffer[3 + 72] << 8) | buffer[3 + 73];

    int modelStartIndex = 3 + 82;
    int modelLen = 0;
    while (modelLen < 19 && (modelStartIndex + modelLen) < (len - 2) && buffer[modelStartIndex + modelLen] != 0)
    {
        machineInfo.modelName[modelLen] = (char)buffer[modelStartIndex + modelLen];
        modelLen++;
    }
    machineInfo.modelName[modelLen] = '\0';

    machineInfo.isValid = true;
    return true;
}

=======
// (Tôi đã bỏ hàm calculateCRC_Response() của bạn vì nó không chuẩn
// và đã sửa lại hàm getMachineStatus để dùng ModRTU_CRC() )
>>>>>>> 7e33170f287134d57d6ad1dda1b8273d221c5467
int getMachineStatus()
{
    Serial.write(requestReadStatus, sizeof(requestReadStatus));
    unsigned long startTime = millis();
    int bytesRead = 0;
    byte statusBuffer[30];
    while (millis() - startTime < 1000 && bytesRead < 30)
    {
        if (Serial.available())
        {
            statusBuffer[bytesRead++] = Serial.read();
        }
    }

    if (bytesRead >= 6) // Ít nhất phải có ID, Func, Count, 1 byte Data, 2 byte CRC
    {
        // Giả sử slave gửi CRC theo kiểu MSB first (High byte, rồi Low byte)
        uint16_t receivedCRC = (statusBuffer[bytesRead - 2] << 8) | statusBuffer[bytesRead - 1];
        uint16_t calculatedCRC = ModRTU_CRC(statusBuffer, bytesRead - 2);

        if (calculatedCRC == receivedCRC)
        {
            byte statusByte = statusBuffer[9]; // Vị trí này theo code gốc của bạn
            if (statusByte != 0x00)
                return 1; // Máy đang chạy
            else
                return 0; // Máy đang dừng
        }
        else
        {
            Serial.println("[MODBUS RX] Status CRC Error");
            return -1; // Lỗi CRC
        }
    }
    return -1; // Lỗi hoặc timeout
}

bool readAndParseMachineData()
{
    Serial.write(requestReadInfo, sizeof(requestReadInfo));
    unsigned long startTime = millis();
    int bytesRead = 0;
    while (millis() - startTime < 500 && bytesRead < RESPONSE_BUFFER_SIZE)
    {
        if (Serial.available())
        {
            responseBuffer[bytesRead++] = Serial.read();
        }
    }

    if (bytesRead > 0)
    {
        return parseResponse(responseBuffer, bytesRead); // Sử dụng hàm nội bộ
    }
    machineInfo.isValid = false;
    return false;
}