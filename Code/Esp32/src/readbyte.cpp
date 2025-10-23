#include "readbyte.hpp"

// --- Định nghĩa các lệnh Modbus cố định ---
byte requestReadInfo[] = {0x01, 0x03, 0x03, 0x20, 0x00, 0x46, 0xC5, 0xB6};
byte requestReadStatus[] = {0x01, 0x01, 0x00, 0x00, 0x00, 0xA0, 0x3C, 0x72};

// --- Khung truyền mẫu cho các lệnh Modbus GHI ---
const byte frameTemplate_C[] = {0x01, 0x10, 0x01, 0x2E, 0x00, 0x01, 0x02}; // Start/Stop/Next...
const byte frameTemplate_M[] = {0x01, 0x10, 0x01, 0x31, 0x00, 0x01, 0x02}; // Giả lập coin
const byte frameTemplate_P[] = {0x01, 0x10, 0x01, 0x32, 0x00, 0x01, 0x02}; // Chọn chương trình

// --- Buffer và Struct cho dữ liệu máy ---
const int RESPONSE_BUFFER_SIZE = 145;
byte responseBuffer[RESPONSE_BUFFER_SIZE];

// --- Định nghĩa biến machineInfo (đã được khai báo extern trong .hpp) ---
MachineData machineInfo;


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

// --- HÀM GỬI DỮ LIỆU QUA CỔNG SERIAL ---
void send_to_serial(byte _msg[], int _len)
{
    while (Serial.available())
        Serial.read(); // Xóa bộ đệm nhận trước khi gửi
    Serial.write(_msg, _len);
    Serial.flush(); // Đợi cho đến khi tất cả dữ liệu được gửi đi
}

/**
 * @brief Xây dựng và gửi một khung truyền Modbus RTU để ghi 1 thanh ghi (Function Code 16)
 * @param commandType Loại lệnh ("c", "m", hoặc "p") để chọn thanh ghi đích.
 * @param commandValue Giá trị (dưới dạng chuỗi) để ghi vào thanh ghi.
 */
void sendMobus(const char *commandType, const char *commandValue)
{
    // HÀM GỐC CỦA BẠN - KHÔNG SỬA ĐỔI
    // Nó sử dụng trực tiếp 'Serial', 'ModRTU_CRC', 'send_to_serial', 
    // 'client', và 'mqtt_topic_cmd'
    
    const byte *templateFrame = NULL;
    size_t templateSize = 0;
    uint16_t dataValue = 0; // Giá trị sẽ được ghi vào thanh ghi

    // Chuyển đổi giá trị từ chuỗi sang số (sẽ dùng cho trường hợp chung)
    uint16_t valueFromStr = atoi(commandValue);

    // --- LOGIC CHỌN TEMPLATE ĐÃ SỬA ĐỔI ---
    if (strcmp(commandType, "c") == 0)
    {
        templateFrame = frameTemplate_C;
        templateSize = sizeof(frameTemplate_C);
        dataValue = valueFromStr; // Dùng giá trị từ input
    }
    else if (strcmp(commandType, "m") == 0)
    {
        templateFrame = frameTemplate_M;
        templateSize = sizeof(frameTemplate_M);
        dataValue = valueFromStr; // Dùng giá trị từ input
    }
    else if (strcmp(commandType, "p") == 0)
    {
        templateFrame = frameTemplate_P;
        templateSize = sizeof(frameTemplate_P);
        dataValue = valueFromStr; // Dùng giá trị từ input
    }
    else
    {
        Serial.printf("Loi: Loai lenh khong xac dinh '%s'\n", commandType);
        return;
    }
    // --- HẾT LOGIC SỬA ĐỔI ---

    // Kích thước cuối cùng = template (7) + 2 byte dữ liệu + 2 byte CRC = 11 bytes
    const size_t finalFrameSize = templateSize + 4;
    byte finalFrame[finalFrameSize];

    // 1. Sao chép khung mẫu vào
    memcpy(finalFrame, templateFrame, templateSize);

    // 2. Gắn 2 byte dữ liệu vào (high byte first)
    finalFrame[templateSize] = dataValue >> 8;
    finalFrame[templateSize + 1] = dataValue & 0xFF;

    // 3. Tính CRC cho phần dữ liệu đã có (template + 2 byte data)
    unsigned int crc = ModRTU_CRC(finalFrame, templateSize + 2);

    // 4. Gắn 2 byte CRC vào cuối (low byte first)
    finalFrame[templateSize + 2] = crc & 0xFF;
    finalFrame[templateSize + 3] = crc >> 8;

    Serial.print("[MODBUS TX]: ");
    for (size_t i = 0; i < finalFrameSize; i++)
    {
        Serial.printf("%02X ", finalFrame[i]);
    }
    Serial.println();

    // (Giả sử hàm send_to_serial() tồn tại)
    send_to_serial(finalFrame, finalFrameSize);

    // Sau khi gửi lệnh, publish lại trạng thái lệnh lên MQTT
    char jsonMsg[50];
    sprintf(jsonMsg, "{\"t\":\"%s\", \"v\":\"%s\"}", commandType, commandValue);
    // Sử dụng 'client' và 'mqtt_topic_cmd' toàn cục
    client.publish(mqtt_topic_cmd, jsonMsg);
}

// =================================================================
// ==== CÁC HÀM XỬ LÝ PHẢN HỒI MODBUS (CỦA BẠN) ====
// =================================================================

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
    return (crc << 8) | (crc >> 8); // CRC cho phản hồi có thể bị đảo byte
}

bool parseResponse(const byte *buffer, int len)
{
    // Toàn bộ logic gốc của bạn
    if (len < 5)
        return false;
    if (buffer[0] != requestReadInfo[0] || buffer[1] != requestReadInfo[1])
        return false;

    int dataByteCount = buffer[2];
    if (len != 3 + dataByteCount + 2)
        return false;

    uint16_t receivedCRC = (buffer[len - 2] << 8) | buffer[len - 1];
    uint16_t calculatedCRC = calculateCRC_Response(buffer, len - 2);
    // (Bạn nên kiểm tra CRC ở đây)

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

int getMachineStatus()
{
    // Sử dụng 'Serial' trực tiếp
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

    if (bytesRead >= 6)
    {
        byte statusByte = statusBuffer[6];
        uint16_t receivedCRC = (statusBuffer[bytesRead - 2] << 8) | statusBuffer[bytesRead - 1];
        if (calculateCRC_Response(statusBuffer, bytesRead - 2) == receivedCRC)
        {
            if (statusByte != 0x00)
                return 1; // Máy đang chạy
            else
                return 0; // Máy đang dừng
        }
    }
    return -1; // Lỗi hoặc timeout
}

bool readAndParseMachineData()
{
    // Sử dụng 'Serial' trực tiếp
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
        return parseResponse(responseBuffer, bytesRead);
    }
    machineInfo.isValid = false;
    return false;
}