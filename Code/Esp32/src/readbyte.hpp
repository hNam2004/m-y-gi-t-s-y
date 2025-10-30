#ifndef READBYTE_HPP
#define READBYTE_HPP

#include <Arduino.h> // Cần thiết cho các kiểu dữ liệu như 'byte', 'bool'

// --- Buffer và Struct cho dữ liệu máy ---
const int RESPONSE_BUFFER_SIZE = 145;
extern byte responseBuffer[RESPONSE_BUFFER_SIZE]; // 'extern' báo rằng biến này được định nghĩa ở file .cpp

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

extern MachineData machineInfo; // 'extern' báo rằng biến này được định nghĩa ở file .cpp

// =================================================================
// ==== CÁC HÀM CÔNG KHAI (Public Functions) ====
// =================================================================

/**
 * @brief Tính toán Modbus RTU CRC-16
 */
unsigned int ModRTU_CRC(byte buf[], int len);

/**
 * @brief In một byte ra dạng nhị phân (dùng cho debug)
 */
void printByteAsBinary(byte data);

/**
 * @brief Xây dựng và gửi một khung truyền Modbus RTU để ghi 1 thanh ghi (Function Code 16)
 * @param commandType Loại lệnh ("c", "m", hoặc "p")
 * @param commandValue Giá trị (dưới dạng chuỗi) để ghi vào thanh ghi.
 */
void sendMobus(const char *commandType, const char *commandValue);

/**
 * @brief Gửi lệnh đọc trạng thái và trả về status.
 * @return 0 = Dừng, 1 = Chạy, -1 = Lỗi/Timeout
 */
int getMachineStatus();

/**
 * @brief Gửi lệnh đọc thông tin, nhận phản hồi và phân tích cú pháp.
 * @return true nếu đọc và phân tích thành công, false nếu thất bại.
 */
bool readAndParseMachineData();

#endif // READBYTE_HPP
