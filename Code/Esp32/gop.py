import serial
import time
import csv
from datetime import datetime

# ================== CONFIG ==================
PORT = "COM3"            # ⚠️ chỉnh đúng cổng COM
BAUDRATE = 2000000       # Trùng UART STM32
FRAME_LEN = 12           # 12 byte mỗi frame
CSV_FILE = "adc_data.csv"
# ============================================


def read_frames(ser, duration=1):
    """Đọc dữ liệu trong 'duration' giây, đếm số frame hợp lệ, xóa đúng 12 byte sau mỗi frame."""
    start_time = time.time()
    frames = []
    count = 0
    buffer = bytearray()

    while time.time() - start_time < duration:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            buffer.extend(data)

            # Khi đủ 12 byte → xử lý frame → xóa đúng 12 byte đầu
            while len(buffer) >= FRAME_LEN:
                frame = buffer[:FRAME_LEN]
                frames.append(frame)
                count += 1
                del buffer[:FRAME_LEN]

    # 🧩 Debug xem buffer có giảm đúng hay không
    print(f"Buffer còn lại: {len(buffer)} bytes")
                
    return frames, count


def parse_frame(frame):
    """Giải mã frame 12 byte thành 3 giá trị 24-bit signed."""
    if len(frame) != FRAME_LEN:
        return None

    channels = []
    for i in range(3, 12, 3):
        raw = (frame[i] << 16) | (frame[i + 1] << 8) | frame[i + 2]
        if raw & 0x800000:
            raw -= 1 << 24
        channels.append(raw)
    return channels


def save_to_csv(all_data):
    """Lưu dữ liệu vào file CSV."""
    now = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"{now}_{CSV_FILE}"

    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Time (s)", "Ch0", "Ch1", "Ch2"])
        for t, ch0, ch1, ch2 in all_data:
            writer.writerow([t, ch0, ch1, ch2])

    print(f"✅ Đã lưu {len(all_data)} mẫu vào '{filename}'.")


def main():
    ser = serial.Serial(PORT, BAUDRATE, timeout=0.001)
    print(f"Đang đọc từ {PORT} @ {BAUDRATE}... Nhấn Ctrl+C để dừng.\n")

    all_data = []
    start = time.time()

    try:
        while True:
            frames, count = read_frames(ser, duration=1)
            print(f"SPS = {count}")

            timestamp = time.time() - start
            for frame in frames:
                parsed = parse_frame(frame)
                if parsed:
                    all_data.append((timestamp, *parsed))

    except KeyboardInterrupt:
        print("\nDừng đọc. Ghi file CSV...")
        save_to_csv(all_data)
        ser.close()


if __name__ == "__main__":
    main()
