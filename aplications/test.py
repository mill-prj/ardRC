import serial
import time

# 블루투스 시리얼 포트 설정
# Windows 예시: 'COM5'
# Linux 예시: '/dev/rfcomm0'
PORT = 'COM5'
BAUDRATE = 9600  # 장치 설정에 맞춰 조정

try:
    # 시리얼 포트 열기
    bt = serial.Serial(PORT, BAUDRATE, timeout=1)
    print(f"[+] Connected to {PORT} at {BAUDRATE} baud.")

    while True:
        if bt.in_waiting > 0:
            data = bt.readline().decode('utf-8').strip()
            if data:
                print(f"Received: {data}")
        time.sleep(0.1)

except serial.SerialException as e:
    print(f"[!] Serial error: {e}")

except KeyboardInterrupt:
    print("\n[+] Stopped by user.")

finally:
    try:
        bt.close()
        print("[+] Connection closed.")
    except:
        pass
