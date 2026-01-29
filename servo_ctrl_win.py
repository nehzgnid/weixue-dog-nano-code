import serial
import serial.tools.list_ports
import time
import math
import sys

# === 协议常量 ===
HEADER = [0xA5, 0x5A]
SERVO_COUNT = 6

class WindowsServoController:
    def __init__(self, port="COM13", baud=115200):
        if port is None:
            port = self.auto_find_port()
        
        if port is None:
            print("[错误] 未发现可用串口ảng。")
            sys.exit(1)

        try:
            self.ser = serial.Serial(port, baud, timeout=1, write_timeout=1)
            print(f"[成功] 已连接至 Windows 串口: {port}")
        except Exception as e:
            print(f"[错误] 无法打开串口 {port}: {e}")
            self.ser = None

    def auto_find_port(self):
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            if "STMicroelectronics" in p.description or "USB Serial" in p.description:
                return p.device
        if ports:
            return ports[-1].device
        return None

    def send_packet(self, servo_data):
        if not self.ser: return
        packet = bytearray(HEADER)
        
        for item in servo_data:
            # 解析 (id, pos, [speed], [acc])
            servo_id = item[0]
            pos = item[1]
            speed = item[2] if len(item) > 2 else 0
            acc = item[3] if len(item) > 3 else 0

            safe_pos = max(0, min(4095, int(pos)))
            safe_speed = max(0, min(3000, int(speed)))
            safe_acc = max(0, min(254, int(acc)))

            packet.append(servo_id)
            packet.append(safe_pos & 0xFF)
            packet.append((safe_pos >> 8) & 0xFF)
            packet.append(safe_speed & 0xFF)
            packet.append((safe_speed >> 8) & 0xFF)
            packet.append(safe_acc & 0xFF)

        checksum = sum(packet) & 0xFF
        packet.append(checksum)
        try:
            self.ser.write(packet)
        except Exception as e:
            print(f"\n[错误] 发送失败: {e}")

    def close(self):
        if self.ser:
            self.ser.close()

def main():
    print("=== STM32 轮足机器人 Windows 调试端 ===")
    controller = WindowsServoController()
    
    try:
        t = 0
        while True:
            servo_data = []
            for i in range(1, 7):
                pos = 2048 + 800 * math.sin(t + i * 0.4)
                speed = 500
                if i == 1:
                    speed = 1000
                servo_data.append((i, int(pos), speed, 50))
            controller.send_packet(servo_data)
            time.sleep(0.02)
            t += 0.05
    except KeyboardInterrupt:
        reset_data = [(i, 2048) for i in range(1, 7)]
        controller.send_packet(reset_data)
        controller.close()

if __name__ == "__main__":
    main()
