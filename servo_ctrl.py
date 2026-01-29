import serial
import time
import math
import struct

# === 配置参数 ===
# 串口号配置
# Linux/Jetson Nano: 通常是 '/dev/ttyACM0' 或 '/dev/ttyUSB0'
SERIAL_PORT = '/dev/ttyACM0' 
BAUD_RATE = 115200 # USB虚拟串口波特率可随意设置

# 协议常量
HEADER = [0xA5, 0x5A] # 帧头 0xA5 0x5A
SERVO_COUNT = 6       # 控制舵机数量

class ServoController:
    def __init__(self, port, baud):
        """初始化串口连接"""
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            print(f"[Info] 成功打开串口: {port}")
        except Exception as e:
            print(f"[Error] 无法打开串口 {port}: {e}")
            self.ser = None

    def send_packet(self, servo_data):
        """
        发送二进制舵机控制包
        参数 servo_data: 包含6个元组的列表 [(id, pos, [speed], [acc]), ...]
                        speed 和 acc 可选，默认为 0 (最大)
        """
        if not self.ser:
            return

        # 数据量检查
        if len(servo_data) != SERVO_COUNT:
            print(f"[Error] 数据错误: 需要 {SERVO_COUNT} 个舵机的数据，实际收到 {len(servo_data)} 个")
            return

        packet = bytearray()
        
        # 1. 添加帧头 (Header)
        packet.extend(HEADER)
        
        # 2. 添加舵机数据 (Payload)
        for item in servo_data:
            # 解析参数，支持变长元组 (id, pos) 或 (id, pos, speed, acc)
            servo_id = item[0]
            pos = item[1]
            speed = item[2] if len(item) > 2 else 0 # 默认 0 (最快)
            acc = item[3] if len(item) > 3 else 0   # 默认 0 (最大)

            # 限制范围
            safe_pos = max(0, min(4095, int(pos)))
            safe_speed = max(0, min(3000, int(speed))) # ST3215 0-3000
            safe_acc = max(0, min(254, int(acc)))

            packet.append(servo_id)               # ID (1字节)
            packet.append(safe_pos & 0xFF)        # Pos Low
            packet.append((safe_pos >> 8) & 0xFF) # Pos High
            packet.append(safe_speed & 0xFF)      # Speed Low
            packet.append((safe_speed >> 8) & 0xFF)# Speed High
            packet.append(safe_acc & 0xFF)        # Acc (1字节)

        # 3. 计算校验和 (Checksum)
        # 算法: 前38个字节的累加和，取低8位
        checksum = sum(packet) & 0xFF
        packet.append(checksum)
        
        # 4. 发送数据
        try:
            self.ser.write(packet)
            # print(f"[Debug] 发送({len(packet)}): {packet.hex()}") 
        except Exception as e:
            print(f"[Error] 发送失败: {e}")

    def close(self):
        """关闭串口"""
        if self.ser:
            self.ser.close()

def main():
    # 实例化控制器
    controller = ServoController(SERIAL_PORT, BAUD_RATE)
    if not controller.ser:
        return

    print("=== 开始测试: 正弦波运动控制 (Jetson Nano) ===")
    print("提示: 按 Ctrl+C 停止测试")
    
    try:
        t = 0
        while True:
            servo_data = []
            for i in range(1, 7):
                phase = t + (i * 0.5)
                pos = 2048 + 1000 * math.sin(phase)
                servo_data.append((i, int(pos)))
            
            controller.send_packet(servo_data)
            time.sleep(0.02)
            t += 0.1

    except KeyboardInterrupt:
        print("\n[Info] 用户停止测试")
        reset_data = [(i, 2048) for i in range(1, 7)]
        controller.send_packet(reset_data)
        time.sleep(0.1)
        controller.close()

if __name__ == '__main__':
    main()
