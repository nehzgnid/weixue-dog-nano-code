import serial
import time
import math
import struct

# === 配置参数 ===
# 串口号配置
# Linux/Jetson Nano: 通常是 '/dev/ttyACM0' 或 '/dev/ttyUSB0'
# Windows: 通常是 'COM3', 'COM4' 等
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
        参数 servo_data: 包含6个元组的列表 [(id, pos), (id, pos)...]
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
        for servo_id, pos in servo_data:
            # 限制位置范围在 0-4095 之间，防止越界
            safe_pos = max(0, min(4095, int(pos)))
            
            packet.append(servo_id)             # ID (1字节)
            packet.append(safe_pos & 0xFF)      # 位置低8位
            packet.append((safe_pos >> 8) & 0xFF) # 位置高8位

        # 3. 计算校验和 (Checksum)
        # 算法: 前20个字节的累加和，取低8位
        checksum = sum(packet) & 0xFF
        packet.append(checksum)
        
        # 4. 发送数据
        try:
            self.ser.write(packet)
            # print(f"[Debug] 发送: {packet.hex()}") # 需要调试时取消注释
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

    print("=== 开始测试: 正弦波运动控制 ===")
    print("提示: 按 Ctrl+C 停止测试")
    
    try:
        t = 0
        while True:
            # === 生成测试数据 ===
            # 让 6 个舵机做相位不同的正弦运动
            # 中位: 2048, 幅度: 1000
            servo_data = []
            
            for i in range(1, 7): # 舵机 ID 从 1 到 6
                # 每个舵机相位差 0.5 弧度，形成波浪效果
                phase = t + (i * 0.5)
                # 计算目标位置
                pos = 2048 + 1000 * math.sin(phase)
                servo_data.append((i, int(pos)))
            
            # === 发送指令 ===
            controller.send_packet(servo_data)
            
            # === 控制频率 ===
            # 20ms = 50Hz，这是典型的机器人控制频率
            time.sleep(0.02)
            t += 0.1 # 时间增量

    except KeyboardInterrupt:
        print("\n[Info] 用户停止测试")
        print("[Info] 正在发送回中指令...")
        # 停止时所有舵机归中 (2048)
        reset_data = [(i, 2048) for i in range(1, 7)]
        controller.send_packet(reset_data)
        time.sleep(0.1)
        controller.close()
        print("[Info] 程序已退出")

if __name__ == '__main__':
    main()