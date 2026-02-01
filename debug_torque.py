import serial
import time
import struct

# 配置
PORT = 'COM13' # 请修改为您的实际端口，例如 /dev/ttyACM0 或 COMx
BAUD = 115200

HEAD_1 = 0xA5
HEAD_2 = 0x5A
CMD_TYPE_TORQUE = 0x11

def send_torque(ser, enable):
    payload = bytearray([1 if enable else 0])
    packet = bytearray([HEAD_1, HEAD_2, CMD_TYPE_TORQUE, len(payload)])
    packet.extend(payload)
    checksum = sum(packet) & 0xFF
    packet.append(checksum)
    
    ser.write(packet)
    print(f"发送扭矩指令: {'上锁' if enable else '放松'} | Hex: {packet.hex()}")

def main():
    try:
        # 尝试自动寻找端口 (如果不想手动改)

        target_port = PORT

        print(f"正在打开串口: {target_port}")
        ser = serial.Serial(target_port, BAUD, timeout=1)
        
        # 1. 发送放松
        print("3秒后发送放松指令...")
        time.sleep(3)
        send_torque(ser, False)
        
        # 2. 等待用户确认
        input("按回车键发送上锁指令...")
        send_torque(ser, True)
        
        ser.close()
        
    except Exception as e:
        print(f"错误: {e}")

if __name__ == "__main__":
    main()
