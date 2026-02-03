import time
import struct
import serial
import serial.tools.list_ports
import sys

# 尝试导入 GPIO 库
try:
    import Jetson.GPIO as GPIO
except ImportError:
    print("[错误] 未安装 Jetson.GPIO 库")
    print("请运行: sudo pip3 install Jetson.GPIO")
    sys.exit(1)

# === 配置 (已根据 config.json 修正) ===
LED_PIN = 7       # 物理引脚 7 (靠近板子边缘的第4排)
TARGET_ID = 6     # 修改为 ID 6 (右前腿膝盖)，动作明显且安全
MOVE_POS = 2400   # 目标位置
REST_POS = 2048   # 初始位置
BAUD_RATE = 115200

# 协议常量
HEAD_1 = 0xA5
HEAD_2 = 0x5A
CMD_TYPE_SERVO_CTRL = 0x10
CMD_TYPE_TORQUE     = 0x11

def get_serial_port():
    ports = list(serial.tools.list_ports.comports())
    candidates = [p for p in ports if "ACM" in p.device or "USB" in p.device]
    if candidates: return candidates[0].device
    if ports: return ports[0].device
    return None

def send_packet(ser, pkt_type, payload):
    pkt = bytearray([HEAD_1, HEAD_2, pkt_type, len(payload)])
    pkt.extend(payload)
    pkt.append(sum(pkt) & 0xFF)
    ser.write(pkt)

def run():
    # 1. GPIO 初始化
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(LED_PIN, GPIO.OUT, initial=GPIO.LOW)
    
    # 2. 串口 初始化
    port_name = get_serial_port()
    if not port_name: print("未找到串口"); return
    ser = serial.Serial(port_name, BAUD_RATE)
    
    print(f"=== 光学延迟测试 (LED Latency Test) ===")
    print(f"LED: Pin {LED_PIN} | Servo: ID {TARGET_ID}")
    print(f"动作: {REST_POS} -> {MOVE_POS}")
    
    # === 预先打包数据 (极致同步的关键) ===
    # 构造 "动作" 数据包
    payload_move = bytearray([1])
    # ID, Pos, Speed(0=Max), Acc(0=Max)
    payload_move.extend(struct.pack('<B h h B', TARGET_ID, int(MOVE_POS), 0, 0)) 
    pkt_move = bytearray([HEAD_1, HEAD_2, CMD_TYPE_SERVO_CTRL, len(payload_move)])
    pkt_move.extend(payload_move)
    pkt_move.append(sum(pkt_move) & 0xFF)
    
    # 构造 "复位" 数据包
    payload_rest = bytearray([1])
    # ID, Pos, Speed(500), Acc(50) - 慢速复位
    payload_rest.extend(struct.pack('<B h h B', TARGET_ID, int(REST_POS), 500, 50)) 
    pkt_rest = bytearray([HEAD_1, HEAD_2, CMD_TYPE_SERVO_CTRL, len(payload_rest)])
    pkt_rest.extend(payload_rest)
    pkt_rest.append(sum(pkt_rest) & 0xFF)
    
    try:
        # 上劲
        send_packet(ser, CMD_TYPE_TORQUE, b'\x01')
        ser.write(pkt_rest) # 先归位
        time.sleep(1)
        
        while True:
            input("\n按 Enter 触发 (LED + 动作)...")
            
            # === 核心: 原子级同步操作 ===
            GPIO.output(LED_PIN, GPIO.HIGH)
            ser.write(pkt_move)
            # ============================
            
            print("  >>> 触发!")
            time.sleep(0.5)
            
            # 复位
            GPIO.output(LED_PIN, GPIO.LOW)
            ser.write(pkt_rest)
            
    except KeyboardInterrupt:
        pass
    finally:
        # 结束处理
        print("Cleaning up...")
        send_packet(ser, CMD_TYPE_TORQUE, b'\x00')
        GPIO.cleanup()
        ser.close()

if __name__ == "__main__":
    run()