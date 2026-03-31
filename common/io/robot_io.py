import serial
import serial.tools.list_ports
import threading
import time
import struct
import os
from ..config.robot_config import cfg
from .servo_safety import (
    ServoSafetyGuard,
    build_servo_control_payload,
    split_servo_batches,
)

# Protocol Constants
HEAD_1 = 0xA5
HEAD_2 = 0x5A
CMD_TYPE_HEARTBEAT = 0x01
CMD_TYPE_SERVO_CTRL = 0x10
CMD_TYPE_TORQUE = 0x11
CMD_TYPE_REQUEST_STATE = 0x30
FB_TYPE_SENSOR_IMU = 0x30
FB_TYPE_RL_STATE = 0x40


def _env_int(name, default):
    raw = os.getenv(name)
    if raw is None:
        return default
    try:
        return int(raw)
    except (TypeError, ValueError):
        return default

class RobotIO:
    def __init__(self, port=None, baud_rate=115200):
        self.port = port
        self.baud_rate = baud_rate
        self.ser = None
        self.running = False
        self.thread = None
        self.last_rx_time = 0.0
        self.last_rl_state_time = 0.0
        
        # Shared State
        self.imu_data = {
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            'gyro': [0.0, 0.0, 0.0],
            'accel': [0.0, 0.0, 0.0],
        }
        self.servo_states = {} # id -> {pos, spd, load}
        self.servo_count = 0
        self.servo_ids = []
        self.lock = threading.Lock()

        self.safety_guard = ServoSafetyGuard(
            enforce_nonzero_timing=(os.getenv("WAVEGO_ENFORCE_SAFE_TIMING", "1") != "0"),
            default_time_ms=_env_int("WAVEGO_SAFE_TIME_MS", 30),
            default_speed=_env_int("WAVEGO_SAFE_SPEED", 1200),
            max_speed=_env_int("WAVEGO_SAFE_MAX_SPEED", 3400),
            max_time_ms=_env_int("WAVEGO_SAFE_MAX_TIME_MS", 2000),
            max_step_delta=_env_int("WAVEGO_MAX_STEP_DELTA", 260),
        )
        
        # Connect automatically if port is not specified
        if not self.port:
            self.auto_connect()
        else:
            self.connect(self.port)

    def auto_connect(self):
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            print("[RobotIO] No serial ports found.")
            return

        # Highest priority: explicit override from environment.
        env_port = os.getenv("WAVEGO_PORT")
        if env_port:
            self.connect(env_port)
            if self.ser is not None:
                return
            print(f"[RobotIO] WAVEGO_PORT={env_port} connect failed, fallback to auto-detect")

        def score(port_info):
            device = (port_info.device or "").lower()
            desc = (port_info.description or "").lower()
            val = 0
            if "wavego" in desc or "stm32" in desc:
                val += 100
            if "ttyacm" in device:
                val += 80
            if "ttyusb" in device:
                val += 60
            if "usb" in desc:
                val += 30
            if "com" in device:
                val += 20
            return val

        sorted_ports = sorted(ports, key=score, reverse=True)
        for p in sorted_ports:
            self.connect(p.device)
            if self.ser is not None:
                return

        all_devices = ", ".join(p.device for p in ports)
        print(f"[RobotIO] No usable serial ports. Detected: {all_devices}")

    def connect(self, port):
        try:
            self.ser = serial.Serial(port, self.baud_rate, timeout=0.01, write_timeout=0.1)
            self.port = port
            self.running = True
            self.thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.thread.start()
            self.last_rx_time = time.perf_counter()
            print(f"[RobotIO] Connected to {port}")
        except Exception as e:
            self.ser = None
            print(f"[RobotIO] Connection failed: {e}")
            self.running = False

    def close(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.ser:
            try:
                self.send_torque(False) 
                self.ser.close()
            except:
                pass
        print("[RobotIO] Closed.")

    def send_torque(self, enable):
        if not self.ser: return
        try:
            payload = bytearray([1 if enable else 0])
            self._send_packet(CMD_TYPE_TORQUE, payload)
            if not enable:
                self.safety_guard.reset_history()
        except Exception as e:
            print(f"[RobotIO] Send Torque Error: {e}")

    def send_heartbeat(self):
        if not self.ser:
            return
        try:
            self._send_packet(CMD_TYPE_HEARTBEAT, bytearray())
        except Exception as e:
            print(f"[RobotIO] Send Heartbeat Error: {e}")

    def request_state(self):
        if not self.ser:
            return
        try:
            self._send_packet(CMD_TYPE_REQUEST_STATE, bytearray())
        except Exception as e:
            print(f"[RobotIO] Request State Error: {e}")

    def send_servos(self, servo_data_list):
        if not self.ser or not servo_data_list:
            return
        try:
            with self.lock:
                feedback_pos = {
                    sid: int(state.get('pos'))
                    for sid, state in self.servo_states.items()
                    if isinstance(state, dict) and state.get('pos') is not None
                }

            safe_cmds = self.safety_guard.sanitize_batch(
                servo_data_list,
                offset_getter=cfg.get_offset,
                feedback_getter=lambda sid: feedback_pos.get(sid),
            )

            for chunk in split_servo_batches(safe_cmds):
                payload = build_servo_control_payload(chunk)
                self._send_packet(CMD_TYPE_SERVO_CTRL, payload)
        except Exception as e:
            print(f"[RobotIO] Send Servo Error: {e}")
            print(f"[RobotIO] Debug - First item: {servo_data_list[0] if servo_data_list else 'EMPTY'}")

    def get_imu(self):
        with self.lock:
            return self.imu_data.copy()

    def get_servo_states(self):
        with self.lock:
            return {sid: dict(state) for sid, state in self.servo_states.items()}

    def _send_packet(self, pkt_type, payload):
        packet = bytearray([HEAD_1, HEAD_2, pkt_type, len(payload)])
        packet.extend(payload)
        checksum = sum(packet) & 0xFF
        packet.append(checksum)
        self.ser.write(packet)

    def _rx_loop(self):
        rx_buffer = bytearray()
        while self.running:
            try:
                if self.ser.in_waiting:
                    rx_buffer.extend(self.ser.read(self.ser.in_waiting))
                
                while len(rx_buffer) >= 5:
                    if rx_buffer[0] != HEAD_1 or rx_buffer[1] != HEAD_2:
                        del rx_buffer[0]
                        continue
                    
                    pkt_len = rx_buffer[3]
                    total_len = 4 + pkt_len + 1
                    
                    if len(rx_buffer) < total_len:
                        break 
                        
                    pkt = rx_buffer[:total_len]
                    if (sum(pkt[:-1]) & 0xFF) == pkt[-1]:
                        self._process_packet(pkt[2], pkt[4:4+pkt_len])
                        del rx_buffer[:total_len]
                    else:
                        del rx_buffer[:2]
                
                time.sleep(0.001)
            except Exception as e:
                print(f"[RobotIO] RX Error: {e}")
                time.sleep(0.1)

    def _process_packet(self, pkt_type, payload):
        now = time.perf_counter()
        self.last_rx_time = now
        if pkt_type == FB_TYPE_SENSOR_IMU:
            self._parse_imu(payload)
        elif pkt_type == FB_TYPE_RL_STATE:
            if self._parse_rl_state(payload):
                self.last_rl_state_time = now

    def _parse_imu(self, payload):
        if len(payload) < 36: return
        try:
            data = struct.unpack('<9f', payload[:36])
            with self.lock:
                alpha = 0.2
                self.imu_data['accel'] = [
                    self.imu_data['accel'][i] * (1 - alpha) + data[i] * alpha for i in range(3)
                ]
                self.imu_data['gyro'] = [
                    self.imu_data['gyro'][i] * (1 - alpha) + data[3 + i] * alpha for i in range(3)
                ]
                self.imu_data['roll'] = self.imu_data['roll'] * (1 - alpha) + data[6] * alpha
                self.imu_data['pitch'] = self.imu_data['pitch'] * (1 - alpha) + data[7] * alpha
                self.imu_data['yaw'] = self.imu_data['yaw'] * (1 - alpha) + data[8] * alpha
        except: pass

    def _parse_rl_state(self, payload):
        self._parse_imu(payload)
        if len(payload) < 37:
            return False
        
        count = payload[36]
        servo_data = payload[37:]
        # Each servo is 11 bytes: ID(1), Pos(2), Spd(2), Load(2), Current(2), Voltage(1), Temp(1)
        servo_size = 11
        if len(servo_data) < count * servo_size:
            return False
        
        new_states = {}
        for i in range(count):
            base = i * servo_size
            chunk = servo_data[base : base + servo_size]
            sid, pos, spd_u, load, current, voltage, temp = struct.unpack('<BhHhHbB', chunk)
            
            # ST3215 uses sign-magnitude for velocity
            sign = -1 if (spd_u & 0x8000) else 1
            magnitude = spd_u & 0x7FFF
            real_spd = sign * magnitude
            
            new_states[sid] = {
                'pos': pos,
                'spd': real_spd,
                'load': load,
                'current': current,
                'voltage': voltage,
                'temp': temp
            }
        
        with self.lock:
            self.servo_states = new_states
            self.servo_count = len(new_states)
            self.servo_ids = sorted(new_states.keys())
        return True
