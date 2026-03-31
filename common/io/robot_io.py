import serial
import serial.tools.list_ports
import threading
import time
import struct
from ..config.robot_config import cfg

# Protocol Constants
HEAD_1 = 0xA5
HEAD_2 = 0x5A
CMD_TYPE_SERVO_CTRL = 0x10
CMD_TYPE_TORQUE = 0x11
FB_TYPE_SENSOR_IMU = 0x30
FB_TYPE_RL_STATE = 0x40

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
        
        # Connect automatically if port is not specified
        if not self.port:
            self.auto_connect()
        else:
            self.connect(self.port)

    def auto_connect(self):
        ports = list(serial.tools.list_ports.comports())
        candidates = [p.device for p in ports if "USB" in p.description or "COM" in p.device]
        if candidates:
            self.connect(candidates[-1])
        else:
            print("[RobotIO] No serial ports found.")

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
        except Exception as e:
            print(f"[RobotIO] Send Torque Error: {e}")

    def send_servos(self, servo_data_list):
        if not self.ser: return
        try:
            count = len(servo_data_list)
            payload = bytearray([count])
            for item in servo_data_list:
                # New Protocol: ID(1), Acc(1), Pos(2), Time(2), Speed(2) = 8 bytes total
                if len(item) == 5:
                    # (id, pos, time, speed, acc)
                    sid, pos, time_val, spd, acc = item
                else:
                    # Backward compatibility for (id, pos, spd, acc)
                    sid, pos, spd, acc = item
                    time_val = 0 
                
                # Debug: Check for invalid values before packing
                if not (0 <= int(sid) <= 255):
                    print(f"[DEBUG] Invalid SID: {sid} (type: {type(sid)})")
                if not (0 <= int(acc) <= 255):
                    print(f"[DEBUG] Invalid ACC: {acc} (type: {type(acc)}, item: {item})")
                
                # Format: < (little), B(ID), B(Acc), h(Pos), H(Time), H(Spd)
                u_sid = max(0, min(255, int(sid)))
                u_acc = max(0, min(255, int(acc)))
                calibrated_pos = int(pos) + cfg.get_offset(u_sid)
                i_pos = max(-32768, min(32767, calibrated_pos))
                u_time = max(0, min(65535, int(time_val)))
                u_spd = max(0, min(65535, int(spd)))
                
                # CRITICAL FIX: If Time>0 but Speed=0, servo may ignore Time and move instantly!
                # Auto-correct Speed to max (3400) to ensure Time takes effect.
                if u_time > 0 and u_spd == 0:
                    print(f"[WARN] ID {u_sid}: Time={u_time}ms but Speed=0. Speed=0 disables Time mode!")
                    print(f"[WARN] Auto-setting Speed to 3400 to allow Time-based motion.")
                    u_spd = 3400  # ST3215 max speed
                
                payload.extend(struct.pack('<B B h H H', u_sid, u_acc, i_pos, u_time, u_spd))
            self._send_packet(CMD_TYPE_SERVO_CTRL, payload)
        except Exception as e:
            print(f"[RobotIO] Send Servo Error: {e}")
            print(f"[RobotIO] Debug - First item: {servo_data_list[0] if servo_data_list else 'EMPTY'}")

    def get_imu(self):
        with self.lock:
            return self.imu_data.copy()

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
