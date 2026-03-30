import time

# Modules
from robot_config import cfg
from kinematics_v5 import LegKinematics
from robot_io import RobotIO
from gait_generator import BezierGait
from balance_controller import BalanceController

# === Configuration ===
FREQ = 2.0             # Trot Frequency (Hz)
DT = 0.02              # Loop period (50Hz)
BODY_HEIGHT = -180.0   # Standing height
STEP_HEIGHT = 40.0     # Lift height
STEP_LENGTH = 0.0      # In place for now, can be increased to move

# Center of Gravity Shift (Static)
COG_X_SHIFT = 40.0 
COG_Y_SHIFT = 5.0

class TrotApp:
    def __init__(self):
        self.io = RobotIO() # Auto-connects
        self.gait = BezierGait()
        self.balance = BalanceController()
        
        self.legs = {
            "FL": LegKinematics("FL"),
            "FR": LegKinematics("FR"),
            "RL": LegKinematics("RL"),
            "RR": LegKinematics("RR")
        }
        
        self.leg_ids = {
            "FL": [1, 2, 3],
            "FR": [4, 5, 6],
            "RL": [7, 8, 9],
            "RR": [10, 11, 12]
        }
        
        # Trot Phase Offsets
        # Pair 1: FL & RR (0.0)
        # Pair 2: FR & RL (0.5)
        self.phase_offsets = {
            "FL": 0.0, "RR": 0.0,
            "FR": 0.5, "RL": 0.5
        }
        
        self.running = False
        
    def run(self):
        if not self.io.ser:
            print("Error: Serial not connected.")
            return

        print("=== Advanced Trot Controller ===")
        print("Features: Bezier Gait, IMU Balance (PID)")
        print("Keep the robot suspended first!")
        input("Press Enter to Start...")
        
        self.running = True
        self.io.send_torque(True)
        
        # Soft Start
        print("Soft starting...")
        start_time = time.perf_counter()
        
        try:
            while self.running:
                loop_start = time.perf_counter()
                elapsed = loop_start - start_time
                
                # 1. Update State
                imu = self.io.get_imu() # {'roll':..., 'pitch':..., 'yaw':...}
                
                # 2. Balance Correction
                # Note: imu data might be noisy or zero if not connected
                # PID update
                bal_offsets = self.balance.solve(
                    current_roll=imu['roll'], 
                    current_pitch=imu['pitch'],
                    target_roll=0.0,
                    target_pitch=0.0,
                    dt=DT
                )
                
                # 3. Gait Generation
                period = 1.0 / FREQ
                global_phase = (loop_start % period) / period
                
                # Ramp up step height for 3 seconds
                current_step_height = min(STEP_HEIGHT, STEP_HEIGHT * (elapsed / 3.0))
                
                servo_cmds = []
                
                for name in self.legs:
                    kin = self.legs[name]
                    
                    # Local Phase
                    phase = (global_phase + self.phase_offsets[name]) % 1.0
                    
                    # Trajectory
                    x_gait, z_gait = 0.0, 0.0
                    
                    if phase < 0.5:
                        # Swing (0 -> 1 normalized for swing)
                        swing_p = phase / 0.5
                        x_gait, z_gait = self.gait.get_swing_pos(swing_p, STEP_LENGTH, current_step_height)
                    else:
                        # Stance
                        stance_p = (phase - 0.5) / 0.5
                        x_gait, z_gait = self.gait.get_stance_pos(stance_p, STEP_LENGTH)
                        
                    # Combine with Balance and COG
                    # Coordinate System:
                    # X: Forward
                    # Y: Left
                    # Z: Up (but BODY_HEIGHT is -180, so we are working in hip frame where foot is at -180)
                    
                    # Final Target Position relative to Hip
                    # Default Stance: (0, +/-L1, BODY_HEIGHT)
                    
                    target_x = x_gait + COG_X_SHIFT
                    target_y = kin.side_sign * cfg.L1 + COG_Y_SHIFT # Keep Y fixed mostly
                    
                    # Z Balance Correction
                    # Positive balance offset means "Extend Leg" => Foot goes lower (more negative Z?)
                    # Wait, IK Z is coordinate of foot. Hip is 0. Foot is at -180.
                    # If we want to Extend Leg, Z should be MORE Negative (e.g. -190).
                    # My PID logic in balance_controller.py returned offsets where Positive means "Extend Leg".
                    # Let's adjust:
                    # offset > 0 => Extend => Z decreases (becomes more negative).
                    
                    z_bal = -bal_offsets[name]
                    target_z = BODY_HEIGHT + z_gait + z_bal
                    
                    # IK Solve
                    res = kin.solve_ik(target_x, target_y, target_z)
                    
                    if res:
                        q1, q2, q3 = res
                        p1, p2, p3 = kin.rad_to_pwm(q1, q2, q3)
                        
                        ids = self.leg_ids[name]
                        # ID, Pos, Spd, Acc
                        # Speed 0 = Max
                        servo_cmds.append((ids[0], p1, 0, 0))
                        servo_cmds.append((ids[1], p2, 0, 0))
                        servo_cmds.append((ids[2], p3, 0, 0))
                
                # 4. Send Commands
                self.io.send_servos(servo_cmds)
                
                # 5. Loop Timing
                loop_end = time.perf_counter()
                loop_dur = loop_end - loop_start
                sleep_time = max(0.0, DT - loop_dur)
                time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            self.io.send_torque(False)
            self.io.close()
            print("Done.")

if __name__ == "__main__":
    app = TrotApp()
    app.run()
