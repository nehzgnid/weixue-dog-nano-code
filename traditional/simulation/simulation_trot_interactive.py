import tkinter as tk
import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import sys
from pathlib import Path

# Imports from existing codebase
def _ensure_repo_root_on_path():
    for parent in Path(__file__).resolve().parents:
        if (parent / "MIGRATION_PLAN_SCHEME_B_EXECUTION.md").exists():
            if str(parent) not in sys.path:
                sys.path.insert(0, str(parent))
            break


_ensure_repo_root_on_path()

from common.motion.kinematics import LegKinematics, L1
from common.config.robot_config import cfg
from common.motion.gait_generator import BezierGait
from common.motion.balance_controller import BalanceController

LENGTH = cfg.LENGTH
WIDTH = cfg.WIDTH

class TrotSimApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Trot Gait & Balance Simulation")
        self.geometry("1400x900")
        
        # --- Core Logic Modules ---
        self.legs = {name: LegKinematics(name) for name in ["FL", "FR", "RL", "RR"]}
        self.gait = BezierGait()
        self.balance = BalanceController()
        
        # Trot Phase Offsets
        self.phase_offsets = {
            "FL": 0.0, "RR": 0.0,
            "FR": 0.5, "RL": 0.5
        }
        
        # Simulation State
        self.start_time = time.perf_counter()
        self.running = True
        self.mode = "TROT"  # "TROT" or "NEW_YEAR"
        self.gesture_start_time = 0.0
        
        # --- UI Layout ---
        left_panel = tk.Frame(self, width=350, bg="#f0f0f0")
        left_panel.pack(side="left", fill="y")
        
        right_panel = tk.Frame(self, bg="white")
        right_panel.pack(side="right", fill="both", expand=True)
        
        # --- Controls ---
        self.vars = {}
        
        # Gait Parameters
        gait_frame = tk.LabelFrame(left_panel, text="Gait Parameters", bg="#f0f0f0", font=("Arial", 10, "bold"))
        gait_frame.pack(fill="x", padx=10, pady=5)
        self.add_slider(gait_frame, "Frequency (Hz)", 0.5, 4.0, 2.0)
        self.add_slider(gait_frame, "Step Height (mm)", 0, 80, 40.0)
        self.add_slider(gait_frame, "Step Length (mm)", -60, 60, 0.0)
        self.add_slider(gait_frame, "Body Height (mm)", -250, -100, -180.0)
        
        # IMU Simulation (Inputs to Balance Controller)
        imu_frame = tk.LabelFrame(left_panel, text="IMU Simulation (Disturbance)", bg="#f0f0f0", font=("Arial", 10, "bold"))
        imu_frame.pack(fill="x", padx=10, pady=5)
        self.add_slider(imu_frame, "Sim Roll (deg)", -30, 30, 0.0)
        self.add_slider(imu_frame, "Sim Pitch (deg)", -30, 30, 0.0)
        
        # Balance Controller Tuning
        pid_frame = tk.LabelFrame(left_panel, text="Balance PID Tuning", bg="#f0f0f0", font=("Arial", 10, "bold"))
        pid_frame.pack(fill="x", padx=10, pady=5)
        self.add_slider(pid_frame, "Kp Roll", 0.0, 5.0, 1.0)
        self.add_slider(pid_frame, "Kp Pitch", 0.0, 5.0, 0.8)
        self.add_toggle(pid_frame, "Balance ON", True)

        # Actions
        action_frame = tk.LabelFrame(left_panel, text="Actions", bg="#f0f0f0", font=("Arial", 10, "bold"))
        action_frame.pack(fill="x", padx=10, pady=5)
        tk.Button(action_frame, text="🧨 拜年 (Happy New Year)", command=self.start_new_year, bg="#ffcccc", font=("Arial", 10, "bold")).pack(fill="x", padx=5, pady=5)

        # Info Display
        self.info_label = tk.Label(left_panel, text="Status: Running", justify="left", font=("Consolas", 9))
        self.info_label.pack(fill="x", padx=10, pady=10)

        # --- Matplotlib Plot ---
        self.fig = plt.Figure(figsize=(5, 5), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=right_panel)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        
        # Initial Plot Setup
        self.ax.set_xlim(-250, 250)
        self.ax.set_ylim(-200, 200)
        self.ax.set_zlim(-300, 100)
        self.ax.set_xlabel('X (Front)')
        self.ax.set_ylabel('Y (Left)')
        self.ax.set_zlabel('Z (Up)')
        
        # Start Animation Loop
        self.animate()

    def add_slider(self, parent, name, min_val, max_val, default):
        f = tk.Frame(parent, bg="#f0f0f0")
        f.pack(fill="x", pady=2)
        tk.Label(f, text=name, width=15, anchor="w", bg="#f0f0f0").pack(side="left")
        v = tk.DoubleVar(value=default)
        self.vars[name] = v
        tk.Scale(f, from_=min_val, to=max_val, orient="horizontal", variable=v, resolution=0.1, length=150).pack(side="right", expand=True, fill="x")

    def add_toggle(self, parent, name, default):
        v = tk.BooleanVar(value=default)
        self.vars[name] = v
        tk.Checkbutton(parent, text=name, variable=v, bg="#f0f0f0").pack(pady=5)

    def start_new_year(self):
        if self.mode == "TROT":
            self.mode = "NEW_YEAR"
            self.gesture_start_time = time.perf_counter()
            self.info_label.config(text="Status: Happy New Year! 🧨")

    def animate(self):
        if not self.running: return
        
        # 1. Get Time & Parameters
        current_time = time.perf_counter()
        elapsed = current_time - self.start_time
        
        # Override for Gesture
        if self.mode == "NEW_YEAR":
            gesture_time = current_time - self.gesture_start_time
            if gesture_time > 5.0:
                self.mode = "TROT"
                gesture_time = 0
            
            # Interpolate Pitch for Rearing Up (0 to -45 deg)
            # 0-1s: Sit (Pitch down/up depending on definition, usually Pitch < 0 is nose up in some frames, let's test)
            # Actually in this sim: sim_pitch is just rotation.
            # Let's say we want Body to pitch UP (Nose Up).
            # If standard is nose forward.
            
            # Phase 1: Sit/Rear (0-1.5s)
            if gesture_time < 1.5:
                p = gesture_time / 1.5
                # Smoothstep
                p = p * p * (3 - 2 * p)
                target_pitch_deg = -45.0 * p # Assume - is nose up
                target_body_h = -180 + (40 * p) # Lift body slightly
            # Phase 2: Bow/Beg (1.5-3.5s)
            elif gesture_time < 3.5:
                target_pitch_deg = -45.0
                target_body_h = -140
            # Phase 3: Return (3.5-5.0s)
            else:
                p = (gesture_time - 3.5) / 1.5
                p = 1 - p # Go back
                p = p * p * (3 - 2 * p)
                target_pitch_deg = -45.0 * p
                target_body_h = -180 + (40 * p)
                
            # Set UI var to visualize the pitch change
            self.vars["Sim Pitch (deg)"].set(target_pitch_deg)
            self.vars["Body Height (mm)"].set(target_body_h)
            
            # Disable balance during gesture
            self.vars["Balance ON"].set(False)

        freq = self.vars["Frequency (Hz)"].get()
        step_h = self.vars["Step Height (mm)"].get()
        step_l = self.vars["Step Length (mm)"].get()
        body_h = self.vars["Body Height (mm)"].get()
        
        sim_roll = np.radians(self.vars["Sim Roll (deg)"].get())
        sim_pitch = np.radians(self.vars["Sim Pitch (deg)"].get())
        
        balance_on = self.vars["Balance ON"].get()
        kp_roll = self.vars["Kp Roll"].get()
        kp_pitch = self.vars["Kp Pitch"].get()
        
        # Update PID gains dynamically
        self.balance.pid_roll.kp = kp_roll
        self.balance.pid_pitch.kp = kp_pitch
        
        # 2. Calculate Balance Offsets
        bal_offsets = {"FL":0, "FR":0, "RL":0, "RR":0}
        if balance_on:
            bal_offsets = self.balance.solve(
                current_roll=sim_roll,
                current_pitch=sim_pitch,
                target_roll=0.0,
                target_pitch=0.0,
                dt=0.05 # Approx refresh rate
            )
            
        # 3. Calculate Leg Positions
        period = 1.0 / max(0.1, freq)
        global_phase = (elapsed % period) / period
        
        # Visualization Data
        legs_data = {}
        
        # Body Rotation Matrix (simulated disturbance)
        # We rotate the "Body" frame to show the tilt
        cr, sr = np.cos(sim_roll), np.sin(sim_roll)
        cp, sp = np.cos(sim_pitch), np.sin(sim_pitch)
        # Simple R_roll * R_pitch (Yaw=0)
        R = np.array([
            [cp, 0, sp],
            [sr*sp, cr, -sr*cp],
            [-cr*sp, sr, cr*cp]
        ]).T # Transpose because we usually rotate vector FROM body TO world? 
             # Wait, if Roll is + (Right Tilt), Body X-axis matches World X. Body Y-axis points down-ish.
             # Here we want to plot the robot IN World Frame.
             # Hip_World = R @ Hip_Local + Body_Pos
        
        dx, dy = LENGTH/2, WIDTH/2
        hip_origins = {
            "FL": [dx, dy, 0], "FR": [dx, -dy, 0],
            "RL": [-dx, dy, 0], "RR": [-dx, -dy, 0]
        }
        
        info_txt = f"Phase: {global_phase:.2f}\n"
        
        self.ax.clear()
        self.ax.set_xlim(-250, 250); self.ax.set_ylim(-200, 200); self.ax.set_zlim(-300, 100)
        self.ax.set_xlabel('X'); self.ax.set_ylabel('Y'); self.ax.set_zlabel('Z')
        
        # Draw Body Plane
        body_pts = [R @ np.array(hip_origins[l]) for l in ["FL", "FR", "RR", "RL", "FL"]]
        body_pts = np.array(body_pts)
        self.ax.plot(body_pts[:,0], body_pts[:,1], body_pts[:,2], 'k-', lw=2)
        
        for name in ["FL", "FR", "RL", "RR"]:
            kin = self.legs[name]
            
            # --- Gait Logic ---
            if self.mode == "NEW_YEAR":
                # Custom IK Targets for Gesture
                gesture_time = current_time - self.gesture_start_time
                
                # Default Stance
                x_gait, y_gait, z_gait = 0.0, 0.0, 0.0
                
                # Rear Legs: Anchored/Sitting
                if name in ["RL", "RR"]:
                    z_gait = 0.0 # Stay relative to body height (which is moving)
                    x_gait = -20.0 # Shift legs back slightly
                
                # Front Legs: The "Begging" Motion
                if name in ["FL", "FR"]:
                    # Phase 1: Lift
                    if gesture_time < 1.5:
                        p = gesture_time / 1.5
                        p = p * p * (3 - 2 * p)
                        z_gait = 80.0 * p # Lift feet UP (positive is up? Standard Z is usually negative down)
                        # Wait, in this sim target_z = body_h + z_gait. 
                        # Body H is -180. Ground is at Z=-180 approx.
                        # If we want feet to lift, we want Z to be LESS negative (closer to 0).
                        # So z_gait should be positive.
                        x_gait = 50.0 * p # Paws forward
                        
                    # Phase 2: Wave/Bow
                    elif gesture_time < 3.5:
                        # Oscillation 2Hz
                        wave = np.sin((gesture_time - 1.5) * 4 * np.pi) 
                        z_gait = 80.0 + 30.0 * wave
                        x_gait = 50.0
                        
                    # Phase 3: Return
                    else:
                        p = (gesture_time - 3.5) / 1.5
                        p = 1 - p
                        p = p * p * (3 - 2 * p)
                        z_gait = 80.0 * p
                        x_gait = 50.0 * p
                
                color = 'm' # Magenta for Gesture

            else:
                # Standard Trot
                phase = (global_phase + self.phase_offsets[name]) % 1.0
                
                x_gait, z_gait = 0.0, 0.0
                is_swing = phase < 0.5
                
                if is_swing:
                    swing_p = phase / 0.5
                    x_gait, z_gait = self.gait.get_swing_pos(swing_p, step_l, step_h)
                    color = 'r' # Red for Swing
                else:
                    stance_p = (phase - 0.5) / 0.5
                    x_gait, z_gait = self.gait.get_stance_pos(stance_p, step_l)
                    color = 'b' # Blue for Stance
            
            # --- Combine ---
            # Target relative to HIP
            # Note: balance offset is added to Z. 
            # If Balance logic says "Push Down" (+Offset), Leg should extend -> Z becomes MORE Negative.
            # My balance_controller returns + for push. 
            # In Hip Frame, Foot is at -180. To push, we want -190. 
            # So Target Z = BodyH + GaitZ - BalOffset
            
            z_bal = bal_offsets[name]
            target_x = x_gait
            target_y = kin.side_sign * L1 # Standard Y offset
            target_z = body_h + z_gait - z_bal # Note the minus sign
            
            # Solve IK
            # IK assumes Hip is at (0,0,0) and rotation aligned with world (effectively).
            # But wait, if Body is tilted (Sim Roll), the "Down" vector for gravity is distinct from Body "Down".
            # The Balance Controller is trying to keep Body Upright.
            # But in this simulation, we are FORCE tilting the body with sliders to see if legs react.
            # So we should calculate IK in the LEG's local frame.
            
            res = kin.solve_ik(target_x, target_y, target_z)
            
            if res:
                q1, q2, q3 = res
                
                # Forward Kinematics for Plotting (in Body Frame)
                p0, p1, p2, p3 = kin.forward_kinematics_strict(q1, q2, q3)
                
                # Transform to World Frame for Plotting
                # P_world = R @ P_body + Hip_Origin_World
                # Actually Hip_Origin is Body Frame relative to Center.
                # So P_world = R @ (Hip_Local + P_leg_local)
                
                hip_loc = np.array(hip_origins[name])
                pts = []
                for pt in [p0, p1, p2, p3]:
                    pts.append(R @ (hip_loc + pt)) 
                pts = np.array(pts)
                
                self.ax.plot(pts[:,0], pts[:,1], pts[:,2], f'{color}-o', lw=2)
                
                # Draw Foot Trajectory dot
                self.ax.scatter(pts[-1,0], pts[-1,1], pts[-1,2], color=color, s=20)
                
                info_txt += f"{name}: Z_Bal={z_bal:.1f}\n"
            else:
                info_txt += f"{name}: IK Fail\n"

        self.info_label.config(text=info_txt)
        self.canvas.draw()
        
        # Loop 20Hz
        self.after(50, self.animate)

if __name__ == "__main__":
    app = TrotSimApp()
    app.mainloop()
