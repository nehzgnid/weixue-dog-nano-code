import tkinter as tk
from tkinter import ttk
import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Imports from existing codebase
from kinematics_v5 import LegKinematics, L1, L2, L3, OFFSET_HIP, OFFSET_KNEE
from robot_config import cfg
from gait_generator import BezierGait
from balance_controller import BalanceController

LENGTH = cfg.LENGTH
WIDTH = cfg.WIDTH

class IKControlFrame(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.legs = {name: LegKinematics(name) for name in ["FL", "FR", "RL", "RR"]}
        self.forced_zero = False
        
        # Action State
        self.action_mode = "None"
        self.action_start_time = 0.0
        self.start_pwm = {}
        
        # PWM Keyframes
        self.POSE_SIT = {"1": 1995, "2": 1170, "3": 2122, "4": 2111, "5": 2855, "6": 1888, "7": 2000, "8": 3848, "9": 3506, "10": 2055, "11": 261, "12": 610}
        self.POSE_BOW_1 = {"1": 2534, "2": 1594, "3": 2352, "4": 1785, "5": 2535, "6": 1662, "7": 2000, "8": 3774, "9": 3506, "10": 2058, "11": 353, "12": 610}
        self.POSE_BOW_2 = {"1": 2535, "2": 1227, "3": 1980, "4": 1719, "5": 2937, "6": 2065, "7": 2000, "8": 3768, "9": 3506, "10": 2059, "11": 357, "12": 610}
        self.POSE_STAND = {} # Will be captured on start
        
        # UI Layout
        left = tk.Frame(self, width=300, bg="#eee")
        left.pack(side="left", fill="y")
        right = tk.Frame(self, bg="white")
        right.pack(side="right", fill="both", expand=True)
        
        # Controls
        self.vars = {}
        
        # Action Selector
        action_frame = tk.LabelFrame(left, text="Action / Gesture", bg="#eee", font=("Arial", 9, "bold"))
        action_frame.pack(fill="x", padx=5, pady=5)
        self.action_combo = ttk.Combobox(action_frame, values=["None", "Happy New Year 🧨"])
        self.action_combo.current(0)
        self.action_combo.pack(fill="x", padx=5, pady=5)
        self.action_combo.bind("<<ComboboxSelected>>", self.on_action_change)
        
        self.add_slider(left, "Height", -30, -250, -180)
        self.add_slider(left, "Pitch", -30, 30, 0)
        self.add_slider(left, "Roll", -30, 30, 0)
        self.add_slider(left, "Yaw", -30, 30, 0)
        self.add_slider(left, "Shift X", -50, 50, 0)
        self.add_slider(left, "Shift Y", -50, 50, 0)
        
        btn_f = tk.Frame(left); btn_f.pack(pady=10)
        tk.Button(btn_f, text="Reset Pose", command=self.reset).pack(side="left", padx=5)
        tk.Button(btn_f, text="Zero All (2048)", command=self.toggle_zero, bg="#ffcccb").pack(side="left", padx=5)
        self.btn_copy = tk.Button(btn_f, text="Copy PWM", command=self.copy_pwm, bg="#cce5ff")
        self.btn_copy.pack(side="left", padx=5)
        
        # PWM
        self.lbls = {}
        grid = tk.Frame(left); grid.pack(pady=20)
        for i, leg in enumerate(self.legs):
            tk.Label(grid, text=leg).grid(row=i, column=0)
            self.lbls[leg] = []
            for j in range(3):
                l = tk.Label(grid, text="0000", width=6, bg="white", relief="sunken")
                l.grid(row=i, column=j+1, padx=2)
                self.lbls[leg].append(l)

        # Plot
        self.fig = plt.Figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        
        # Initial Plot
        self.update_plot()

    def on_action_change(self, event):
        val = self.action_combo.get()
        if val == "None":
            self.action_mode = "None"
        elif val == "Happy New Year 🧨":
            self.action_mode = "NEW_YEAR"
            self.action_start_time = time.perf_counter()
            # Capture current state as start pose
            self.POSE_STAND = self.capture_current_pwm()
            self.animate_action()

    def capture_current_pwm(self):
        # Calculate current PWMs based on current sliders (IK)
        # We invoke a dry-run of update_plot logic essentially, or just grab the last valid PWMs
        # Simpler: calculate from current sliders
        h = self.vars["Height"].get()
        bx, by = self.vars["Shift X"].get(), self.vars["Shift Y"].get()
        # Assume 0 rotation for stand capture to be safe, or use current
        current_pwm = {}
        
        dx, dy = LENGTH/2, WIDTH/2
        origins = {"FL": [dx, dy, 0], "FR": [dx, -dy, 0], "RL": [-dx, dy, 0], "RR": [-dx, -dy, 0]}
        
        for name, origin in origins.items():
            kin = self.legs[name]
            # Standard IK
            res = kin.solve_ik(0, kin.side_sign*L1, h) # Relative to hip
            if res:
                q = kin.rad_to_pwm(*res)
                # Map to IDs
                if name == "FL": ids=[1,2,3]
                elif name == "FR": ids=[4,5,6]
                elif name == "RL": ids=[7,8,9]
                elif name == "RR": ids=[10,11,12]
                current_pwm[str(ids[0])] = q[0]
                current_pwm[str(ids[1])] = q[1]
                current_pwm[str(ids[2])] = q[2]
        return current_pwm

    def interpolate_pwm(self, pose_a, pose_b, t):
        res = {}
        for k in pose_a.keys():
            va = pose_a.get(k, 2048)
            vb = pose_b.get(k, 2048)
            # Shortest path check
            diff = vb - va
            if diff > 2048: diff -= 4096
            elif diff < -2048: diff += 4096
            
            res[k] = int(va + diff * t)
        return res

    def animate_action(self):
        if self.action_mode == "None":
            return

        current_time = time.perf_counter()
        t_total = current_time - self.action_start_time
        
        # Sequence Definition
        # 0.0 - 1.5s: Stand -> Sit
        # 1.5 - 2.0s: Sit -> Bow 1
        # 2.0 - 4.0s: Bow Loop (Bow 1 <-> Bow 2)
        # 4.0 - 5.0s: Bow -> Sit
        # 5.0 - 6.5s: Sit -> Stand
        
        loop_dur = 6.5
        t = t_total % loop_dur
        
        current_pwm = {}
        phase_txt = ""
        
        if t < 1.5:
            # Stand -> Sit
            p = t / 1.5
            p = p * p * (3 - 2 * p) # Smoothstep
            current_pwm = self.interpolate_pwm(self.POSE_STAND, self.POSE_SIT, p)
            phase_txt = "Sit Down"
        elif t < 2.0:
            # Sit -> Bow 1
            p = (t - 1.5) / 0.5
            p = p * p * (3 - 2 * p)
            current_pwm = self.interpolate_pwm(self.POSE_SIT, self.POSE_BOW_1, p)
            phase_txt = "Ready Bow"
        elif t < 4.0:
            # Bow Loop (2s duration)
            # Cycle: B1 -> B2 -> B1
            local_t = (t - 2.0)
            cycle = local_t % 1.0 # 1Hz wave
            
            if cycle < 0.5:
                p = cycle / 0.5
                p = p * p * (3 - 2 * p)
                current_pwm = self.interpolate_pwm(self.POSE_BOW_1, self.POSE_BOW_2, p)
            else:
                p = (cycle - 0.5) / 0.5
                p = p * p * (3 - 2 * p)
                current_pwm = self.interpolate_pwm(self.POSE_BOW_2, self.POSE_BOW_1, p)
            phase_txt = "Gong Xi! 🧨"
        elif t < 5.0:
            # Bow 1 -> Sit
            p = (t - 4.0) / 1.0
            p = p * p * (3 - 2 * p)
            current_pwm = self.interpolate_pwm(self.POSE_BOW_1, self.POSE_SIT, p)
            phase_txt = "Sit Back"
        else:
            # Sit -> Stand
            p = (t - 5.0) / 1.5
            p = p * p * (3 - 2 * p)
            current_pwm = self.interpolate_pwm(self.POSE_SIT, self.POSE_STAND, p)
            phase_txt = "Stand Up"

        # Trigger Plot Update with PWM Override
        self.update_plot(override_pwm=current_pwm)
        
        # Loop
        if self.action_mode != "None":
            self.after(20, self.animate_action)

    def add_slider(self, p, name, min_v, max_v, val):
        f = tk.Frame(p); f.pack(fill="x", padx=5)
        tk.Label(f, text=name, width=6).pack(side="left")
        v = tk.DoubleVar(value=val)
        self.vars[name] = v
        tk.Scale(f, from_=min_v, to=max_v, orient="horizontal", variable=v, command=lambda x: self.update_plot()).pack(fill="x")

    def reset(self):
        self.forced_zero = False
        self.vars["Height"].set(-180)
        for k in ["Pitch","Roll","Yaw","Shift X","Shift Y"]: self.vars[k].set(0)
        self.update_plot()

    def toggle_zero(self):
        self.forced_zero = not self.forced_zero
        self.update_plot()

    def copy_pwm(self):
        import json
        data = {}
        mapping = {
            "FL": [1, 2, 3], "FR": [4, 5, 6],
            "RL": [7, 8, 9], "RR": [10, 11, 12]
        }
        try:
            for leg, ids in mapping.items():
                data[ids[0]] = int(self.lbls[leg][0].cget("text").replace("!", ""))
                data[ids[1]] = int(self.lbls[leg][1].cget("text").replace("!", ""))
                data[ids[2]] = int(self.lbls[leg][2].cget("text").replace("!", ""))
            
            json_str = json.dumps(data)
            self.clipboard_clear()
            self.clipboard_append(json_str)
            self.update() 
            
            orig_text = self.btn_copy.cget("text")
            self.btn_copy.config(text="Copied!", bg="#ccffcc")
            self.after(1000, lambda: self.btn_copy.config(text=orig_text, bg="#cce5ff"))
        except Exception as e:
            print(f"Copy failed: {e}")

    def update_plot(self, gesture_time=None, override_pwm=None):
        # 1. Prepare Data
        h = self.vars["Height"].get()
        bx, by = self.vars["Shift X"].get(), self.vars["Shift Y"].get()
        r, p, y_ang = [np.radians(self.vars[k].get()) for k in ["Roll", "Pitch", "Yaw"]]
        
        # Standard Rotation Matrix
        cr, sr = np.cos(r), np.sin(r); cp, sp = np.cos(p), np.sin(p); cy, sy = np.cos(y_ang), np.sin(y_ang)
        R = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])
        
        dx, dy = LENGTH/2, WIDTH/2
        origins = {"FL": [dx, dy, 0], "FR": [dx, -dy, 0], "RL": [-dx, dy, 0], "RR": [-dx, -dy, 0]}
        
        results = {}
        global_valid = True
        
        # --- PWM Override Logic (FK) ---
        visual_z_offset = 0.0
        visual_pitch_offset = 0.0
        
        if override_pwm:
            # 1. Calculate FK for all legs relative to body
            # 2. Find lowest Z point of Rear Legs to anchor to ground
            
            # Map leg names to IDs
            leg_map = {
                "FL": [1,2,3], "FR": [4,5,6],
                "RL": [7,8,9], "RR": [10,11,12]
            }
            
            lowest_z = 0.0
            
            for name, ids in leg_map.items():
                p1 = override_pwm.get(str(ids[0]), 2048)
                p2 = override_pwm.get(str(ids[1]), 2048)
                p3 = override_pwm.get(str(ids[2]), 2048)
                
                kin = self.legs[name]
                # Convert PWM to Rad
                q1 = (p1 - 2048) / kin.rad_to_step / kin.dir_abd
                q2 = (p2 - 2048) / kin.rad_to_step / kin.dir_hip + OFFSET_HIP
                q3 = (p3 - 2048) / kin.rad_to_step / kin.dir_knee + OFFSET_KNEE
                
                # FK
                points = kin.forward_kinematics_strict(q1, q2, q3)
                foot_pos_body = points[-1] # End effector in Body Frame (relative to hip)
                
                # We need foot pos relative to BODY CENTER
                # Hip Origin + Foot Rel
                foot_pos_center = np.array(origins[name]) + foot_pos_body
                
                results[name] = {
                    "type": "fk",
                    "q": (q1, q2, q3),
                    "pts": points, # Relative to Hip
                    "foot_center": foot_pos_center # Relative to Body Center
                }
                
                # Track lowest Z of REAR legs to anchor "Sit"
                if name in ["RL", "RR"]:
                    if foot_pos_center[2] < lowest_z:
                        lowest_z = foot_pos_center[2]
            
            # Anchor Logic:
            # We want the lowest rear foot to be at Ground Level (say -180mm or whatever standard height is)
            # Default h is usually -180.
            # So visual_body_z = -180 - lowest_z_body
            visual_z_offset = -180.0 - lowest_z
            
            # Auto-centering: Keep X/Y at 0 visually during animation
            bx, by = 0, 0 
            
            # Note: We are NOT applying Pitch rotation to the body frame here
            # because the PWMs for "Sit" likely already account for the leg angles needed 
            # to support a tilted body. However, visualizing the body as FLAT (Horizontal)
            # while the legs are in "Sit" configuration might look like the robot is 
            # hovering horizontally with legs retracted.
            # 
            # To visualize the BODY TILT properly solely from leg PWMs is hard without IMU data.
            # BUT, we can heuristic: If Front Feet are much higher than Rear Feet, tilt the body.
            # Let's calculate average Front Z vs Rear Z
            fl_z = results["FL"]["foot_center"][2]
            fr_z = results["FR"]["foot_center"][2]
            rl_z = results["RL"]["foot_center"][2]
            rr_z = results["RR"]["foot_center"][2]
            
            avg_front = (fl_z + fr_z) / 2.0
            avg_rear = (rl_z + rr_z) / 2.0
            
            # Slope
            diff_z = avg_front - avg_rear
            # Angle approx sin(theta) = diff_z / LENGTH
            # If Front is HIGHER (less negative) than Rear, diff_z > 0.
            # This implies Body is Tilted UP (Pitch < 0 in our convention usually?)
            # Wait, if Body is Horizontal (0 pitch), and Front Legs are retracted (Higher Z), 
            # then plotting it horizontally is CORRECT - the robot is horizontal, legs are up.
            #
            # BUT, "Sit" usually means the Front Feet are ON THE GROUND, but the body is tilted?
            # NO, "Sit" means Rear Feet on Ground, Front Feet in Air (for Begging).
            # So the Body IS tilted up.
            # 
            # If we just plot with R=Identity, the body is flat.
            # If the user taught the robot to sit, they probably tilted the real robot.
            # The IMU would show ~45 deg pitch.
            # Since we don't have recorded IMU data, only PWM, we have to GUESS the body Pitch
            # to make the visualization look natural.
            # 
            # Heuristic: 
            # In "Sit" PWMs, the rear legs are folded.
            # If we render horizontal, it looks like a loaf.
            # Let's apply a smooth visual pitch based on the phase of animation?
            # Or just trust the FK.
            #
            # Let's stick to FK relative to a Horizontal Body for now, but Anchor Z.
            # This ensures "Center doesn't shift" (Body X/Y=0) and "Grounded" (Rear feet at -180).
            # The user can manually adjust Pitch slider if they want to see it tilted, 
            # but changing slider affects PWMs in IK mode. 
            # In PWM override mode, Sliders are ignored for calculation, so we could update them for viz?
            # 
            # Let's calculate a heuristic pitch to make it look cool:
            # If we are in NEW_YEAR mode, we know we are sitting.
            # Let's artificially set the Visual Rotation R based on time phase in animate_action?
            # No, animate_action doesn't pass pitch.
            #
            # Let's just use the Z-anchor. It's the most robust way to ensure "no drift".
            
        else:
            # Standard IK Mode
            for name, origin in origins.items():
                kin = self.legs[name]
                if self.forced_zero:
                    results[name] = {"type": "zero"}; continue
                
                foot_world = np.array([origin[0], origin[1] + kin.side_sign*L1, h])
                # IK uses Slider-defined Body Transform (R)
                hip_world = R @ np.array(origin) + np.array([bx, by, 0])
                foot_rel = R.T @ (foot_world - hip_world)
                
                res = kin.solve_ik(foot_rel[0], foot_rel[1], foot_rel[2])
                if res:
                    q1,q2,q3 = res
                    # Check validity ...
                    # (Simplified for brevity, copying essential parts)
                    pwms = kin.rad_to_pwm(q1,q2,q3) # Just for display
                    points = kin.forward_kinematics_strict(q1,q2,q3)
                    results[name] = {"type": "ik", "q": res, "pts": points, "pwm": pwms, "valid": True}
                else:
                    results[name] = {"type": "fail"}

        self.ax.clear()
        self.ax.set_box_aspect([1,1,1]) 
        self.ax.set_xlim(-200, 200); self.ax.set_ylim(-150, 150); self.ax.set_zlim(-350, 50)
        self.ax.set_xlabel('X'); self.ax.set_ylabel('Y'); self.ax.set_zlabel('Z')
        
        # Draw Body
        # If Override, Body is at (0,0, visual_z_offset)
        # We assume Body is FLAT (Identity R) for PWM replay unless we want to simulate tilt.
        # To avoid confusion, let's keep body flat but elevated/lowered.
        
        current_R = R if not override_pwm else np.eye(3) # Reset rotation for raw PWM view
        current_pos = np.array([bx, by, 0]) if not override_pwm else np.array([0, 0, visual_z_offset])
        
        body_pts = [current_R @ np.array(origins[l]) + current_pos for l in ["FL", "FR", "RR", "RL", "FL"]]
        body_pts = np.array(body_pts)
        self.ax.plot(body_pts[:,0], body_pts[:,1], body_pts[:,2], 'k-', lw=2)
        
        for name, origin in origins.items():
            if name not in results: continue
            res = results[name]
            
            if res.get("type") in ["ik", "fk"]:
                # Draw Leg
                q1, q2, q3 = res["q"]
                pts = res["pts"] # relative to hip
                
                hip_world_pos = current_R @ np.array(origin) + current_pos
                
                world_pts = []
                for pt in pts:
                    # pt is in local hip frame (aligned with body)
                    # Rotated by Body R
                    world_pts.append(hip_world_pos + current_R @ pt)
                world_pts = np.array(world_pts)
                
                col = 'b' if "L" in name else 'r'
                self.ax.plot(world_pts[:,0], world_pts[:,1], world_pts[:,2], f'{col}-o', lw=2)
                
                # PWM Labels
                if override_pwm:
                    # Update labels with actual interpolated PWM
                    # Need to map back to IDs
                    ids = [1,2,3] # dummy
                    if name=="FL": ids=[1,2,3]
                    elif name=="FR": ids=[4,5,6]
                    elif name=="RL": ids=[7,8,9]
                    elif name=="RR": ids=[10,11,12]
                    
                    self.lbls[name][0].config(text=str(override_pwm.get(str(ids[0]),0)))
                    self.lbls[name][1].config(text=str(override_pwm.get(str(ids[1]),0)))
                    self.lbls[name][2].config(text=str(override_pwm.get(str(ids[2]),0)))

        self.canvas.draw()


class TrotSimFrame(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.legs = {name: LegKinematics(name) for name in ["FL", "FR", "RL", "RR"]}
        self.gait = BezierGait()
        self.balance = BalanceController()
        
        self.phase_offsets = {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}
        self.start_time = time.perf_counter()
        self.running = False
        
        # UI Layout
        left_panel = tk.Frame(self, width=350, bg="#f0f0f0")
        left_panel.pack(side="left", fill="y")
        right_panel = tk.Frame(self, bg="white")
        right_panel.pack(side="right", fill="both", expand=True)
        
        # Controls
        self.vars = {}
        
        gait_frame = tk.LabelFrame(left_panel, text="Gait Parameters", bg="#f0f0f0", font=("Arial", 10, "bold"))
        gait_frame.pack(fill="x", padx=10, pady=5)
        self.add_slider(gait_frame, "Frequency (Hz)", 0.5, 4.0, 2.0)
        self.add_slider(gait_frame, "Step Height (mm)", 0, 80, 40.0)
        self.add_slider(gait_frame, "Step Length (mm)", -60, 60, 0.0)
        self.add_slider(gait_frame, "Body Height (mm)", -250, -100, -180.0)
        
        imu_frame = tk.LabelFrame(left_panel, text="IMU Simulation (Disturbance)", bg="#f0f0f0", font=("Arial", 10, "bold"))
        imu_frame.pack(fill="x", padx=10, pady=5)
        self.add_slider(imu_frame, "Sim Roll (deg)", -30, 30, 0.0)
        self.add_slider(imu_frame, "Sim Pitch (deg)", -30, 30, 0.0)
        
        pid_frame = tk.LabelFrame(left_panel, text="Balance PID Tuning", bg="#f0f0f0", font=("Arial", 10, "bold"))
        pid_frame.pack(fill="x", padx=10, pady=5)
        self.add_slider(pid_frame, "Kp Roll", 0.0, 5.0, 1.0)
        self.add_slider(pid_frame, "Kp Pitch", 0.0, 5.0, 0.8)
        self.add_toggle(pid_frame, "Balance ON", True)

        self.info_label = tk.Label(left_panel, text="Status: Stopped", justify="left", font=("Consolas", 9))
        self.info_label.pack(fill="x", padx=10, pady=10)

        self.fig = plt.Figure(figsize=(5, 5), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=right_panel)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        
        self.ax.set_xlim(-250, 250); self.ax.set_ylim(-200, 200); self.ax.set_zlim(-300, 100)
        
    def start_animation(self):
        self.running = True
        self.animate()
        
    def stop_animation(self):
        self.running = False

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

    def animate(self):
        if not self.running: return
        
        current_time = time.perf_counter()
        elapsed = current_time - self.start_time
        
        freq = self.vars["Frequency (Hz)"].get()
        step_h = self.vars["Step Height (mm)"].get()
        step_l = self.vars["Step Length (mm)"].get()
        body_h = self.vars["Body Height (mm)"].get()
        
        sim_roll = np.radians(self.vars["Sim Roll (deg)"].get())
        sim_pitch = np.radians(self.vars["Sim Pitch (deg)"].get())
        
        balance_on = self.vars["Balance ON"].get()
        self.balance.pid_roll.kp = self.vars["Kp Roll"].get()
        self.balance.pid_pitch.kp = self.vars["Kp Pitch"].get()
        
        bal_offsets = {"FL":0, "FR":0, "RL":0, "RR":0}
        if balance_on:
            # Fix: Handle tuple return from balance.solve
            adj_roll, adj_pitch = self.balance.solve(sim_roll, sim_pitch, 0.0, 0.0, 0.05)
            # Map PID output (angles/signals) to Z-height offsets
            # Simple geometric approximation:
            # To correct +Roll (Body tilted Right), Right legs push down (+Z), Left legs pull up (-Z)
            # To correct +Pitch (Nose Up), Front legs push down (+Z), Rear legs pull up (-Z)
            # (Signs depend on specific robot definition, assuming standard here)
            bal_offsets = {
                "FL":  adj_pitch - adj_roll,
                "FR":  adj_pitch + adj_roll,
                "RL": -adj_pitch - adj_roll,
                "RR": -adj_pitch + adj_roll
            }
            
        period = 1.0 / max(0.1, freq)
        global_phase = (elapsed % period) / period
        
        cr, sr = np.cos(sim_roll), np.sin(sim_roll)
        cp, sp = np.cos(sim_pitch), np.sin(sim_pitch)
        R = np.array([[cp, 0, sp], [sr*sp, cr, -sr*cp], [-cr*sp, sr, cr*cp]]).T 
        
        dx, dy = LENGTH/2, WIDTH/2
        hip_origins = {"FL": [dx, dy, 0], "FR": [dx, -dy, 0], "RL": [-dx, dy, 0], "RR": [-dx, -dy, 0]}
        
        self.ax.clear()
        self.ax.set_xlim(-250, 250); self.ax.set_ylim(-200, 200); self.ax.set_zlim(-300, 100)
        self.ax.set_xlabel('X'); self.ax.set_ylabel('Y'); self.ax.set_zlabel('Z')
        
        body_pts = [R @ np.array(hip_origins[l]) for l in ["FL", "FR", "RR", "RL", "FL"]]
        body_pts = np.array(body_pts)
        self.ax.plot(body_pts[:,0], body_pts[:,1], body_pts[:,2], 'k-', lw=2)
        
        info_txt = f"Phase: {global_phase:.2f}\n"
        
        for name in ["FL", "FR", "RL", "RR"]:
            kin = self.legs[name]
            phase = (global_phase + self.phase_offsets[name]) % 1.0
            x_gait, z_gait = 0.0, 0.0
            
            if phase < 0.5:
                swing_p = phase / 0.5
                x_gait, z_gait = self.gait.get_swing_pos(swing_p, step_l, step_h)
                color = 'r'
            else:
                stance_p = (phase - 0.5) / 0.5
                x_gait, z_gait = self.gait.get_stance_pos(stance_p, step_l)
                color = 'b'
            
            z_bal = bal_offsets[name]
            target_x = x_gait
            target_y = kin.side_sign * L1 
            target_z = body_h + z_gait - z_bal 
            
            res = kin.solve_ik(target_x, target_y, target_z)
            if res:
                q1, q2, q3 = res
                p0, p1, p2, p3 = kin.forward_kinematics_strict(q1, q2, q3)
                hip_loc = np.array(hip_origins[name])
                pts = np.array([R @ (hip_loc + pt) for pt in [p0, p1, p2, p3]])
                self.ax.plot(pts[:,0], pts[:,1], pts[:,2], f'{color}-o', lw=2)
                self.ax.scatter(pts[-1,0], pts[-1,1], pts[-1,2], color=color, s=20)
                info_txt += f"{name}: Z_Bal={z_bal:.1f}\n"
            else:
                info_txt += f"{name}: IK Fail\n"

        self.info_label.config(text=info_txt)
        self.canvas.draw()
        self.after(50, self.animate)

class UnifiedSimApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Weixue Dog Nano - Unified Simulation")
        self.geometry("1400x950")
        
        # Notebook (Tabs)
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill="both", expand=True)
        
        # Tabs
        self.ik_tab = IKControlFrame(self.notebook)
        self.trot_tab = TrotSimFrame(self.notebook)
        
        self.notebook.add(self.ik_tab, text="Static IK Control")
        self.notebook.add(self.trot_tab, text="Dynamic Trot Simulation")
        
        self.notebook.bind("<<NotebookTabChanged>>", self.on_tab_change)
        
    def on_tab_change(self, event):
        # Manage resources when switching
        tab_name = self.notebook.tab(self.notebook.select(), "text")
        if "Trot" in tab_name:
            self.trot_tab.start_animation()
        else:
            self.trot_tab.stop_animation()

if __name__ == "__main__":
    app = UnifiedSimApp()
    app.mainloop()
