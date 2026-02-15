import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import threading
import time
import math
import struct
import json
from datetime import datetime

# Import shared modules
from robot_config import cfg
from kinematics_v5 import LegKinematics, OFFSET_HIP, OFFSET_KNEE
from robot_io import RobotIO
from gait_generator import BezierGait
from balance_controller import BalanceController

# --- Constants ---
BAUD_RATE = 115200
DISPLAY_SERVO_COUNT = 12

class SharedContext:
    """Holds shared resources like serial connection and state."""
    def __init__(self):
        self.io = RobotIO() 
        self.running = True
        self.mode = "RELAX" # Default to RELAX to prevent jerk on startup
        self.log_callback = None
    
    def log(self, msg):
        if self.log_callback:
            self.log_callback(msg)
        else:
            print(f"[Context] {msg}")

class ManualControlFrame(tk.Frame):
    def __init__(self, parent, context):
        super().__init__(parent, bg="#2b2b2b")
        self.ctx = context
        self.sliders = {}
        self.fb_labels = {}
        self.pos_vars = {}
        self.servo_states = {i: {'pos': 2048, 'speed': 0, 'load': 0} for i in range(1, 32)}
        
        # Global Parameters for Native Control
        self.global_vars = {
            "Time (ms)": tk.IntVar(value=0),
            "Speed Limit": tk.IntVar(value=0),
            "Acceleration": tk.IntVar(value=0),
            "Use Native Timing": tk.BooleanVar(value=True)
        }
        
        self.setup_ui()
        self.after(50, self.update_ui_loop)
        self.after(20, self.control_loop)

    def setup_ui(self):
        # Top Bar (Sync/Relax)
        top = tk.Frame(self, bg="#3c3f41", pady=5)
        top.pack(fill="x")
        
        # Row 1: Commands
        btn_f = tk.Frame(top, bg="#3c3f41")
        btn_f.pack(fill="x")
        tk.Button(btn_f, text="Enable Manual", command=self.enable_manual, bg="#4CAF50", fg="white").pack(side="left", padx=5)
        tk.Button(btn_f, text="Relax All", command=self.relax_all, bg="#FF9800").pack(side="left", padx=5)
        tk.Button(btn_f, text="Sync From Hardware", command=self.sync_sliders, bg="#2196F3", fg="white").pack(side="left", padx=5)
        tk.Button(btn_f, text="Lie Down", command=self.lie_down, bg="#607D8B", fg="white").pack(side="left", padx=5)
        tk.Button(btn_f, text="Stand Up", command=self.stand_up, bg="#8BC34A", fg="white").pack(side="left", padx=5)
        tk.Button(btn_f, text="Paste & Move", command=self.paste_and_move, bg="#9C27B0", fg="white").pack(side="left", padx=5)
        
        # Row 2: Native Control Params
        param_f = tk.Frame(top, bg="#3c3f41", pady=5)
        param_f.pack(fill="x")
        
        tk.Checkbutton(param_f, text="NATIVE MODE", variable=self.global_vars["Use Native Timing"], 
                       bg="#3c3f41", fg="#00ff00", selectcolor="#2b2b2b", font=("Arial", 10, "bold")).pack(side="left", padx=10)
        
        for name in ["Time (ms)", "Speed Limit", "Acceleration"]:
            tk.Label(param_f, text=name+":", bg="#3c3f41", fg="#eee").pack(side="left", padx=2)
            tk.Entry(param_f, textvariable=self.global_vars[name], width=6, bg="#1e1e1e", fg="#00ff00").pack(side="left", padx=5)
        
        # Scrollable Area
        canvas = tk.Canvas(self, bg="#2b2b2b", highlightthickness=0)
        scrollbar = ttk.Scrollbar(self, orient="vertical", command=canvas.yview)
        self.scroll_frame = tk.Frame(canvas, bg="#2b2b2b")
        
        self.scroll_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=self.scroll_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        for i in range(1, DISPLAY_SERVO_COUNT + 1):
            self.create_row(self.scroll_frame, i)

    def create_row(self, parent, sid):
        row = tk.Frame(parent, bg="#323232", pady=5, padx=10, relief="groove", bd=1)
        row.pack(fill="x", pady=2)
        
        color = "#ffeb3b" if sid <= 6 else "#00e676" 
        tk.Label(row, text=f"ID {sid:02d}", font=("Consolas", 12, "bold"), fg=color, bg="#323232", width=6).pack(side="left")
        
        var = tk.IntVar(value=2048)
        self.pos_vars[sid] = var
        
        slider = tk.Scale(row, from_=0, to=4095, orient="horizontal", variable=var,
                         length=400, bg="#323232", fg="#eee", highlightthickness=0,
                         troughcolor="#1e1e1e", activebackground="#4CAF50")
        slider.pack(side="left", padx=10, fill="x", expand=True)
        self.sliders[sid] = slider
        
        entry = tk.Entry(row, width=6, font=("Consolas", 12), bg="#1e1e1e", fg="#4CAF50", insertbackground="white")
        entry.insert(0, "2048")
        entry.pack(side="left", padx=5)
        entry.bind("<Return>", lambda e, s=sid, en=entry: self.on_entry_set(s, en))
        
        # Feedback
        l_pos = tk.Label(row, text="----", font=("Consolas", 10), fg="white", bg="#323232", width=5)
        l_pos.pack(side="right", padx=2)
        self.fb_labels[sid] = l_pos

    def on_entry_set(self, sid, entry):
        try:
            val = int(entry.get())
            val = max(0, min(4095, val))
            self.pos_vars[sid].set(val)
        except: pass

    def enable_manual(self):
        self.ctx.mode = "MANUAL"
        self.ctx.io.send_torque(True)
        self.ctx.log("Mode: MANUAL (Torque ON)")

    def sync_sliders(self):
        """Alings sliders to actual hardware positions to prevent sudden jumps."""
        states = self.ctx.io.get_servo_states()
        count = 0
        for sid, data in states.items():
            if sid in self.pos_vars:
                self.pos_vars[sid].set(data['pos'])
                count += 1
        if count > 0:
            self.ctx.log(f"Synced {count} sliders from hardware.")
        else:
            self.ctx.log("Sync failed: No servo feedback data received yet.")

    def relax_all(self):
        self.ctx.mode = "RELAX"
        self.ctx.io.send_torque(False)
        self.ctx.log("Mode: RELAX")

    def perform_synchronized_move(self, targets, duration_sec=2.0, acc=0):
        """
        Supports both Software Trajectory Interpolation and Native Hardware Timing.
        """
        use_native = self.global_vars["Use Native Timing"].get()
        
        if use_native:
            self.ctx.log(f"Executing NATIVE move: {duration_sec}s")
            # 1. Map duration to ms
            time_ms = int(duration_sec * 1000)
            # 2. Get global acc/speed
            # Note: In Native Mode, 'speed' in command usually means Speed Limit (Reg 46).
            # 'time' means Goal Time (Reg 44).
            speed_limit = self.global_vars["Speed Limit"].get()
            acc_val = self.global_vars["Acceleration"].get()
            
            cmds = []
            for sid_str, target in targets.items():
                sid = int(sid_str)
                # Protocol: (id, pos, time, speed, acc)
                cmds.append((sid, target, time_ms, speed_limit, acc_val))
                if sid in self.pos_vars:
                    self.pos_vars[sid].set(target)
            
            # Lockout manual loop briefly to allow hardware to take over
            self.manual_lockout_until = time.perf_counter() + duration_sec + 0.1
            self.ctx.io.send_servos(cmds)
            return

        # --- SOFTWARE INTERPOLATION FALLBACK ---
        # 1. Capture Start Positions
        start_positions = {}
        for sid_str in targets:
            sid = int(sid_str)
            if sid in self.pos_vars:
                start_positions[sid] = self.pos_vars[sid].get()
            else:
                start_positions[sid] = 2048 # Default fallback
        
        # 2. Define Trajectory Function (Ease-in-out)
        def _trajectory_thread():
            steps = int(duration_sec * 50) # 50Hz updates
            if steps < 1: steps = 1
            
            self.ctx.log(f"Starting Software Interpolation: {duration_sec}s, {steps} steps")
            
            # Lockout manual control
            self.manual_lockout_until = time.perf_counter() + duration_sec + 1.0
            
            for step in range(1, steps + 1):
                t = step / steps
                # Smoothstep easing: 3t^2 - 2t^3
                # t_eased = t * t * (3 - 2 * t) 
                # Or simple linear for strict sync:
                t_eased = t 
                
                cmds = []
                for sid_str, target in targets.items():
                    sid = int(sid_str)
                    start = start_positions[sid]
                    
                    # Interpolate
                    current_target = int(start + (target - start) * t_eased)
                    
                    # Send with Time=0, Speed=0, Acc=0 (Max response)
                    cmds.append((sid, current_target, 0, 0, 0))
                
                self.ctx.io.send_servos(cmds)
                time.sleep(0.02) # 20ms
            
            # Final Ensure
            cmds = []
            for sid_str, target in targets.items():
                sid = int(sid_str)
                cmds.append((sid, target, 0, 0, 0))
                if sid in self.pos_vars:
                    self.pos_vars[sid].set(target)
            self.ctx.io.send_servos(cmds)
            self.ctx.log("Software Move Complete")
            
        threading.Thread(target=_trajectory_thread, daemon=True).start()

    def lie_down(self):
        targets = {
            "1": 2048, "2": 1073, "3": 3379,
            "4": 2057, "5": 2964, "6": 785,
            "7": 2055, "8": 1077, "9": 3371,
            "10": 2043, "11": 3011, "12": 765
        }
        self.ctx.mode = "MANUAL"
        self.ctx.io.send_torque(True)
        self.ctx.log("Executing Software Smooth Lie Down")
        self.perform_synchronized_move(targets, duration_sec=2.0)

    def stand_up(self):
        targets = {
            "1": 2048, "2": 1527, "3": 2273, 
            "4": 2048, "5": 2568, "6": 1822, 
            "7": 2048, "8": 1527, "9": 2273, 
            "10": 2048, "11": 2568, "12": 1822
        }
        self.ctx.mode = "MANUAL" # Ensure torque is enabled
        self.ctx.io.send_torque(True)
        self.ctx.log("Executing Software Smooth Stand Up")
        self.perform_synchronized_move(targets, duration_sec=2.0)

    def paste_and_move(self):
        try:
            content = self.clipboard_get()
            targets = json.loads(content)
            self.ctx.mode = "MANUAL"
            self.ctx.io.send_torque(True)
            count = 0
            for str_id, pos in targets.items():
                i = int(str_id)
                if 1 <= i <= DISPLAY_SERVO_COUNT:
                    if i in self.pos_vars:
                        self.pos_vars[i].set(pos)
                        count += 1
            self.ctx.log(f"Pasted {count} positions. Hardware auto-moving.")
        except Exception as e:
            self.ctx.log(f"Paste Error: {e}")

    def control_loop(self):
        # Only send servo commands periodically in MANUAL mode
        # BUT NOT if we are in a transition (handled by hardware)
        # In current design, 'perform_synchronized_move' sends a ONE-SHOT command.
        # But 'control_loop' runs every 20ms.
        # IF 'control_loop' sends position commands with speed=0 (Max), it will OVERWRITE the smooth trajectory!
        
        # FIX: We need a flag to pause manual control loop during smooth transitions.
        # Or, simpler: perform_synchronized_move sets mode to something else briefly?
        # The user code sets self.ctx.mode = "MANUAL".
        # And control_loop checks self.ctx.mode == "MANUAL".
        
        # Issue: perform_synchronized_move updates self.pos_vars immediately.
        # control_loop sees new pos_vars, and sends them with speed=0 (Max).
        # This CANCELS the smooth move and snaps to target immediately!
        
        # Solution: Add a timestamp check or a "transient" mode.
        # Let's change mode to "TRANSITION" during the move, then back to "MANUAL"?
        # But we don't know when the move ends exactly (open loop).
        
        # Better: control_loop should NOT send commands continuously if nothing changed?
        # No, for MANUAL sliders we want continuous updates or on-change.
        # But here sliders are updated by code.
        
        # Proposed Fix:
        # In perform_synchronized_move, set a "lockout" time.
        
        if self.ctx.mode == "MANUAL" and self.ctx.io.ser:
            # Check if we are in a lockout period (smooth move in progress)
            if hasattr(self, 'manual_lockout_until') and time.perf_counter() < self.manual_lockout_until:
                # Do not send override commands
                pass
            else:
                data = []
                # Use current global params for manual sliders if NATIVE MODE is on
                # But usually for sliders we want instant response (Time=0, Spd=0, Acc=0)
                use_native = self.global_vars["Use Native Timing"].get()
                
                t_val = self.global_vars["Time (ms)"].get() if use_native else 0
                s_val = self.global_vars["Speed Limit"].get() if use_native else 0
                a_val = self.global_vars["Acceleration"].get() if use_native else 0
                
                for i in range(1, DISPLAY_SERVO_COUNT + 1):
                    data.append((i, self.pos_vars[i].get(), t_val, s_val, a_val)) 
                self.ctx.io.send_servos(data)
        
        self.after(20, self.control_loop)

    def update_ui_loop(self):
        # Update Feedback Labels from current IO state
        states = self.ctx.io.get_servo_states()
        for sid, data in states.items():
            if sid in self.fb_labels:
                pos = data.get('pos', 0)
                self.fb_labels[sid].config(text=f"{pos:04d}")
                # Colorize based on deviation from target
                target = self.pos_vars[sid].get()
                error = abs(target - pos)
                if error > 50:
                    self.fb_labels[sid].config(fg="#ff5252") # Red if laggy
                else:
                    self.fb_labels[sid].config(fg="#00e676") # Green if accurate
        
        self.after(100, self.update_ui_loop)

class TestLabFrame(tk.Frame):
    def __init__(self, parent, context):
        super().__init__(parent, bg="#f5f5f5")
        self.ctx = context
        self.running_test = False
        self.start_time = 0
        
        self.legs = {name: LegKinematics(name) for name in ["FL", "FR", "RL", "RR"]}
        self.leg_ids = {
            "FL": [1, 2, 3], "FR": [4, 5, 6],
            "RL": [7, 8, 9], "RR": [10, 11, 12]
        }
        self.gait = BezierGait()
        self.vars = {}
        
        self.setup_ui()
        
    def setup_ui(self):
        lbl = tk.Label(self, text="Experimental Testing Lab", font=("Arial", 14, "bold"), bg="#f5f5f5", pady=10)
        lbl.pack(fill="x")
        
        # Single Leg Test Panel
        sl_frame = tk.LabelFrame(self, text="Single Leg Gait Test", font=("Arial", 11, "bold"), bg="#f5f5f5")
        sl_frame.pack(fill="x", padx=20, pady=10)
        
        # Leg Selector
        sel_frame = tk.Frame(sl_frame, bg="#f5f5f5")
        sel_frame.pack(fill="x", padx=10, pady=5)
        tk.Label(sel_frame, text="Select Target Leg:", bg="#f5f5f5").pack(side="left")
        self.target_leg = tk.StringVar(value="FL")
        for l in ["FL", "FR", "RL", "RR"]:
            ttk.Radiobutton(sel_frame, text=l, variable=self.target_leg, value=l).pack(side="left", padx=5)
            
        # Params for Single Leg
        self.add_slider(sl_frame, "Test Freq (Hz)", 0.2, 2.0, 0.5)
        self.add_slider(sl_frame, "Test Height (mm)", 0, 100, 30)
        self.add_slider(sl_frame, "Test Length (mm)", -50, 50, 0)
        self.add_slider(sl_frame, "Static Body Z (mm)", -220, -100, -160)
        
        # Actions
        btn_frame = tk.Frame(sl_frame, bg="#f5f5f5")
        btn_frame.pack(fill="x", pady=10)
        self.btn_run_test = tk.Button(btn_frame, text="RUN SINGLE LEG TEST", command=self.toggle_test, 
                                     bg="#673AB7", fg="white", font=("Arial", 10, "bold"), width=20)
        self.btn_run_test.pack(side="left", padx=20)
        
        tk.Label(sl_frame, text="Note: Other legs will hold 'Stand Up' pose.", bg="#f5f5f5", fg="#666").pack(side="left")

    def add_slider(self, p, name, min_v, max_v, val):
        f = tk.Frame(p, bg="#f5f5f5"); f.pack(fill="x", pady=2)
        tk.Label(f, text=name, width=15, anchor="w", bg="#f5f5f5").pack(side="left")
        v = tk.DoubleVar(value=val)
        self.vars[name] = v
        tk.Scale(f, from_=min_v, to=max_v, orient="horizontal", variable=v, resolution=0.1).pack(side="right", fill="x", expand=True)

    def toggle_test(self):
        if self.running_test:
            self.stop_test()
        else:
            self.start_test()
            
    def start_test(self):
        self.ctx.mode = "TEST_SINGLE_LEG"
        self.running_test = True
        self.start_time = time.perf_counter()
        self.btn_run_test.config(text="STOP TEST", bg="#F44336")
        self.ctx.io.send_torque(True)
        self.test_loop()
        
    def stop_test(self):
        self.running_test = False
        self.btn_run_test.config(text="RUN SINGLE LEG TEST", bg="#673AB7")
        self.ctx.log("Test Stopped.")

    def test_loop(self):
        if not self.running_test or self.ctx.mode != "TEST_SINGLE_LEG": return
        
        # Params
        freq = self.vars["Test Freq (Hz)"].get()
        h = self.vars["Test Height (mm)"].get()
        l = self.vars["Test Length (mm)"].get() * -1.0 # Invert logic same as Trot
        body_z = self.vars["Static Body Z (mm)"].get()
        target = self.target_leg.get()
        
        now = time.perf_counter()
        elapsed = now - self.start_time
        
        period = 1.0 / freq
        phase = (elapsed % period) / period
        
        # Stand Pose for others (Using user provided Stand Up values or IK neutral)
        # Using IK neutral at body_z is safer to ensure height matches
        # Let's use IK for all.
        
        cmds = []
        COG_X =0.0; COG_Y = 0.0
        
        for name in self.legs:
            kin = self.legs[name]
            
            if name == target:
                # Active Leg Gait
                # Use Simple Trot Logic: 0-0.5 Swing, 0.5-1 Stance
                x_g, z_g = 0, 0
                if phase < 0.5:
                    swing_p = phase / 0.5
                    x_g, z_g = self.gait.get_swing_pos(swing_p, l, h)
                else:
                    stance_p = (phase - 0.5) / 0.5
                    x_g, z_g = self.gait.get_stance_pos(stance_p, l)
                
                res = kin.solve_ik(x_g + COG_X, kin.side_sign*cfg.L1 + COG_Y, body_z + z_g)
            else:
                # Static Leg
                res = kin.solve_ik(COG_X, kin.side_sign*cfg.L1 + COG_Y, body_z)
                
            if res:
                q = kin.rad_to_pwm(*res)
                ids = self.leg_ids[name]
                cmds.append((ids[0], q[0], 0, 0))
                cmds.append((ids[1], q[1], 0, 0))
                cmds.append((ids[2], q[2], 0, 0))
        
        self.ctx.io.send_servos(cmds)
        self.after(20, self.test_loop)

class TrotControlFrame(tk.Frame):
    def __init__(self, parent, context):
        super().__init__(parent, bg="#f0f0f0")
        self.ctx = context
        
        self.legs = {name: LegKinematics(name) for name in ["FL", "FR", "RL", "RR"]}
        self.leg_ids = {
            "FL": [1, 2, 3], "FR": [4, 5, 6],
            "RL": [7, 8, 9], "RR": [10, 11, 12]
        }
        self.gait = BezierGait()
        self.balance = BalanceController()
        
        self.running_trot = False
        self.start_time = 0
        
        # Params
        self.vars = {}
        self.setup_ui()
        self.after(100, self.update_imu_display)
        
    def setup_ui(self):
        # Controls Panel
        ctrl_panel = tk.LabelFrame(self, text="Real-time Controls", font=("Arial", 12, "bold"), bg="#f0f0f0")
        ctrl_panel.pack(fill="x", padx=10, pady=10)
        
        # Buttons
        btn_f = tk.Frame(ctrl_panel, bg="#f0f0f0")
        btn_f.pack(fill="x", pady=5)
        
        self.btn_start = tk.Button(btn_f, text="START GAIT", command=self.toggle_trot, 
                                  bg="#4CAF50", fg="white", font=("Arial", 12, "bold"), width=15)
        self.btn_start.pack(side="left", padx=10)
        
        tk.Button(btn_f, text="Emergency Stop", command=self.emergency_stop, 
                 bg="#f44336", fg="white", font=("Arial", 12, "bold")).pack(side="right", padx=10)

        # Gait Type Selection
        type_f = tk.Frame(ctrl_panel, bg="#f0f0f0")
        type_f.pack(fill="x", padx=10, pady=5)
        tk.Label(type_f, text="Gait Type:", bg="#f0f0f0", font=("Arial", 10, "bold")).pack(side="left")
        self.gait_type = tk.StringVar(value="TROT")
        ttk.Radiobutton(type_f, text="Trot (Fast)", variable=self.gait_type, value="TROT").pack(side="left", padx=10)
        ttk.Radiobutton(type_f, text="Walk (Stable 3-Leg)", variable=self.gait_type, value="WALK").pack(side="left", padx=10)

        # Gait Settings
        gf = tk.LabelFrame(ctrl_panel, text="Gait Settings", bg="#f0f0f0")
        gf.pack(fill="x", padx=5, pady=5)
        self.add_slider(gf, "Frequency", 0.5, 3.0, 2.0)
        self.add_slider(gf, "Step Height", 10.0, 60.0, 30.0) # Lower default to prevent kicking
        self.add_slider(gf, "Body Height", -220.0, -140.0, -180.0) # Adjustable!
        self.add_slider(gf, "Body Shift Y", 0.0, 30.0, 10.0) # Dynamic CoG Shift
        self.add_slider(gf, "Soft Cushion", 0.0, 20.0, 5.0) # Landing buffer
        self.add_slider(gf, "Step Length", -40.0, 40.0, 0.0)
        
        # Balance Settings
        bf = tk.LabelFrame(ctrl_panel, text="Balance Tuning", bg="#f0f0f0")
        bf.pack(fill="x", padx=5, pady=5)
        self.add_slider(bf, "Kp Roll", 0.0, 5.0, 1.0)
        self.add_slider(bf, "Kp Pitch", 0.0, 5.0, 0.8)
        self.add_slider(bf, "Kp Yaw", 0.0, 5.0, 1.5)
        self.add_toggle(bf, "Enable Balance", True)
        
        # IMU Display
        imu_f = tk.LabelFrame(self, text="IMU Status", font=("Arial", 10, "bold"), bg="#e0e0e0")
        imu_f.pack(fill="x", padx=10, pady=5)
        self.lbl_imu_roll = tk.Label(imu_f, text="Roll: 0.0", width=15, bg="#e0e0e0", font=("Consolas", 11))
        self.lbl_imu_roll.pack(side="left")
        self.lbl_imu_pitch = tk.Label(imu_f, text="Pitch: 0.0", width=15, bg="#e0e0e0", font=("Consolas", 11))
        self.lbl_imu_pitch.pack(side="left")
        
        # Status
        self.lbl_status = tk.Label(self, text="Status: IDLE", font=("Consolas", 14), bg="#ddd", pady=10)
        self.lbl_status.pack(fill="x", side="bottom")

    def add_slider(self, p, name, min_v, max_v, val):
        f = tk.Frame(p, bg="#f0f0f0"); f.pack(fill="x", pady=2)
        tk.Label(f, text=name, width=12, anchor="w", bg="#f0f0f0").pack(side="left")
        v = tk.DoubleVar(value=val)
        self.vars[name] = v
        tk.Scale(f, from_=min_v, to=max_v, orient="horizontal", variable=v, resolution=0.1).pack(side="right", fill="x", expand=True)

    def add_toggle(self, p, name, val):
        v = tk.BooleanVar(value=val)
        self.vars[name] = v
        tk.Checkbutton(p, text=name, variable=v, bg="#f0f0f0").pack(anchor="w")

    def toggle_trot(self):
        if not self.running_trot:
            self.start_trot()
        else:
            self.stop_trot()

    def start_trot(self):
        self.ctx.mode = "TROT"
        self.running_trot = True
        self.start_time = time.perf_counter()
        
        # Initialize Yaw Target
        imu = self.ctx.io.get_imu()
        self.target_yaw = imu['yaw']
        self.balance.pid_yaw.reset()
        
        self.btn_start.config(text="STOP GAIT", bg="#ff9800")
        self.lbl_status.config(text=f"Status: {self.gait_type.get()}ING", bg="#a5d6a7")
        self.ctx.io.send_torque(True)
        self.trot_loop()

    def stop_trot(self):
        self.running_trot = False
        self.btn_start.config(text="START GAIT", bg="#4CAF50")
        self.lbl_status.config(text="Status: STOPPED", bg="#ddd")
        # Don't relax immediately, maybe hold pose?
        # For safety, let's just stop sending updates. Manual mode can take over.

    def emergency_stop(self):
        self.stop_trot()
        self.ctx.mode = "RELAX"
        self.ctx.io.send_torque(False)
        self.lbl_status.config(text="Status: EMERGENCY STOP (RELAXED)", bg="#ef9a9a")

    def update_imu_display(self):
        if self.running_trot:
            imu = self.ctx.io.get_imu()
            self.lbl_imu_roll.config(text=f"Roll: {imu['roll']:.2f}")
            self.lbl_imu_pitch.config(text=f"Pitch: {imu['pitch']:.2f}")
        self.after(100, self.update_imu_display)

    def trot_loop(self):
        if not self.running_trot or self.ctx.mode != "TROT": return
        
        # 1. Read Params
        freq = self.vars["Frequency"].get()
        step_h = self.vars["Step Height"].get()
        body_h = self.vars["Body Height"].get()
        step_l = self.vars["Step Length"].get() * -1.0 
        shift_amp = self.vars["Body Shift Y"].get()
        cushion_amp = self.vars["Soft Cushion"].get()
        bal_en = self.vars["Enable Balance"].get()
        g_type = self.gait_type.get()
        
        # Update PID
        self.balance.pid_roll.kp = self.vars["Kp Roll"].get()
        self.balance.pid_pitch.kp = self.vars["Kp Pitch"].get()
        self.balance.pid_yaw.kp = self.vars["Kp Yaw"].get()
        
        # 2. State & Timing
        now = time.perf_counter()
        elapsed = now - self.start_time
        dt = 0.02 
        
        # Soft Start (Ramp height)
        current_step_h = min(step_h, step_h * (elapsed / 2.0))
        
        # IMU & Yaw Correction
        imu = self.ctx.io.get_imu()
        yaw_corr_rad = 0.0
        
        bal_offsets = {"FL":0, "FR":0, "RL":0, "RR":0}
        if bal_en:
            # R/P Balance
            r_off, p_off = self.balance.solve(imu['roll'], imu['pitch'], 0, 0, dt)
            
            # Yaw Balance (Closed Loop Heading)
            yaw_adjust_deg = self.balance.solve_yaw(imu['yaw'], self.target_yaw, dt)
            yaw_corr_rad = math.radians(yaw_adjust_deg)
            
            # Apply R/P offsets
            bal_offsets['FL'] = p_off + r_off
            bal_offsets['FR'] = p_off - r_off
            bal_offsets['RL'] = -p_off + r_off
            bal_offsets['RR'] = -p_off - r_off
            
        # Determine Cycle Phase
        period = 1.0 / freq
        global_phase = (now % period) / period
        
        # Dynamic Body Shift (Sine Wave)
        # Shift Body towards support side.
        # Phase 0-0.5: FL/RR Swing -> Support FR/RL (Right-ish). Move Body Right (COG_Y > 0)
        # Phase 0.5-1: FR/RL Swing -> Support FL/RR (Left-ish). Move Body Left (COG_Y < 0)
        # sin(2*pi*ph) is + for 0-0.5, - for 0.5-1.
        cog_y_dynamic = shift_amp * math.sin(2 * math.pi * global_phase)
        
        cmds = []
        
        # Configure Phase Offsets
        if g_type == "WALK":
            phase_offsets = {"RL": 0.0, "FL": 0.25, "RR": 0.5, "FR": 0.75}
            swing_duration = 0.25
        else:
            phase_offsets = {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}
            swing_duration = 0.5
            
        COG_X = 40.0; 
        
        # Shoulder Positions (Approximate for Yaw Rotation)
        # FL: (L/2, W/2), FR: (L/2, -W/2), RL: (-L/2, W/2), RR: (-L/2, -W/2)
        # We use relative to Hip, so we need to be careful.
        # solve_ik takes pos relative to hip.
        # To rotate body, we rotate the target foot pos in body frame.
        
        for name in self.legs:
            kin = self.legs[name]
            ph = (global_phase + phase_offsets[name]) % 1.0
            
            x_g, z_g = 0, 0
            current_acc = 0 # Default max acc
            
            # Generalized Gait Logic
            if ph < swing_duration: 
                # Swing Phase
                swing_p = ph / swing_duration
                x_g, z_g = self.gait.get_swing_pos(swing_p, step_l, current_step_h)
                current_acc = 0 # High responsiveness for swing
            else: 
                # Stance Phase
                stance_p = (ph - swing_duration) / (1.0 - swing_duration)
                x_g, z_g = self.gait.get_stance_pos(stance_p, step_l)
                
                # Soft Landing Cushion
                if stance_p < 0.15:
                    decay = (0.15 - stance_p) / 0.15
                    z_g += cushion_amp * decay
                    # Reduce Acc during landing to dampen impact
                    current_acc = 40 
                else:
                    current_acc = 0
            
            # Combine
            z_bal = -bal_offsets[name] 
            
            # 1. Base Pos relative to Hip
            tx = x_g + COG_X
            ty = kin.side_sign * cfg.L1 + cog_y_dynamic
            tz = body_h + z_g + z_bal
            
            # 2. Yaw Rotation (Optional Balance)
            if bal_en and yaw_corr_rad != 0:
                hip_x = cfg.LENGTH / 2.0 if "F" in name else -cfg.LENGTH / 2.0
                hip_y = cfg.WIDTH / 2.0 if "L" in name else -cfg.WIDTH / 2.0
                bx = hip_x + tx
                by = hip_y + ty
                c, s = math.cos(-yaw_corr_rad), math.sin(-yaw_corr_rad)
                bx_new = bx * c - by * s
                by_new = bx * s + by * c
                tx = bx_new - hip_x
                ty = by_new - hip_y

            res = kin.solve_ik(tx, ty, tz)
            if res:
                q = kin.rad_to_pwm(*res)
                ids = self.leg_ids[name]
                # Use a small Time (20ms) to allow servo internal smoothing 
                # between Python update cycles.
                t_smooth = 20 
                cmds.append((ids[0], q[0], t_smooth, 0, current_acc))
                cmds.append((ids[1], q[1], t_smooth, 0, current_acc))
                cmds.append((ids[2], q[2], t_smooth, 0, current_acc))
                
        self.ctx.io.send_servos(cmds)
        
        # Schedule next loop
        self.after(20, self.trot_loop)

class UnifiedControlApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Weixue Dog Nano - Mission Control")
        self.geometry("1000x800")
        
        self.ctx = SharedContext()
        self.ctx.log_callback = self.log
        self.app_ready = False
        
        # Log Area at bottom
        self.log_box = scrolledtext.ScrolledText(self, height=6, bg="#1e1e1e", fg="#00ff00", font=("Consolas", 9))
        self.log_box.pack(fill="x", side="bottom")
        
        self.setup_connection_bar()
        
        # Auto-detect connection state
        if self.ctx.io.ser:
            self.log(f"Auto-connected to {self.ctx.io.port}. Entering RELAX mode.")
            self.ctx.mode = "RELAX"
            self.ctx.io.send_torque(False)
            
        # Notebook
        self.nb = ttk.Notebook(self)
        self.nb.pack(fill="both", expand=True)
        
        self.tab_manual = ManualControlFrame(self.nb, self.ctx)
        self.tab_trot = TrotControlFrame(self.nb, self.ctx)
        self.tab_test = TestLabFrame(self.nb, self.ctx)
        
        self.nb.add(self.tab_manual, text="Manual Control")
        self.nb.add(self.tab_trot, text="Trot Controller")
        self.nb.add(self.tab_test, text="Test Lab")
        
        self.nb.bind("<<NotebookTabChanged>>", self.on_tab_change)
        
        self.app_ready = True

    def setup_connection_bar(self):
        bar = tk.Frame(self, bg="#ccc", height=30)
        bar.pack(side="bottom", fill="x", before=self.log_box)
        tk.Label(bar, text="Port:").pack(side="left")
        self.cbo_port = ttk.Combobox(bar, values=[p.device for p in serial.tools.list_ports.comports()])
        self.cbo_port.pack(side="left")
        if self.cbo_port['values']: self.cbo_port.current(0)
        tk.Button(bar, text="Connect", command=self.connect).pack(side="left")
        
    def connect(self):
        p = self.cbo_port.get()
        if p:
            self.ctx.io.connect(p)
            self.log(f"Connecting to {p}...")
            # Auto-relax on connect
            self.log("Connected. Entering RELAX mode.")
            self.ctx.mode = "RELAX"
            self.ctx.io.send_torque(False)

    def on_tab_change(self, event):
        if not self.app_ready: return
        
        # Safety switch
        sel = self.nb.select()
        text = self.nb.tab(sel, "text")
        
        if text == "Manual Control":
            # Don't auto-enable MANUAL if we are in RELAX
            # The user must click "Enable Manual"
            self.tab_trot.stop_trot()
            self.tab_test.stop_test()
            self.log("Switched to MANUAL Tab. (Click 'Enable Manual' to move)")
        elif text == "Trot Controller":
            pass
        elif text == "Test Lab":
            self.tab_trot.stop_trot()
            self.log("Switched to TEST LAB.")

    def log(self, msg):
        ts = datetime.now().strftime('%H:%M:%S')
        self.log_box.insert(tk.END, f"[{ts}] {msg}\n")
        self.log_box.see(tk.END)

if __name__ == "__main__":
    app = UnifiedControlApp()
    app.mainloop()
