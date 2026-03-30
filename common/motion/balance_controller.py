class PID:
    def __init__(self, kp, ki, kd, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        
        self.prev_error = 0.0
        self.integral = 0.0
        
        # Low Pass Filter for Derivative
        # alpha = dt / (RC + dt). RC = 1/(2*pi*cutoff_freq)
        # Typically want D-term cutoff around 5-10Hz for stability
        self.alpha = 0.2
        self.prev_derivative = 0.0
        
    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_derivative = 0.0
        
    def update(self, error, dt):
        if dt <= 0: return 0.0
        
        # P
        p_term = self.kp * error
        
        # I
        self.integral += error * dt
        # Anti-windup clamping could be added here if needed
        i_term = self.ki * self.integral
        
        # D (Filtered)
        raw_derivative = (error - self.prev_error) / dt
        derivative = self.prev_derivative * (1.0 - self.alpha) + raw_derivative * self.alpha
        self.prev_derivative = derivative
        
        d_term = self.kd * derivative
        
        self.prev_error = error
        
        output = p_term + i_term + d_term
        
        if self.output_limit:
            output = max(-self.output_limit, min(self.output_limit, output))
            
        return output

class BalanceController:
    def __init__(self):
        # Reduced default gains to prevent oscillation
        self.pid_roll = PID(kp=0.6, ki=0.0, kd=0.02, output_limit=25.0) 
        self.pid_pitch = PID(kp=0.5, ki=0.0, kd=0.02, output_limit=25.0)
        self.pid_yaw = PID(kp=1.5, ki=0.0, kd=0.05, output_limit=15.0) # Yaw correction (degrees rotation)
        
    def reset(self):
        self.pid_roll.reset()
        self.pid_pitch.reset()
        self.pid_yaw.reset()
        
    def solve(self, current_roll, current_pitch, target_roll=0.0, target_pitch=0.0, dt=0.02):
        err_roll = target_roll - current_roll
        err_pitch = target_pitch - current_pitch
        
        adjust_roll = self.pid_roll.update(err_roll, dt)
        adjust_pitch = self.pid_pitch.update(err_pitch, dt)
        
        return adjust_roll, adjust_pitch

    def solve_yaw(self, current_yaw, target_yaw, dt=0.02):
        err_yaw = target_yaw - current_yaw
        # Handle angle wrapping (-180 to 180)
        if err_yaw > 180: err_yaw -= 360
        if err_yaw < -180: err_yaw += 360
        
        return self.pid_yaw.update(err_yaw, dt)

