
class BezierGait:
    def __init__(self):
        self.step_length = 0.0
        self.step_height = 0.0
        self.step_offset = 0.0 
        
    def get_swing_pos(self, phase, step_len, step_height):
        """
        Calculate leg position during swing phase (phase: 0.0 -> 1.0)
        Returns: (x, z) relative to hip center (at neutral stance)
        """
        t = max(0.0, min(1.0, phase))
        
        L = step_len
        H = step_height
        
        # Bernstein Polynomials for N=11 (10th degree)
        n = 10
        x_val = 0.0
        z_val = 0.0
        
        coeffs = [1, 10, 45, 120, 210, 252, 210, 120, 45, 10, 1]
        
        # Control Points Adjustment
        # If step_len > 0 (Forward command), Body moves +X. Feet must move -X during Stance.
        # So during Swing, Feet must move +X (from Back to Front).
        # Swing Start (0.0): Back (-L/2)
        # Swing End (1.0): Front (+L/2)
        # This matches current logic: -L/2 to L/2.
        
        # NOTE: User reported "Negative step length makes it go forward".
        # This means currently Positive Step Length moves feet -L/2 -> L/2 (Back to Front).
        # This pushes the robot backward? 
        # If foot moves Back->Front in Swing, it lands at Front. Then in Stance it goes Front->Back.
        # Front->Back stance pushes body Forward.
        # So Positive Step Length SHOULD be Forward.
        # If User says Negative is Forward, then either:
        # 1. Their motor direction is inverted.
        # 2. My Stance logic is inverted.
        
        # Let's check Stance logic below.
        
        cp_x = [
            -L/2, -L/2, -L/2, -L/2, -L/2, 
             0.0, 
             L/2,  L/2,  L/2,  L/2,  L/2 
        ]
        
        cp_z = [
             0.0,    0.0,    H*0.5,  H,      H,      H,      H,      H,      H*0.5,  0.0,    0.0     
        ]

        for i in range(n + 1):
            basis = coeffs[i] * (t ** i) * ((1 - t) ** (n - i))
            x_val += basis * cp_x[i]
            z_val += basis * cp_z[i]
            
        return x_val, z_val

    def get_stance_pos(self, phase, step_len):
        """
        Calculate leg position during stance phase (phase: 0.0 -> 1.0)
        """
        # Stance Logic:
        # We want linear velocity.
        # If we just landed from Swing (Phase=0), we are at End of Swing (+L/2).
        # We need to go to Start of Swing (-L/2).
        # So +L/2 -> -L/2.
        
        # Original code: x_val = (step_len / 2) - phase * step_len
        # Phase 0: L/2. Phase 1: -L/2.
        # This moves Foot Front -> Back.
        # Ground reaction pushes Body Back -> Front.
        # So Body moves Forward.
        
        # User says: Negative step len makes it go forward.
        # If L is negative:
        # Swing: -(-L)/2 -> +(-L)/2  => +A -> -A (Front to Back).
        # Stance: (-L)/2 -> -(-L)/2 => -A -> +A (Back to Front).
        # Foot moves Back -> Front on ground. Pushes body Backward.
        # So Negative L should mean Backward.
        
        # If User sees Negative L = Forward, then likely kinematic X is inverted relative to robot body.
        # i.e. Robot Front is -X in kinematics?
        # Let's check kinematics_v5.py:
        # L1 is y offset. x is x.
        # x_eff = x. phi = atan2(x, -z).
        # Hip Pitch q_hip = phi + ...
        # If x is positive, phi is positive.
        # Positive hip angle usually moves leg forward?
        # Standard servo: 2048 is 0. 
        # pwm = 2048 + (q - offset) * ratio * dir.
        # If dir is 1. q increases -> pwm increases.
        # We need to verify if +PWM moves leg Forward or Backward.
        # Without hardware access, we trust the user.
        # If they say step_len is inverted, I will simply invert the sign of step_len used in calculations.
        
        # FIX: Invert L for internal calculation to match user expectation.
        # But wait, if I invert L, I invert both Swing and Stance.
        # Effect: L_input > 0 -> L_internal < 0 -> Robot Backward. (Current behavior)
        # User wants L_input > 0 -> Robot Forward.
        # So I need to NEGATE L.
        
        # Let's verify "Drift".
        # Drift might be due to Stance Phase not perfectly matching Swing landing.
        # Swing: -L/2 to L/2.
        # Stance: L/2 to -L/2.
        # They match at boundaries.
        
        # Drift to the right?
        # Could be coordinate system mismatch.
        # Or maybe the "Straight line" in X is not straight in 3D arc of the servo?
        # Our IK is analytical, should be exact.
        # Maybe Side_Sign * L1 logic?
        
        x_val = (step_len / 2) - phase * step_len
        z_val = 0.0 
        
        return x_val, z_val
