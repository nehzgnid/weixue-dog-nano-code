# ... (Previous README content)

## Simulation

A new interactive simulation tool has been added to test the Bezier gait and balance logic without hardware.

*   **`simulation_trot_interactive.py`**:
    *   Visualizes the robot's leg movements in 3D.
    *   **Sliders**:
        *   `Frequency`, `Step Height`, `Step Length`: Adjust gait parameters in real-time.
        *   `Sim Roll/Pitch`: Simulate IMU tilt to test the Balance Controller.
        *   `Kp Roll/Pitch`: Tune PID gains dynamically.
    *   **Usage**: `python simulation_trot_interactive.py`
