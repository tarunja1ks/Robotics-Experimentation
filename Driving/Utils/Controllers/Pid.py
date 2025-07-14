import matplotlib.pyplot as plt
import numpy as np
from collections import deque
class PID:
    def __init__(self, kp=0.1, ki=0.01, kd=0.05,dt=1/240.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0
        self.dt = dt # Simulation time step (default is 1/240 seconds)

    def update(self, kp, ki, kd):
        """
        Update the PID constants.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def calculateVelocity(self, error):
        """
        Compute the control output based on the error using PID.
        """
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output
    
class PID_Plotter:
    def __init__(self,buffer_size=50):
        plt.ion()
        fig, ax = plt.subplots(figsize=(10, 6))
        ax.set_title("PID Tuning Graph")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Position")
        ax.grid()
        self.pid_data = {
            "PID1": {
                "goal_position": 50,
                "time_buffer": deque(maxlen=buffer_size),
                "current_buffer": deque(maxlen=buffer_size),
                "goal_line": ax.plot([], [], 'g--', label="PID1 Goal")[0],
                "current_line": ax.plot([], [], 'b-', label="PID1 Current")[0],
            },
            "PID2": {
                "goal_position": 30,
                "time_buffer": deque(maxlen=buffer_size),
                "current_buffer": deque(maxlen=buffer_size),
                "goal_line": ax.plot([], [], 'r--', label="PID2 Goal")[0],
                "current_line": ax.plot([], [], 'm-', label="PID2 Current")[0],
            }
        }
        ax.legend()

    def update_pid(self,pid_name, elapsed_time, current_position):
        if pid_name not in self.pid_data:
            return
        pid = self.pid_data[pid_name]
        pid["time_buffer"].append(elapsed_time)
        pid["current_buffer"].append(current_position)
        pid["goal_line"].set_data(list(pid["time_buffer"]), [pid["goal_position"]] * len(pid["time_buffer"]))
        pid["current_line"].set_data(list(pid["time_buffer"]), list(pid["current_buffer"]))