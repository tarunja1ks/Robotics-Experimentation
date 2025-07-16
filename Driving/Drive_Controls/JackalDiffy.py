import pybullet as p
import pybullet_data
import logging
import time
import numpy as np
import math
import sys
import os

# Adjusting sys.path for module imports
current_dir = os.path.dirname(os.path.abspath(__file__))
n_levels_up = 2
parent_dir = current_dir
for _ in range(n_levels_up):
    parent_dir = os.path.abspath(os.path.join(parent_dir, ".."))
sys.path.append(parent_dir)

# Assuming these modules exist in the 'Driving' directory
# from Driving import PID
# from Driving import Bezier
# from Driving import PurePursuit

logging.basicConfig(
    filename="Driving/Simulation_logging/Diffy_simulation.log",
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)


class PybulletEnvironment:
    def __init__(self):
        logging.info("Initializing PyBullet environment...")
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        self.plane_id = p.loadURDF("plane.urdf")
        p.changeDynamics(self.plane_id, -1, lateralFriction=3.0)
        
        self.jackal_robot = Robot()

        self.sliders = []
        self.create_sliders()
            
    def create_sliders(self):
        """Creates debug sliders for linear and angular velocity control."""
        self.linear_velocity_slider = p.addUserDebugParameter("Linear Vel", -2.0, 2.0, 0) # Reduced range for smoother control
        self.angular_velocity_slider = p.addUserDebugParameter("Angular Vel", -2.0, 2.0, 0) # Reduced range for smoother control
            
    def run_simulation(self): # Removed boolean parameter as it wasn't used
        logging.info("Starting simulation...")
        
        # Get the fixed simulation time step from PyBullet
        # This is more robust than a hardcoded value if it changes
        self.dt = p.getPhysicsEngineParameters()['fixedTimeStep'] if p.getPhysicsEngineParameters()['fixedTimeStep'] else (1.0 / 240.0)
        
        while True:
            p.stepSimulation()
            time.sleep(self.dt) # Use the actual simulation time step for sleep

            self.read_sliders() # Reads in input sliders
            
            # Localization
            self.jackal_robot.localize()
            self.current_x = self.jackal_robot.position[0]
            self.current_y = self.jackal_robot.position[1]
            self.current_h = self.jackal_robot.position[2]
            
            # Setting the desired velocities for the robot
            self.jackal_robot.inverse_kinematics(self.linear_velocity, self.angular_velocity)
            
            # Apply the ramped velocities to the wheels
            self.jackal_robot.setVelocity(self.dt) # Pass dt to setVelocity for ramping

            # Logging data
            logging.info(f"Position: {', '.join(str(i) for i in self.jackal_robot.position)}, "
                         f"Desired Linear Velocity: {self.linear_velocity}, "
                         f"Desired Angular Velocity: {self.angular_velocity}, "
                         f"Left Wheel Cmd: {self.jackal_robot.current_vl_cmd:.2f}, "
                         f"Right Wheel Cmd: {self.jackal_robot.current_vr_cmd:.2f}")

    def angle_wrap(self, radians): # returning angle between -pi and pi
        while radians > math.pi:
            radians -= 2 * math.pi
        while radians < -math.pi:
            radians += 2 * math.pi
        return radians
    
    def read_sliders(self): # reading in data from sliders
        """Reads linear and angular velocity commands from debug sliders."""
        try:
            self.linear_velocity = float(p.readUserDebugParameter(self.linear_velocity_slider))
            self.angular_velocity = float(p.readUserDebugParameter(self.angular_velocity_slider))
        except Exception as e:
            logging.error(f"Failed to read slider parameters: {e}")
            # Set to 0 to prevent errors from stopping simulation if sliders are not ready
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            return
        
# /home/tarunj/Documents/Robotics/Robotics-Experimentation/Driving/urdf/jackal.urdf
        
class Robot:
    def __init__(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(script_dir)
        urdf_path = os.path.join(project_root,"urdf", "jackal.urdf")
        print(f"Loading URDF from: {urdf_path}") # Confirming path
        self.robot_id = p.loadURDF(urdf_path, basePosition=[0, 0, 0.1])
        
        # Set initial position after loading
        self.position = self.localize() 
        
        self.wheels = [1, 2, 3, 4]   # All four wheels
        self.wheels_left = [1, 3]
        self.wheels_right = [2, 4]
        
        self.track_width_half = 0.187795 # Half the distance between the wheels (L/2)
        self.wheel_radius = 0.098 # Radius of the wheel in meters (from URDF)
        
        # Desired angular velocities for wheels (calculated by inverse_kinematics)
        self.target_vl = 0.0 
        self.target_vr = 0.0
        
        # Current angular velocity commands sent to PyBullet (ramped values)
        self.current_vl_cmd = 0.0 
        self.current_vr_cmd = 0.0
        
        # Maximum angular acceleration for wheels (rad/s^2) - tune this!
        # A higher value means faster acceleration/deceleration.
        # Start with a moderate value and adjust based on feel.
        self.max_angular_acceleration = 50.0 # Increased from 20.0 for potentially better response

        # Apply dynamics changes to wheels
        for wheel in self.wheels:
            p.changeDynamics(self.robot_id, wheel,
                            lateralFriction=2.0, # Good for grip
                            spinningFriction=0.4,
                            rollingFriction=0.1,
                            linearDamping=0,
                            angularDamping=0,
                            jointDamping=0.05) # Added a small joint damping for stability

        # Apply dynamics changes to the base link (chassis)
        # Link index -1 refers to the base link (chassis in this case)
        # Ensure the mass is set in the URDF, but you can override here if needed for testing
        # p.changeDynamics(self.robot_id, -1, mass=17.0) # Only uncomment if you want to override URDF mass
        p.changeDynamics(self.robot_id, -1,
                         linearDamping=0.5,  # Increased linear damping to reduce sliding
                         angularDamping=0.5) # Increased angular damping to reduce oscillations
        
        """
        Wheel Index 1: Front Left
        Wheel Index 2: Front Right
        Wheel Index 3: Back Left
        Wheel Index 4: Back Right
        """

    def getPosition(self): # getting robot position
        """Returns the base position (x, y) and orientation (z-axis yaw) of the robot."""
        return p.getBasePositionAndOrientation(self.robot_id)[0]
