import pybullet as p
import pybullet_data
import logging
import time
import numpy as np
import math
import sys
import os

# logging

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
        p.changeDynamics(self.plane_id, -1, lateralFriction=1.0)
        self.robot = Robot() # initalizing the jackal
        self.sliders=[] 
        self.create_sliders()
    def create_sliders(self):
        self.linear_velocity_parameter=p.addUserDebugParameter("Linear Vel", -100, 100, 0) 
        self.angular_velocity_parameter=p.addUserDebugParameter("Angular Vel", -100, 100, 0) 
            
    def run_simulation(self, boolean = True):
        logging.info("Starting simulation...")
        
        while True:
                p.stepSimulation()
                time.sleep(1.0 / 240.0) 
                self.read_sliders()
                # localization
                self.robot.localize()
                self.current_x=self.robot.position[0]
                self.current_y=self.robot.position[1]
                self.current_h=self.robot.position[2]

                # setting the velocities
                self.robot.inverse_kinematics(self.robot.linear_velocity,self.robot.angular_velocity)
                self.robot.setVelocity()
              

                

    def angle_wrap(self,radians): # returning angle between -pi and pi
        while radians > math.pi:
            radians -= 2 * math.pi
        while radians < -math.pi:
            radians += 2 * math.pi
        return radians
    def read_sliders(self): 
        try:
            self.robot.linear_velocity=float(p.readUserDebugParameter(self.linear_velocity_parameter))
            self.robot.angular_velocity=float(p.readUserDebugParameter(self.angular_velocity_parameter))
        except:
            return


class Robot:
    def __init__(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(script_dir)
        urdf_path = os.path.join(project_root,"urdf", "jackal.urdf")
        self.robot_id = p.loadURDF(urdf_path, basePosition=[0, 0, 0.2])
        self.position=self.localize()
        self.wheels = [1,2,3,4]   # All four wheels
        self.wheels_left=[1,3]
        self.wheels_right=[2,4]
        self.track_radius=0.187795 
        self.vl=0
        self.vr=0
        self.linear_velocity=0
        self.angular_velocity=0
        
        
        
        """
        Wheel Index 1: Front Left
        Wheel Index 2: Front Right
        Wheel Index 3: Back Left
        Wheel Index 4: Back Right
        """

    def getPosition(self): # getting robot position
        return p.getBasePositionAndOrientation(self.robot_id)[0]

    def getHeading(self): # getting robot heading
        orientation = p.getBasePositionAndOrientation(self.robot_id)[1]
        euler_angles = p.getEulerFromQuaternion(orientation)
        return euler_angles[2]  # Yaw angle
    
    def localize(self): # outputs and sets position in x, y, heading format
        self.position=[]# clearing old position
        self.position.append(self.getPosition()[0])
        self.position.append(self.getPosition()[1])
        self.position.append(self.getHeading())
        return self.position
        
    def getRobotId(self):
        return self.robot_id
    def setVelocity(self): # setting velocity on the left and right wheels(you set wheels to left and right not individual in tank drive)

            for wheel in self.wheels_left:
                p.setJointMotorControl2(
                    bodyIndex=self.robot_id,
                    jointIndex=wheel,
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocity=self.vl,
                    force = 50 # i think the torque on a jackal
                )
            for wheel in self.wheels_right:
                p.setJointMotorControl2(
                    bodyIndex=self.robot_id,
                    jointIndex=wheel,
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocity=self.vr,
                    force = 50 # i think the torque on a jackal
                )
    def inverse_kinematics(self,v_f, v0): # converting forward+angular velocity into wheel velocities
        self.vr=v_f+self.track_radius*v0        # velocity of left wheels
        self.vl=v_f-self.track_radius*v0       # velocity of right wheels 
        
        
        


if __name__ == "__main__":
    env = PybulletEnvironment()
    try:
        env.run_simulation()
    except KeyboardInterrupt:
        logging.info("Simulation terminated by user.")
    
