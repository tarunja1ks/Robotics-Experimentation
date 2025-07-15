import pybullet as p
import pybullet_data
import logging
import time
import numpy as np
import os

# Configure logging
logging.basicConfig(
    filename="..\Simulation_logging\Ackerman_simulation.log",
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)


class PybulletEnvironment:
    def __init__(self):
        logging.info("Initializing PyBullet environment...")
        self.physics_client =p.connect(p.GUI, options="--opengl2")
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        # p.changeDynamics(
        #    lateralFriction=2
        # )
        self.plane_id = p.loadURDF("plane.urdf")
        
        # choose which to use by setting self.Cylinder to Cylinder() or Robot()
        # self.Cylinder = Cylinder()

        self.Cylinder = Robot()

        # PID controllers
        self.x_pid = PID(kp=1.0, ki=0.01, kd=0.1)
        self.y_pid = PID(kp=1.0, ki=0.01, kd=0.1)

        # Goal position sliders
        self.x_goalpos = p.addUserDebugParameter("X Goal Position", -100, 100, 1)
        self.y_goalpos = p.addUserDebugParameter("Y Goal Position", -100, 100, 1)

        # PID constant sliders
        self.kp_slider = p.addUserDebugParameter("KP", 0, 10, 1)
        self.ki_slider = p.addUserDebugParameter("KI", 0, 1, 0.01)
        self.kd_slider = p.addUserDebugParameter("KD", 0, 1, 0.1)

    def run_simulation(self, boolean=True):
        logging.info("Starting simulation...")
        start_time = time.time()
        print(boolean)

        if boolean:
            cube = Cylinder()
        try:
            while True:
                p.stepSimulation()
                current_time = time.time()
                time.sleep(1.0 / 240.0)
                # Update PID constants from sliders
                kp = p.readUserDebugParameter(self.kp_slider)
                ki = p.readUserDebugParameter(self.ki_slider)
                kd = p.readUserDebugParameter(self.kd_slider)
                self.x_pid.update(kp, ki, kd)
                self.y_pid.update(kp, ki, kd)

                # Get current position and goal
                current_position = self.Cylinder.getPosition()

                x_goal = p.readUserDebugParameter(self.x_goalpos)
                y_goal = p.readUserDebugParameter(self.y_goalpos)
                line = p.addUserDebugLine([x_goal, y_goal, 0], current_position, lineColorRGB=[1, 0, 0], lineWidth=7)
                if cube.getPosition() != [x_goal, y_goal, 1]:
                    cube.setPosition(x_goal, y_goal)
                    cube.updatePosition()
                if (current_time - start_time) > 0.1:
                    # print(current_time)
                    p.removeAllUserDebugItems()
                    start_time = current_time

                # Calculate error
                x_error = x_goal - current_position[0]
                y_error = y_goal - current_position[1]

                # Use PID to calculate velocities
                x_velocity = self.x_pid.calculateVelocity(x_error)
                y_velocity = self.y_pid.calculateVelocity(y_error)
                target_yaw = np.arctan2(y_error, x_error)
                x_yaw = self.Cylinder.getHeading()

                # steering_angle = np.arctan2(np.sin(target_yaw - x_yaw), np.cos(target_yaw - x_yaw))
                steering_angle = (target_yaw - x_yaw)
                if x_velocity < 0:
                    steering_angle = -steering_angle + np.pi
                if abs(steering_angle) > np.pi:
                    steering_angle -= 2 * np.pi

                #print(steering_angle, x_velocity, target_yaw, x_yaw)
                # Apply the velocities
                # self.Cylinder.setVelocity(np.abs(x_velocity),steering_angle) #forward only
                self.Cylinder.setVelocity(x_velocity, steering_angle)

                # Update debug text
                # self.update_info_text(x_velocity, y_velocity, current_position)

                # Log details
                logging.info(f"Position: {current_position}, Errors: X={x_error}, Y={y_error}, "
                             f"Velocities: X={x_velocity}, Y={y_velocity}, PID: KP={kp}, KI={ki}, KD={kd}")

        except KeyboardInterrupt:
            logging.warning("Simulation stopped by user.")
        except Exception as e:
            logging.error(f"Unexpected error: {e}")
        finally:
            p.disconnect()
            logging.info("Simulation ended.")

    def update_info_text(self, x_velocity, y_velocity, position):
        """
        Update the debug text overlay with the current velocities and position.
        """
        text = f"Velocities: X={x_velocity:.2f}, Y={y_velocity:.2f}\nPosition: {position}"
        p.addUserDebugText(
            text=text,
            textPosition=[0, 0, 3],
            textColorRGB=[1, 1, 1],
            textSize=1.5,
            replaceItemUniqueId=self.info_text_id
        )


class Cylinder:
    def __init__(self):
        cylinder_height = 2
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(script_dir)
        urdf_path = os.path.join(project_root,"urdf", "cylinder.urdf")
        self.cylinder_id = p.loadURDF(urdf_path, basePosition=[5, 5, cylinder_height / 2])
        self.position = self.getPosition()

    def getPosition(self):
        return p.getBasePositionAndOrientation(self.cylinder_id)[0]

    def updatePosition(self):
        self.position = self.getPosition()

    def setPosition(self, x, y):
        p.resetBasePositionAndOrientation(self.cylinder_id, [x, y, 0], [0, 0, 0, 1])

    def setVelocity(self, x, y):
        p.resetBaseVelocity(self.cylinder_id, [x, y, 0])
        logging.info(f"Set cylinder velocity: X={x}, Y={y}")


class Robot:
    def __init__(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(script_dir)
        urdf_path = os.path.join(project_root,"urdf", "racecar.urdf")
        self.robot_id = p.loadURDF(urdf_path, basePosition=[0, 0, 0.2])
        self.position = self.getPosition()

        # Retrieve wheel and steering joint indices
        self.steering_links = [4, 6]  # Front left and front right steering links
        self.wheels = [2, 3, 5, 7]  # All four wheels

    def getPosition(self):

        return p.getBasePositionAndOrientation(self.robot_id)[0]

    def getHeading(self):

        orientation = p.getBasePositionAndOrientation(self.robot_id)[1]
        euler_angles = p.getEulerFromQuaternion(orientation)
        return euler_angles[2]  # Yaw angle

    def setVelocity(self, forward_speed_x, steering_angle):

        # Set the steering angle for front wheels
        for steering_link in self.steering_links:
            p.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=steering_link,
                controlMode=p.POSITION_CONTROL,
                targetPosition=steering_angle
            )

        # Set the wheel velocities
        for wheel in self.wheels:
            p.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=wheel,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=forward_speed_x,
                force=0.25
            )

        logging.info(f"Set racecar velocity: Forward={forward_speed_x}, Steering={steering_angle}")


class PID:
    def __init__(self, kp=0.1, ki=0.01, kd=0.05):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0
        self.dt = 1 / 240.0  # Simulation time step (default is 1/240 seconds)

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


if __name__ == "__main__":
    env = PybulletEnvironment()
    try:
        env.run_simulation()
    except KeyboardInterrupt:
        logging.info("Simulation terminated by user.")
