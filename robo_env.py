import gymnasium as gym
from gymnasium import error, spaces
import pybullet as p
import pybullet_data
import os
import numpy as np
import math
import random
import time
from gymnasium.utils import seeding  # Import seeding utility from Gymnasium

class RoBots(gym.Env):

    def __init__(self):
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.observation = []
        self.action_space = spaces.Discrete(15) # 15 different +ve/ -ve velocities to regulate the tyres angular vel
        # pitch, gyro, commanded speed
        self.observation_space = spaces.Box(np.array([-math.pi, -math.pi, -5]), high=np.array([math.pi, math.pi, 5]),
                                            shape=(3,), dtype=np.float64) 
        self.seed()
    
    def set_camera(self):
        distance = 1  # Distance from the target
        yaw = 50  # Yaw angle
        pitch = -35  # Pitch angle
        target_position = [0, 0, 0.1]  # Target position (near the bot)
        p.resetDebugVisualizerCamera(distance, yaw, pitch, target_position)

    def step(self, action):
        self.throttle(action)
        p.stepSimulation()
        self.observation = self.compute_obs()
        reward = self.comp_reward()
        done = self.is_done()

        self._Step_Counter += 1
        terminated = done
        truncated = self._Step_Counter >= 500
        time.sleep(0.001)
        return  np.array(self.observation), reward, terminated, truncated, {}

    def throttle(self, action):
        d_vel = 0.1
        delta_vel = [-15*d_vel,-10*d_vel,-8.*d_vel,-5.*d_vel, -3.*d_vel,-d_vel, -0.1*d_vel, 0,
                     0.1*d_vel,d_vel, 3.*d_vel,5.*d_vel, 8.*d_vel,10*d_vel,15*d_vel][action] 
        vt = self.vt + delta_vel # this changes the tyre velocity in order to find a desired vel for stability
        self.vt = vt
        p.setJointMotorControl2(bodyUniqueId=self.botId, 
                                jointIndex=0, 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocity=vt)
        p.setJointMotorControl2(bodyUniqueId=self.botId, 
                                jointIndex=1, 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocity=vt)
    
    def compute_obs(self):
        _, cubeOrin = p.getBasePositionAndOrientation(self.botId)

        cubeEuler = p.getEulerFromQuaternion(cubeOrin)

        linear_vel, angular_vel = p.getBaseVelocity(self.botId)
        return [cubeEuler[0], angular_vel[0], self.vt]

    def comp_reward(self):
        _, cubeOrin = p.getBasePositionAndOrientation(self.botId)
        cubeEuler = p.getEulerFromQuaternion(cubeOrin)
    
        return ((np.pi)/2 - abs(cubeEuler[0])) * 0.1 -  abs(self.vt - self.vd) * 0.01
    
    def is_done(self):
        cubePos, _ = p.getBasePositionAndOrientation(self.botId)

        return cubePos[2] < 0.1 or self._Step_Counter >= 500 # C.O.M of robot is below 0.1 means the robot has fallen 
    
    def reset(self,seed = None):
        self.vt = 0 # current velocity
        self.vd = 0 # desired velocity it is required in order to calculate reward/ loss. Initially we don't have desired velocity
        self._Step_Counter = 0 # it is the number of times action taken till the current state from initial point of a particular episode

        p.resetSimulation()
        p.setGravity(0, 0, -9.8) 
        p.setTimeStep(0.01) 

        # Set the path to the directory containing your URDF files
        path1 = "C:/Users/omswa/OneDrive/Desktop/gym-robotics/gym_robot/envs"
        planeId = p.loadURDF("plane.urdf")
        
        cubeStartPos = [0, 0, 0.001]
        cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
        
        # Load the turtle_bot URDF
        self.botId = p.loadURDF(os.path.join(path1, "turtle_bot.urdf"), cubeStartPos, cubeStartOrientation)

        # Initial observation
        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.botId)
        cubeEul = p.getEulerFromQuaternion(cubeOrn)
        self.observation = np.array([cubeEul[0]])

        self.observation = self.compute_obs()

        # Reset camera position
        self.set_camera()

        return np.array(self.observation),{}

    def render(self, mode='human'):
        pass
    
    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

# if __name__ == '__main__':
#     env = RoBots()
#     obs = env.reset()
    
#     while True:
#         keys = p.getKeyboardEvents()
#         if p.B3G_SPACE in keys and keys[p.B3G_SPACE] & p.KEY_WAS_TRIGGERED:
#             break
#         action = env.action_space.sample()  # Random torque actions for both wheels
#         obs, reward, done, info = env.step(action)
#         print(f"Action: {action}, Observation: {obs}, Reward: {reward}, Done: {done}")
#         if done:
#             obs = env.reset()


