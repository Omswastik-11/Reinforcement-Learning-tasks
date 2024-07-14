Multi-Agent Environment: Robber and Cop
Overview
This environment involves two agents: a Robber and a Cop. The Robber aims to reach specific checkpoints and eventually the goal while avoiding the Cop. The Cop aims to catch the Robber. The environment is designed to be used with reinforcement learning algorithms, focusing on the interaction and strategies between the agents.
Agents
Robber
Objective:
Reach checkpoints and the final goal.
Reward:
Each step: -1
Reaching a checkpoint: +30
Reaching the final goal: +100
Penalty:
Being caught by the Cop: -100
If caught, the episode terminates.
Cop
Objective:
Catch the Robber.
Reward:
Each step: -1
Catching the Robber: +100
Penalty:
Not applicable directly, but prolonged episodes without catching the Robber result in accumulated step penalties.
Environment Dynamics
Grid Layout
The environment is represented as a grid or maze.
The positions of the Robber, Cop, checkpoints, and the final goal are initialized as specified.
Actions
Both agents can perform actions to move in the grid.
The available actions include: up, down, left, right.
Actions are executed simultaneously for both agents.
State Representation
The state space includes the positions of both the Robber and the Cop, as well as the positions of checkpoints and the goal.
Termination Conditions
Robber
The episode ends if the Robber reaches the final goal.
The episode ends if the Robber is caught by the Cop.
Cop
The episode ends if the Robber is caught.
The episode ends if a predefined maximum number of steps is reached.
Rewards and Penalties Summary
Robber
Step: -1 per step.
Checkpoint: +30 for each checkpoint reached.
Goal: +100 for reaching the final goal.
Caught by Cop: -100 and episode termination.
Cop
Step: -1 per step.
Catching Robber: +100 and episode termination.
Visualization and Colors
The environment is visualized using the following color scheme:
Paths: White (255, 255, 255)
Walls: Black (0, 0, 0)
Start Position: Green (0, 255, 0)
Goal Position: Red (255, 0, 0)
Checkpoints: Blue (0, 0, 255)
Robber: Yellow (255, 255, 0)
Cop: Orange (255, 165, 0)















Turtle Bot Reinforcement Learning Project :

This project focuses on training a 2-wheeled bot, known as Turtle Bot, to balance for as long as possible without falling using reinforcement learning techniques, specifically with the Proximal Policy Optimization (PPO) algorithm from the Stable Baselines3 library. The Turtle Bot environment is created using PyBullet and Gymnasium, allowing for simulating the robot's movements and interactions in a physics-based environment.


Installation
Prerequisites
Ensure you have Python 3.7+ installed. The following libraries are required:

Gymnasium
PyBullet
Stable Baselines3
NumPy
You can install these dependencies using pip:



Problem Statement
The goal of this project is to create an environment for training a 2-wheeled bot, known as Turtle Bot, to balance for as long as possible without falling. This problem is formulated as a Markov Decision Process (MDP). The task involves defining the state space, action space, reward function, and other necessary components, so that the reinforcement learning algorithm can train the bot to achieve the goal.

Key Objectives:
State Space: Define the state space representing the bot's state, including orientation, angular velocity, and tire velocities.
Action Space: Define the action space representing the control actions, specifically the velocities for the two tires.
Reward Function: Design a reward function to encourage the bot to balance without falling.
Simulation Environment: Use PyBullet to simulate the bot's physics and movements.
Environment Details
The custom Turtle Bot environment is defined in robo_env.py and includes the following features:

Action Space: One discrete action controlling the velocities of the two tires.
Observation Space: An array containing the robot's orientation, angular velocity, and the average tire velocity.
Rewards: Calculated based on the robot's orientation and deviation from the desired velocity.
Termination: The episode ends if the robot falls or a maximum number of steps is reached.



Acknowledgments
Gymnasium for the reinforcement learning environment framework.
PyBullet for the physics simulation.
Stable Baselines3 for the PPO implementation.

