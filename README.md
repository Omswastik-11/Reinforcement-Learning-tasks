
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

