# 🤖 Turtle Bot Reinforcement Learning Project

## Overview
This project focuses on training a 2-wheeled bot, known as Turtle Bot, to balance for as long as possible without falling using reinforcement learning techniques, specifically with the Proximal Policy Optimization (PPO) algorithm from the Stable Baselines3 library. The Turtle Bot environment is created using PyBullet and Gymnasium, allowing for simulating the robot's movements and interactions in a physics-based environment.

## 🛠️ Installation

### Prerequisites
Ensure you have Python 3.7+ installed. The following libraries are required:

- Gymnasium
- PyBullet
- Stable Baselines3
- NumPy


## 🎯 Problem Statement
The goal of this project is to create an environment for training a 2-wheeled bot, known as Turtle Bot, to balance for as long as possible without falling. This problem is formulated as a Markov Decision Process (MDP).

### Key Objectives

| Component | Description |
|-----------|-------------|
| State Space | Robot's orientation, angular velocity, and tire velocities |
| Action Space | Control actions for the two tires' velocities |
| Reward Function | Encourages balancing without falling |
| Simulation | PyBullet physics-based environment |

## 🔧 Environment Details
The custom Turtle Bot environment is defined in `robo_env.py` and includes:

### Components
- **Action Space**: One discrete action controlling tire velocities
- **Observation Space**: Array containing:
  - Robot orientation
  - Angular velocity
  - Average tire velocity
- **Rewards**: Based on:
  - Robot orientation
  - Deviation from desired velocity
- **Termination Conditions**:
  - Robot falls
  - Maximum steps reached

**Training Results**

Two demonstration videos showcase the robot's performance:
  - Robot_before_training.mp4: Shows the untrained robot's unstable behavior
  - Robot_after_training.mp4: Demonstrates successful balancing after PPO training
The training process transforms the robot from an unstable state to achieving sustained balance, validating the effectiveness of the PPO algorithm and the designed reward structure.

## Acknowledgments
- [Gymnasium](https://gymnasium.farama.org/) - Reinforcement learning environment framework
- [PyBullet](https://pybullet.org/) - Physics simulation
- [Stable Baselines3](https://stable-baselines3.readthedocs.io/) - PPO implementation


