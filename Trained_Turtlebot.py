import gymnasium as gym
from gym_robot.envs.robo_env import RoBots
import pybullet as p
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.vec_env import DummyVecEnv


# Create your custom environment
env = RoBots()

# Check the environment
check_env(env, warn=True)

# Wrap the environment
env = DummyVecEnv([lambda: env])

# Create the PPO model
model = PPO("MlpPolicy", env, verbose=1)

# Train the model
model.learn(total_timesteps=100000)

# Save the model
model.save("ppo_robot")

# Load the trained model
model = PPO.load("ppo_robot")

# Run the trained model in the environment
obs = env.reset()
while True:
    keys = p.getKeyboardEvents()
    if p.B3G_SPACE in keys and keys[p.B3G_SPACE] & p.KEY_WAS_TRIGGERED:
        break
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, done, info = env.step(action)
    env.render()
    if done:
        obs = env.reset()


