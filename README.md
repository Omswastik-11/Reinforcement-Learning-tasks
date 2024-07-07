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
