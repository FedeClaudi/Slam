import matplotlib.pyplot as plt

from slam import Environment, Agent
from slam.obstacle import Obstacle

f, axes = plt.subplots(figsize=(12, 9), ncols=3, nrows=3)
axes = axes.flatten()

# create environment
env = Environment(n_obstacles=0)
env.obstacles += [
    Obstacle((30, 30), angle=20, width=5, height=10, name="test")
]

# create agents
agents = [
    Agent(env, x=30, y=25, angle=90),  # wall
    Agent(env, x=25, y=25, angle=45),  # east
    # Agent(env, x=5, y=5, angle=180),  # west
    # Agent(env, x=20, y=5, angle=270),  # bottom
    # Agent(env, x=20, y=55, angle=90),  # top
    # Agent(env, x=55, y=25, angle=45),  # east
    # Agent(env, x=5, y=5, angle=225),  # west
    # Agent(env, x=20, y=5, angle=215),  # bottom
    # Agent(env, x=20, y=55, angle=45),  # top
]
for ax, agent in zip(axes, agents):

    # re-draw environment
    env.draw(ax)

    # move and draw environment
    agent.update()
    agent.draw(ax)


f.tight_layout()
plt.show()
