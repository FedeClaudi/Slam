import matplotlib.pyplot as plt


from slam import Environment, Agent
from slam.obstacle import Obstacle

f, axes = plt.subplots(figsize=(12, 9), ncols=2, nrows=2)
axes = axes.flatten()

# create environment
env = Environment(n_obstacles=0)
env.obstacles += [
    Obstacle((30, 30), angle=20, width=5, height=10, name="test")
]

# create agents
agents = [
    Agent(env, x=30, y=25, angle=90),
    Agent(env, x=25, y=25, angle=45),
    Agent(env, x=45, y=40, angle=180),
    Agent(env, x=28, y=50, angle=270),
]
for ax, agent in zip(axes, agents):

    # re-draw environment
    env.draw(ax)

    # move and draw environment
    agent.update()
    agent.draw(ax)


f.tight_layout()
plt.show()
