import matplotlib.pyplot as plt

from slam import Wall, Agent


f, axes = plt.subplots(figsize=(12, 9), ncols=3, nrows=3)
axes = axes.flatten()

# create environment
env = Wall()


# create agents
agents = [
    Agent(env, x=30, y=25, angle=90),  # wall
    Agent(env, x=55, y=25, angle=0),  # east
    Agent(env, x=5, y=5, angle=180),  # west
    Agent(env, x=20, y=5, angle=270),  # bottom
    Agent(env, x=20, y=36, angle=90),  # top
    Agent(env, x=55, y=25, angle=45),  # east
    Agent(env, x=5, y=5, angle=225),  # west
    Agent(env, x=20, y=5, angle=215),  # bottom
    Agent(env, x=20, y=36, angle=45),  # top
]
for ax, agent in zip(axes, agents):

    # re-draw environment
    env.draw(ax)

    # move and draw environment
    agent.draw(ax)


f.tight_layout()
plt.show()
