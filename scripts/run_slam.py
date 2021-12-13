import matplotlib.pyplot as plt
import numpy as np

from fcutils.progress import track

from slam import Environment, Agent


f, axes = plt.subplots(figsize=(20, 10), ncols=2)


# create environment
# env = Environment(100, 100, n_obstacles=10)
env = Environment()

# create agent
agent = Agent(env, x=20, y=10, angle=np.random.uniform(10, 80))

# run simulation
for i in track(range(1000)):
    # move/update agent
    agent.update()

# draw environment
env.draw(axes[0])

# draw agent and map
agent.draw(axes[0])
agent.map.draw(axes[1])

axes[0].axis("equal")
axes[1].axis("equal")


f.tight_layout()
plt.show()
