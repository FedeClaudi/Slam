import matplotlib.pyplot as plt
import numpy as np
from loguru import logger

from fcutils.progress import track

from slam import Environment, Agent


# create environment
env = Environment()

# create agent
agent = Agent(env, x=20, y=10, angle=np.random.uniform(10, 80))


# run simulation
for i in track(range(100)):
    # move/update agent
    agent.update()

    # check termination conditions
    if env.out_of_bounds(agent.COM):
        logger.warning("Agent out of bounds")
        break
    elif env.is_point_in_obstacle(agent.COM):
        logger.warning("Agent is in an obstacle")
        break

    # if 'navigate' in agent._current_routine.name:
    #     break

# bagent SLAM - build map and planner helpers
agent.slam()

# draw environment
f, axes = plt.subplots(figsize=(20, 8), ncols=2)
env.draw(axes[0])

# draw agent, map and planner
agent.draw(axes[0])
agent.map.draw(axes[1])
agent.planner.draw(axes[1])

# clean axes
axes[0].axis("equal")
axes[1].axis("equal")


axes[0].set(title="world view")
axes[1].set(title="agent view")

f.tight_layout()
plt.show()
