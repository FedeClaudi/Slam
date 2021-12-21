import matplotlib.pyplot as plt
import numpy as np
from rich.prompt import Confirm
from loguru import logger

from fcutils.progress import track

from slam import Environment, Agent


# create env
env = Environment()

# create agent
agent = Agent(env, x=20, y=10, angle=np.random.uniform(10, 80))
agent.update_map_every = 1000

# check intial conditions
f, ax = plt.subplots(figsize=(10, 10))
env.draw(ax)
agent.draw(ax)
print("Is the starting configuration OK?")
plt.show()

if Confirm.ask("Continue?"):

    # run simulation
    for i in track(range(500)):
        # move/update agent
        agent.update()

        # check termination conditions
        if env.out_of_bounds(agent.COM):
            logger.warning("Agent out of bounds")
            break
        elif env.is_point_in_obstacle(agent.COM):
            logger.warning("Agent is in an obstacle")
            break

    agent.slam()

    # draw environment
    f, axes = plt.subplots(figsize=(20, 10), ncols=2)
    env.draw(axes[0])

    # draw agent and map
    agent.draw(axes[0])
    agent.map.draw(axes[1])
    agent.planner.draw(axes[1])

    axes[0].axis("equal")
    axes[1].axis("equal")
    axes[1].legend()

    axes[0].set(title="world view")
    axes[1].set(title="agent view")

    f.tight_layout()
    plt.show()
