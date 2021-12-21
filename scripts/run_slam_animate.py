import matplotlib.pyplot as plt
from celluloid import Camera
import numpy as np
from loguru import logger
from rich.prompt import Confirm

from fcutils.progress import track

from slam import Environment, Agent


DRAW_EVERY = 1
SLAM_EVERY = 5
N_FRAMES = 500

# create Environment
env = Environment()

# create agent
agent = Agent(env, x=20, y=5, angle=np.random.uniform(10, 80))
agent.update_map_every = SLAM_EVERY

# check intial conditions
f, ax = plt.subplots(figsize=(10, 10))
env.draw(ax)
agent.draw(ax)
print("Is the starting configuration OK?")
plt.show()

if Confirm.ask("Continue?"):
    f, axes = plt.subplots(figsize=(20, 10), ncols=2)
    cam = Camera(f)
    f.tight_layout()

    # run simulation
    for i in track(range(N_FRAMES)):
        # move/update agent
        agent.update()

        if i % DRAW_EVERY == 0:
            # draw environment
            env.draw(axes[0])

            # move and draw agent and amp
            agent.map.build()
            agent.draw(axes[0])
            agent.map.draw(axes[1])
            agent.planner.draw(axes[1])

            # snap
            cam.snap()

            # check termination conditions
            if env.out_of_bounds(agent.COM):
                logger.warning("Agent out of bounds")
                break
            elif env.is_point_in_obstacle(agent.COM):
                logger.warning("Agent is in an obstacle")
                break

    axes[0].set(title="world view")
    axes[1].set(title="agent view")

    animation = cam.animate(interval=40 * DRAW_EVERY)
    animation.save("animation.mp4")
