import matplotlib.pyplot as plt
from celluloid import Camera

from fcutils.progress import track

from slam import Environment, Agent


f, axes = plt.subplots(figsize=(20, 10), ncols=2)
cam = Camera(f)


# create environment
env = Environment(100, 100, n_obstacles=10)

# create agent
agent = Agent(env, x=2, y=2, angle=45)

# run simulation
for i in track(range(1500)):
    # move/update agent
    agent.update()

    if i % 5 == 0:
        # draw environment
        env.draw(axes[0])
        env.draw(axes[1])

        # move and draw agent and amp
        agent.draw(axes[0])
        agent.map.draw(axes[1])

        # snap
        cam.snap()

animation = cam.animate(interval=40 * 5)
animation.save("animation.mp4")
