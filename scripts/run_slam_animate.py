import matplotlib.pyplot as plt
from celluloid import Camera

from fcutils.progress import track

from slam import Wall, Agent


DRAW_EVERY = 1

f, axes = plt.subplots(figsize=(20, 10), ncols=2)
cam = Camera(f)


# create environment
# env = Environment(100, 100, n_obstacles=10)
env = Wall()


# create agent
# agent = Agent(env, x=2, y=2, angle=45)
agent = Agent(env, x=20, y=8, angle=90)


# run simulation
for i in track(range(100)):
    # move/update agent
    agent.update()

    if i % DRAW_EVERY == 0:
        # draw environment
        env.draw(axes[0])

        # move and draw agent and amp
        agent.draw(axes[0])
        agent.map.draw(axes[1])

        # snap
        cam.snap()

animation = cam.animate(interval=40 * DRAW_EVERY)
animation.save("animation.mp4")
