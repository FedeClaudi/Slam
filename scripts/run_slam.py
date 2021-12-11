import matplotlib.pyplot as plt
from celluloid import Camera

from fcutils.progress import track

from slam import Wall, Agent


f, ax = plt.subplots(figsize=(16, 8))
cam = Camera(f)


# create environment
env = Wall()

# create agent
agent = Agent(env, x=30, y=5, angle=90)

# run simulation
for i in track(range(100)):
    # move/update agent
    agent.update()

    # re-draw environment
    env.draw(ax)

    # move and draw environment
    agent.draw(ax)

    # snap
    cam.snap()

animation = cam.animate()
animation.save("animation.mp4")
