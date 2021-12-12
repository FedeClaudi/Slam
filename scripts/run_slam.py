import matplotlib.pyplot as plt

from fcutils.progress import track

from slam import Environment, Agent


f, axes = plt.subplots(figsize=(20, 10), ncols=2)


# create environment
env = Environment(100, 100, n_obstacles=20)

# create agent
agent = Agent(env, x=10, y=10, angle=90)

# run simulation
for i in track(range(500)):
    # move/update agent
    agent.update()


# draw environment
env.draw(axes[0])
# env.draw(axes[1])

# move and draw agent and amp
agent.draw(axes[0])
agent.map.draw(axes[1])

axes[1].set(
    xlim=[-2, env.width + 2],
    ylim=[-2, env.height + 2],
    xlabel="cm",
    ylabel="cm",
)

plt.show()
