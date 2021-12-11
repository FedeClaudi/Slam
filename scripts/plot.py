import matplotlib.pyplot as plt

from slam import Wall, Agent


f, ax = plt.subplots(figsize=(16, 8))

# create environment
env = Wall()

# create agent
agent = Agent(env, x=10, y=25, angle=270)

# re-draw environment
env.draw(ax)

# move and draw environment
agent.draw(ax)

plt.show()
