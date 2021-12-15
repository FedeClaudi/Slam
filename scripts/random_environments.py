import matplotlib.pyplot as plt

from slam import Environment

f, axes = plt.subplots(figsize=(9, 9), ncols=3, nrows=3)
axes = axes.flatten()


for ax in axes:
    Environment().draw(ax)


f.tight_layout()
plt.show()
