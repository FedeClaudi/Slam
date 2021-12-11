from typing import Optional
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from numpy.random import uniform

from slam.obstacle import Obstacle
from slam.plot_utils import BACKGROUND_COLOR


class Environment:
    def __init__(self, width: int, heigh: int, n_obstacles: int = 10):
        self.width = width
        self.height = heigh

        self.add_obtacles(n_obstacles)

    def add_obtacles(self, n_obstacles: int):
        self.obstacles = []
        for n in range(n_obstacles):
            self.obstacles.append(
                Obstacle(
                    xy=(
                        uniform(10, self.width - 10),
                        uniform(10, self.height - 10),
                    ),
                    angle=uniform(0, 360),
                    width=uniform(1, 5),
                    height=uniform(1, 5),
                    name=f"Obj {n}",
                )
            )

    def draw(self, ax: Optional[plt.Axes] = None) -> plt.Axes:
        ax = ax or plt.subplots(figsize=(16, 8))[1]
        ax.axis("equal")

        # draw environment
        ax.add_artist(
            Rectangle(
                (0, 0),
                self.width,
                self.height,
                facecolor=BACKGROUND_COLOR,
                edgecolor=[0.3, 0.3, 0.3],
                lw=2,
                zorder=-100,
            )
        )

        # draw obtacles
        for obstacle in self.obstacles:
            obstacle.draw(ax)

        # set ax properties
        ax.set(
            xlim=[-10, self.width + 10],
            ylim=[-10, self.height + 10],
            xlabel="cm",
            ylabel="cm",
        )

        return ax


class Wall(Environment):
    """ Environment with a single wall obstacle
    """

    def __init__(self):
        super().__init__(60, 40, 1)
        self.obstacles = [Obstacle((4, 30), 0, 50, 5, "wall")]
