from typing import Optional
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from numpy.random import uniform

from slam.obstacle import Obstacle
from slam.plot_utils import BACKGROUND_COLOR


class Environment:
    def __init__(
        self, width: int = 60, heigh: int = 60, n_obstacles: int = 10
    ):
        self.width = width
        self.height = heigh

        self.add_obtacles(n_obstacles)

        # create north sout east west walls
        self.walls = [
            Obstacle(
                (0, -4),
                0,
                width=self.width,
                height=4,
                name="south",
                txt_size=12,
            ),
            Obstacle(
                (self.width, -4),
                0,
                width=4,
                height=self.height + 8,
                name="east",
                txt_size=12,
            ),
            Obstacle(
                (0, self.height),
                0,
                width=self.width,
                height=4,
                name="north",
                txt_size=12,
            ),
            Obstacle(
                (-4, -4),
                0,
                width=4,
                height=self.height + 8,
                name="west",
                txt_size=12,
            ),
        ]
        self.obstacles += self.walls

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
                    width=uniform(5, 20),
                    height=uniform(5, 20),
                    name=f"Obj {n}",
                )
            )

    def draw(self, ax: Optional[plt.Axes] = None) -> plt.Axes:
        ax = ax or plt.subplots(figsize=(9, 9))[1]

        # set ax properties
        ax.set(
            xlim=[-2.5, self.width + 2.5],
            ylim=[-2.5, self.height + 2.5],
            xlabel="cm",
            ylabel="cm",
        )

        # ax.axis("equal")
        ax.axis("off")

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
        return ax


class Wall(Environment):
    """ Environment with a single wall obstacle
    """

    def __init__(self):
        super().__init__(100, 100, 1)
        self.obstacles = [
            Obstacle((20, 50), 0, 60, 5, "wall-1"),
            Obstacle((52, 55), 90, 45, 4, "wall-2"),
        ] + self.walls
