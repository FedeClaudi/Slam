from typing import Optional, Union
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from numpy.random import uniform
import numpy as np

from kino.geometry import Vector
from kino.geometry.point import Point

from slam.obstacle import Obstacle
from slam.plot_utils import BACKGROUND_COLOR


class Environment:
    def __init__(
        self, width: int = 100, heigh: int = 100, n_obstacles: int = 5
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
        if not n_obstacles:
            return

        for n in range(n_obstacles):
            pt = self.random_point()

            self.obstacles.append(
                Obstacle(
                    xy=(pt.x, pt.y),
                    angle=uniform(0, 180),
                    width=uniform(20, 60),
                    height=uniform(5, 10),
                    name=f"Obj {n}",
                )
            )

    def random_point(self) -> Point:
        """
            Returns a random point that is not in an obstacle
        """
        while True:
            point = Point(
                np.random.uniform(10, self.width - 10),
                np.random.uniform(10, self.height - 10),
            )
            if not self.is_point_in_obstacle(point):
                break
        return point

    def is_point_in_obstacle(self, point: Union[Vector, Point]) -> bool:
        """
            Checks if a point is in any given obstacle
        """
        for obs in self.obstacles:
            if obs.contains(point):
                return True
        return False

    def out_of_bounds(self, point: Union[Vector, Point]) -> bool:
        """
            Checks if a point is out bounds (outside of environemnt)
        """
        if 0 < point.x < self.width and 0 < point.y < self.height:
            return False
        else:
            return True

    def draw(self, ax: Optional[plt.Axes] = None) -> plt.Axes:
        ax = ax or plt.subplots(figsize=(9, 9))[1]

        # set ax lim by plotting transparent boundaries
        ax.scatter(
            [-2.5, self.width + 2.5, self.width + 2.5, -2.5],
            [-2.5, -2.5, self.height + 2.5, self.height + 2.5],
            alpha=0,
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
    """ Environment with a few wa;;s
    """

    def __init__(self):
        super().__init__(100, 60, 1)
        self.obstacles = [
            Obstacle((20, 30), 0, 60, 4, "wall-1"),
            Obstacle((40, 34), 90, 10, 4, "wall-2"),
            Obstacle((80, 0), 90, 15, 4, "wall-3"),
        ] + self.walls


class Small(Environment):
    def __init__(self):
        super().__init__(30, 30, 0)


class BigBox(Environment):
    """ Environment with a big box obstacle
    """

    def __init__(self):
        super().__init__(60, 60, 1)
        self.obstacles = [Obstacle((20, 20), 0, 40, 40, "box")] + self.walls
