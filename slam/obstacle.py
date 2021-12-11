from typing import Tuple
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from kino.geometry import Vector
from kino.geometry.point import Point

from slam.plot_utils import outline
from slam.geometry import Line


class Obstacle:
    def __init__(
        self,
        xy: Tuple[float, float],
        angle: float,
        width: float,
        height: float,
        name: str,
    ):
        self.xy = xy
        self.angle = angle
        self.width = width
        self.height = height
        self.name = name

        # compute the position of vertices
        self.com = Vector(*xy)

        self.A_offset = Vector(0, 0)
        self.B_offset = Vector(0, self.height)
        self.C_offset = Vector(self.width, self.height)
        self.D_offset = Vector(self.width, 0)

        self.A = Point(*(self.com + self.A_offset.rotate(angle)).xy)
        self.B = Point(*(self.com + self.B_offset.rotate(angle)).xy)
        self.C = Point(*(self.com + self.C_offset.rotate(angle)).xy)
        self.D = Point(*(self.com + self.D_offset.rotate(angle)).xy)

        self.points = {
            name: pt
            for name, pt in zip("ABCD", (self.A, self.B, self.C, self.D))
        }

        # compute lines connecting vertices
        self.lines = dict(
            AB=Line.from_points(self.A, self.B),
            BC=Line.from_points(self.B, self.C),
            CD=Line.from_points(self.C, self.D),
            DA=Line.from_points(self.D, self.A),
        )

    def __repr__(self) -> str:
        return f"(Obstacle: {self.name}) - {self.points}"

    def draw(self, ax: plt.Axes):
        ax.add_artist(
            Rectangle(
                self.xy,
                self.width,
                self.height,
                self.angle,
                color=[0.2, 0.2, 0.2],
                hatch="/////",
                fill=False,
                lw=2,
            )
        )

        # add vertices names
        for name, point in self.points.items():
            outline(ax.text(point.x, point.y, name, zorder=200), lw=4)

        # draw lines
        for line in self.lines.values():
            line.draw(ax)
