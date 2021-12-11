from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from typing import Union, Optional

from kino.geometry.point import Point
from kino.geometry import Vector


def distance(p1: Union[Point, Vector], p2: Union[Point, Vector]) -> float:
    return np.sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2)


class Line:
    def __init__(
        self,
        slope: float,
        intercept: float,
        point: Optional[Point] = None,
        color: str = "k",
        lw: float = 0.75,
    ):
        self.slope = slope
        self.intercept = intercept
        self.color = color
        self.lw = lw
        self.point = point

    def __repr__(self) -> str:
        return f"(Line) y = {self.slope:.2f}x + {self.intercept:.2f}"

    @classmethod
    def from_points(cls, p1: Point, p2: Point, **kwargs) -> Line:
        """ Computes the slope and intercept of a line through two points
        """

        delta_x = p2.x - p1.x
        delta_y = p2.y - p1.y

        if delta_x == 0:
            # vertical line
            slope = 1e4
            intercept = np.nan
            point = Point(float(p1.x), 0)
        elif delta_y == 0:
            # horizontal linte
            slope = 0
            intercept = p1.y
            point = Point(0, float(p1.y))
        else:
            # angled line
            slope = delta_y / delta_x
            intercept = p1.y - slope * p1.x
            point = p1
        return Line(float(slope), float(intercept), point=point, **kwargs)

    def intersection(self, other: Line) -> Optional[Point]:
        """
            Finds the point of intersection between two lines
        """
        # remove parallel lines
        if self.slope == other.slope:
            return None
        if self.slope == 1e4 and other.slope == 1e4:
            return None

        if self.slope == 1e4:
            # this line vertical
            x = self.point.x  # type: ignore
            y = other.slope * x + other.intercept
        elif other.slope == 1e4:
            # other line vertical
            x = other.point.x  # type: ignore
            y = self.slope * x + self.intercept
        else:
            # angled line
            x = -(other.intercept - self.intercept) / (
                other.slope - self.slope
            )
            y = self.slope * x + self.intercept
        return Point(x, y)

    def draw(self, ax: plt.Axes):
        ax.axline(
            (0, self.intercept),
            slope=self.slope,
            lw=self.lw,
            color=self.color,
            zorder=-1,
        )
