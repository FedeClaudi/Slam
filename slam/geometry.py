from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from typing import Union, Optional

from kino.geometry.point import Point
from kino.geometry import Vector
from kino.geometry.interpolation import lerp


def distance(p1: Union[Point, Vector], p2: Union[Point, Vector]) -> float:
    return np.sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2)


def segments_intersection(
    p0: Point, p1: Point, q0: Point, q1: Point
) -> Optional[Point]:
    """ Checks for intersections between two line segments defined by two sets of 2 points.
        https://blogs.sas.com/content/iml/2018/07/09/intersection-line-segments.html
    """
    # get parameterrized segments intersection parameter values
    A = np.array([[p1.x - p0.x, q0.x - q1.x], [p1.y - p0.y, q0.y - q1.y]])
    b = np.array([q0.x - p0.x, q0.y - p0.y])
    x_hat = np.linalg.lstsq(A, b.T, rcond=None)[0]

    # check that params in unit square
    if np.any(x_hat < 0) or np.any(x_hat > 1):
        return None

    # get intersection point
    x = lerp(p0.x, p1.x, x_hat[0])
    y = lerp(p0.y, p1.y, x_hat[0])

    return Point(x, y)


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
