from typing import List, Tuple, Dict, Optional
import matplotlib.pyplot as plt
import numpy as np
from dataclasses import dataclass

from kino.geometry import Vector
from kino.geometry.point import Point
from kino.geometry.interpolation import lerp
from myterial import salmon_dark, blue_light

from slam.geometry import Line, distance, segments_intersection
from slam.obstacle import Obstacle


class Ray:
    """
        A lidar ray.
    """

    def __init__(
        self,
        agent,
        angle: float,  # relative to agent orientation
        length: int,
    ):
        self.agent = agent
        self.angle_shift = angle
        self.length = length

        self.events_count = 0  # count every time an object is detected

        self.sampled_distance: List[float] = [
            self.length * p for p in np.linspace(0, 1, 5)
        ]  # distance values from start to end

    @property
    def angle(self) -> float:
        return self.agent.angle + self.angle_shift

    @property
    def p0(self) -> Point:
        """ position of origin of ray """
        return Point(*self.agent.head_position)

    @property
    def p1(self) -> Point:
        """
            Position of point at end of ray
        """
        x = self.length * np.cos(np.radians(self.angle)) + self.p0.x
        y = self.length * np.sin(np.radians(self.angle)) + self.p0.y
        return Point(x, y)

    @property
    def line(self) -> Line:
        """  Line going through ray points
        """
        return Line.from_points(self.p0, self.p1, color=salmon_dark)

    def sample(self, n: int = 5) -> List[Point]:
        """
            Sample n points along the ray from p0 to p1, included.
            Returns points carrying information of their distance along the ray as well/
        """
        pts: List[Point] = []
        for p in np.linspace(0, 1, n):
            pt = Point(
                lerp(self.p0.x, self.p1.x, p), lerp(self.p0.y, self.p1.y, p)
            )
            pt.distance = lerp(0, self.length, p)
            pts.append(pt)
        return pts

    def sample_at_distance(self, distance: float) -> Optional[Point]:
        """
            Returns a point at a sampled distance along the ray
        """
        if distance == 0:
            factor: float = 0
        elif distance > self.length:
            return None
        else:
            factor = distance / self.length

        pt = Point(
            lerp(self.p0.x, self.p1.x, factor),
            lerp(self.p0.y, self.p1.y, factor),
        )
        pt.distance = distance
        return pt

    def scan(self, obstacles: List[Obstacle]):
        """
            Scans through a list of objects to find intersections
        """
        intersection_points: List[Tuple[Point, float, Obstacle]] = []

        for obj in obstacles:
            # check if the object is closer than the ray length
            dist = distance(self.p0, obj.COM)
            if dist - (1.5 * obj.size) > self.length:
                continue

            # get the interesection between the ray line and
            # each edge-line of the obstacle
            obj_intersections: Dict[str, Tuple[Point, float, Obstacle]] = {}
            for name, line in obj.lines.items():
                # get vertice points
                q0, q1 = obj.points[name[0]], obj.points[name[1]]
                intersection = segments_intersection(self.p0, self.p1, q0, q1)
                if intersection is None:
                    continue

                # get distance from intersection
                dist = distance(self.p0, intersection)

                # all checks passed, keep point
                obj_intersections[name] = (intersection, dist, obj)

            if len(obj_intersections):
                closest = np.argmin([v[1] for v in obj_intersections.values()])
                intersection_points.append(
                    list(obj_intersections.values())[closest]
                )

        # keep the closest intersection points
        if intersection_points:
            closest = np.argmin([v[1] for v in intersection_points])
            contact_point = list(intersection_points)[closest]
            self.contact_point = Contact(  # type: ignore
                self,
                contact_point[2],
                contact_point[0],  # contact in allocentric coordinates
                Vector(  # contact in egocentrinc coordinates
                    contact_point[0].x - self.p0.x,
                    contact_point[0].y - self.p0.y,
                ).rotate(-self.agent.angle),
                contact_point[1],  # distances
            )
            self.events_count += 1
        else:
            self.contact_point = None  # type: ignore

    def draw(self, ax: plt.Axes):
        p0 = self.p0
        p1 = self.p1

        ax.plot(
            [p0.x, p1.x],
            [p0.y, p1.y],
            lw=2,
            ls=":",
            color=salmon_dark,
            zorder=99,
        )

        if self.contact_point is not None:
            ax.scatter(
                self.contact_point.point.x,
                self.contact_point.point.y,
                s=60,
                lw=1,
                ec="k",
                color=blue_light,
                zorder=100,
            )


@dataclass
class Contact:
    """ A contact point between a Ray and an Obstacle
    """

    ray: Ray
    obstacle: Obstacle
    point: Point  # coordinates in word space
    position: Vector  # coordinates in egocentric space
    distance: float
