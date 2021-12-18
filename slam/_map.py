import matplotlib.pyplot as plt
from dataclasses import dataclass

from myterial import red_dark, blue_dark


@dataclass
class Gaussian:
    """
        Stores information about a gaussian representings confidence of the availability of a point
        along a laser with a given angle_delta and at a certain distance from the robot.
        The gaussian is defined by the mean (>0 for open spots and <0 otherwise) and std.
    """

    mean: float
    std: float
    distance: float
    angle_delta: int

    def draw(self, ax: plt.Axes):
        ax.add_artist(
            plt.Circle(
                self.point.xy,  # type: ignore
                1,
                color=blue_dark if self.mean < 0 else red_dark,
                alpha=0.2 if self.mean > 0 else 1,
                zorder=-1 if self.mean > 0 else 1,
            )
        )


@dataclass
class GridPoint:
    x: float
    y: float
    value: float = 0.1
    confidence_threshold: float = 1.5  # only map points with confidence > this are trusted

    @property
    def confidence(self) -> int:
        """
            Returns the confidence about it
            being an open spot (-1 for obstacle, 1 for certainly open otherwise 0)
        """
        if self.value < 0:
            return -1
        elif self.value < self.confidence_threshold:
            return 0
        else:
            return 1
