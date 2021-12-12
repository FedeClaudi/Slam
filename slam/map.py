from typing import List
import matplotlib.pyplot as plt

from myterial import red
from kino.geometry import Vector
from kino.geometry.point import Point

from slam.ray import Contact


class Map:
    def __init__(self, agent):
        self.agent = agent

        self.events: List[Point] = []  # reconstructed XY coords of each event
        self.events_ground_truth: List[Contact] = []

    def add(self, *events: Contact):
        """
            Given a list of ray-object contact events (in egocentric coordinates)
            corrects by the agent's posture to 
        """
        # store ground-truth events location
        self.events_ground_truth.extend(events)

        # go egocentric -> allocentric
        for event in events:
            pos = event.position
            pos = pos.rotate(self.agent.angle)
            pos = pos + Vector(*self.agent.head_position)
            self.events.append(Point(pos.x, pos.y))

    def draw(self, ax: plt.Axes):
        # plot ground truth events
        x = [ev.point.x for ev in self.events_ground_truth]
        y = [ev.point.y for ev in self.events_ground_truth]

        ax.scatter(x, y, zorder=150, s=20, color="k")

        # plot reconstructed
        x = [ev.x for ev in self.events]
        y = [ev.y for ev in self.events]
        ax.scatter(x, y, zorder=150, s=15, color=red)
