from typing import List, Dict
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from myterial import red

from slam.ray import Contact


class Map:
    def __init__(self, agent):
        self.agent = agent

        self.events: Dict[str, List[float]] = {
            **{"speed": [], "omega": []},
            **{str(ray.angle_shift): [] for ray in agent.rays},
        }

    def add(self, *events: Contact):
        """
            Given a list of ray-object contact events (in egocentric coordinates)
            stores the distance each ray picked up an object together witht he robot's
            kinematics
        """

        self.events["speed"].append(self.agent._current_speed)
        self.events["omega"].append(self.agent._current_omega)

        for ray in self.agent.rays:
            ray_events = [ev for ev in events if ev.ray == ray]
            if not ray_events:
                self.events[str(ray.angle_shift)].append(np.nan)
            else:
                self.events[str(ray.angle_shift)].append(
                    ray_events[0].distance
                )

    def build(self):
        """
            Integrates the stored robot motion to reconstruct the position of the dots
        """
        self.points_x: List[float] = []
        self.points_y: List[float] = []

        data = pd.DataFrame(self.events)

        # find first event timestep
        ray_names = [str(ray.angle_shift) for ray in self.agent.rays]
        first = data[ray_names].apply(pd.Series.first_valid_index).min()

        # drop rows before first event
        try:
            data = data[first:].reset_index()
        except TypeError:
            # no events detected
            return

        # reconstruct vehicle position at each time step
        x, y, theta = [0], [0], [0]
        for i, step in data.iterrows():
            thet_rad = np.radians(theta[-1])
            x.append(x[-1] + step.speed * np.cos(thet_rad))
            y.append(y[-1] + step.speed * np.cos(thet_rad))
            theta.append(theta[-1] + step.omega)

        # reconstruct the position of each sensore event
        for sensor in ray_names:
            delta = np.radians(float(sensor))
            for i, value in data[sensor].iteritems():
                if value is not None:
                    # get the position of the robot at that moment
                    _x, _y, _theta = x[i], y[i], np.radians(theta[i])

                    # get the position of the event
                    self.points_x.append(_x + np.cos(_theta + delta) * value)
                    self.points_y.append(_y + np.sin(_theta + delta) * value)

    def draw(self, ax: plt.Axes):
        self.build()

        # plot reconstructed
        ax.scatter(self.points_x, self.points_y, zorder=150, s=15, color=red)
