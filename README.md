# SLAM

Playing around with SLAM implementation in Python and Julia.

Initially the agent could navigate the environment and use LIDAR to pick up objects. The agent was told when an object was detected and at what distance, and it had knowledge of it's own movements (linear and angular velocity).
From these it can reconstruct a map of where the detected objects were and what it's location with respect to them was
![](SLAM.png)

Later, to create a more generally useful map, the agent assigns a positively valued gaussian to each point along the laser path for each LIDAR ray. Such gaussians are placed between the agent and the end of the ray or were the ray detects an object. Again the agent only know when these guassians are 'picked up' and their distance and can then reconstruct their location in the map.
![](SLAM2.png)


### Roadmap (working on Python only for now):
- [x] Environment
  - [x] create randomly populate environments with square objects
- [x] Agent
  - [x] LIDAR rays
    - [x] created LIDAR rays that can detect objects and report their distance to the agent
  - [x] navigation: agent goes in a straight line until LIDAR rays detect objects nearby, then steers to avoid them
    - [x] add movement routines (full 360 scan, turn scan, backtrack)
    - [ ] add fill in map gaps routine
- [ ] MAP
  - [x] points information is transformed into a coherent representation
  - [x] the agent is located within the points cloud
  - [ ] go from points to map
    - [x] assigns +/- gaussians along rays paths
    - [ ] use gaussians to crate map
    - [ ] use map to fill in gaps

### Bugs
- [ x agent escapes the environment occasionally (or enters an obstacle)
- [ ] agent is initialized in an obstacle in random Env
