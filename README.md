# SLAM

Playing around with SLAM implementation in Python and Julia.

![](SLAM.png)

### Roadmap (working on Python only for now):
- [x] Environment
  - [x] create randomly populate environments with square objects
- [x] Agent
  - [x] LIDAR rays
    - [x] created LIDAR rays that can detect objects and report their distance to the agent
  - [x] navigation: agent goes in a straight line until LIDAR rays detect objects nearby, then steers to avoid them
    - [x] add movement routines (full 360 scan, turn scan, backtrack)
- [ ] MAP
  - [x] points information is transformed into a coherent representation
  - [x] the agent is located within the points cloud
  - [ ] go from points to map

### Bugs
- [ ] agent escapes the environment occasionally (or enters an obstacle)
- [ ] agent is initialized in an obstacle in random Env

### TODO
- [x] fix ray intersection: 
  - [x] only positive range
  - [x] only segment and not line
  - [x] ceck with boundaries
- [ ] MAP remove movement and store stuff
- [ ] MAP plotting