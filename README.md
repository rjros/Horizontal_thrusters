# PX4 Autopilot modified for fully actuated system (Work in Progress)

Modified version of the release version v1.14.0 of the PX4-Autopilot firmware, using the existing control allocation method.

* Modify the controllers in the system to allow for flight with no tilt.
* Consider semi-actuated systems (have limited control on some axes and can produce 3D forces in others)
* Include the simulation documents for gazebo for the frames
* Tested in the pixhawk 6C

## Acknowledgment
The early version of this project started with inspiration from work published by the Airlab group and their implementation of a fully actuated multirotor shown [here](https://github.com/castacks/PX4-fully-actuated).
