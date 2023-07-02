# LRAUV Simulation

This repository contains the libraries, plugins and other files for the simulation of the Tethys-class Long-Range AUV (LRAUV) from the Monterey Bay Aquarium Research Institute (MBARI).

For documentation regarding this repository please refer to the [wiki](https://github.com/osrf/lrauv/wiki).

<p align="center">
  <img width="40%" src="https://raw.githubusercontent.com/wiki/osrf/lrauv/media/LRUAV_3D.gif" alt="LRAUV 3D">
</p>

Source files, models, and plugins relevant to a general audience are upstreamed on an irregular basis to [Gazebo libraries](https://gazebosim.org), the top-level library being [gz-sim](https://github.com/gazebosim/gz-sim). Upstreamed files may eventually be removed from the repository.

Standalone, the repository contains the environment and plugins necessary to simulate an underwater vehicle in Gazebo. Integrated with the real-world LRAUV controller code, the binaries of which are provided to the public on MBARI's DockerHub, the simulated robot can be controlled using the same code executed on the real robot. This enables the validation of scientific missions for oceanography research.

## Citations
If you use this simulator for your project, please cite our paper:

> Timothy R. Player, Arjo Chakravarty, Mabel M. Zhang, Ben Yair Raanan, Brian Kieft, Yanwu Zhang, and Brett Hobson, "From Concept to Field Tests: Accelerated Development of Multi-AUV Missions Using a High-Fidelity Faster-than-Real-Time Simulator," in *IEEE International Conference on Robotics and Automation (ICRA)*, May 2023.
