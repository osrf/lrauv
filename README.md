# LRAUV Simulation

This repository contains packages for simulating the MBARI Tethys.

## Instructions to build

Make sure you have [ignition-dome](https://ignitionrobotics.org/docs/dome) and
[colcon](https://colcon.readthedocs.io/en/released/). 

Clone this repository then run
```
colcon build
```

## Instructions to test out

This package comes with an empty example world. To run this example world simply
source the colcon workspace and run:
```
. install/setup.bash
ign launch lrauv_world.ign
```

To run the integration world for testing communication with the LRAUV code base:
```
ign gazebo -v 4 -r tethys_comm.sdf
```

## Using docker

You may also choose to use docker for convenience. Make sure you have 
a recent version of [docker](https://docs.docker.com/) and 
[nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)
installed. Next to get started simply run the following command.
```
docker/build_and_run_docker.sh
```
