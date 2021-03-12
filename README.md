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

## To test integration with MBARI LRAUV code base

Pull the private Docker image on [OSRF DockerHub](https://hub.docker.com/u/osrf)
containing Ignition, MBARI LRAUV code base, and this repository.

Once inside a container, source the colcon workspaces:
```
. ~/ign_ws/install/setup.bash
. ~/lrauv_ws/install/setup.bash
```
This needs to be done for each terminal.

### Basic test

Test by launching a world in the LRAUV Ignition simulation, for example:
```
ign launch lrauv_world.ign
```

Or run an executable the MBARI code base, for example:
```
cd ~/lrauv_ws/src/lrauv-application
bin/SimDaemon &
bin/LRAUV
```

### Run a mission

Run the MBARI LRAUV SimDaemon, or replace it with the Ignition simulation when
it is fully functional:
```
# Original LRAUV SimDaemon
bin/SimDaemon &

# And/or launch Ignition simulation
ign launch lrauv_world.ign
```

Run the vehicle code:
```
bin/LRAUV
```
This will bring you to a command prompt.

At the LRAUV command prompt:
```
>configset micromodem.loadatstartup 0 bool persist
>restart app
```
This will pause for a bit, you might not be able to type right away.

Speed up loading to 100 times the speed for a bit, before returning to normal
speed:
```
>quick on
>quick off
```

Verify that it is running the default `GoToSurface` app:
```
>show stack
2021-03-03T18:24:46.699Z,1614795886.699 [Default](IMPORTANT): Priority 0: Default:B.GoToSurface
```
An app is always being run.
If no missions are specified, then it is running the default.

Load the yoyo mission, which will go into the water:
```
>load Science/profile_station.xml
```

Set the max depth to a desired number, and run the mission:
```
>set profile_station.yoyomaxdepth 20 meter
>run;quick off
```

You should see the depth increasing to the max depth set above, and no more:
```
>report touch depth
>quick on
```

To clear the report and go back to normal speed:
```
>report clear
>quick off
```

To stop the mission and terminate:
```
>stop
>quit
```

### Run the circle mission

Following the same basic steps as above, change the mission file:
```
>load Engineering/circle_test.xml
>set circle_test.Depth01 10 meter;set circle_test.Depth02 15 meter;set circle_test.RudderAngle01 15 degree;set circle_test.RudderAngle02 10 degree;set circle_test.WaitDuration 10 minute
>run;quick off
```

### Other operation commands

Control commands can be issued.
For example, the rudder can be controlled like so:
```
>maintain control horizontalcontrol.rudderangleaction 15 degree
```
This overwrites the controller and maintains the rudder at 15 degrees.

A sample list of command variables:
```
Config/Control-->HorizontalControl.loadAtStartup=1 bool
Config/Control-->HorizontalControl.kdHeading=0.049999 s
Config/Control-->HorizontalControl.kiHeading=0.001000 1/s
Config/Control-->HorizontalControl.kiwpHeading=0.000500 rad/s/m
Config/Control-->HorizontalControl.kpHeading=0.400000 n/a
Config/Control-->HorizontalControl.kwpHeading=0.049999 rad/m
Config/Control-->HorizontalControl.maxHdgAccel=7.499876 arcdeg/s2
Config/Control-->HorizontalControl.maxHdgInt=0.087266 rad
Config/Control-->HorizontalControl.maxHdgRate=11.999932 arcdeg/s
Config/Control-->HorizontalControl.maxKxte=45.000001 arcdeg
Config/Control-->HorizontalControl.rudDeadband=0.500000 arcdeg
Config/Control-->HorizontalControl.rudLimit=15.000000 arcdeg
VerticalControl-->VerticalControl.buoyancyAction=944.986938 cc
VerticalControl-->VerticalControl.depthIntegralInternal=nan rad
VerticalControl-->VerticalControl.depth2buoyIntInternal=nan cc
VerticalControl-->VerticalControl.massIntegralInternal=nan m
VerticalControl-->VerticalControl.elevatorIntegralInternal=nan rad
HorizontalControl-->HorizontalControl.rudderAngleAction=0.000000 rad
SpeedControl-->SpeedControl.propOmegaAction=0.000000 rad/s
```

### LRAUV cheat sheet

This contains some most-often used commands for quick reference:

Show general help or for a specific command:
```
>?
>help report
```

Show mission currently being run
```
>show stack
```

Speed up 100 times faster than real time:
```
>quick on
# To go back to normal speed
>quick off
```

To report the value continuously on variable touch:
```
>report touch <componentName>.<variableName>
# To stop reporting
>report clear
```

To get the current value of a variable:
```
>get <componentName>.<variableName>
```

A sample list of variables in the `ExternalSim` component:
```
ExternalSim.latitudeSim=36.803400 arcdeg
ExternalSim.longitudeSim=-121.822200 arcdeg
ExternalSim.eastingSim=605067.311028 m
ExternalSim.northingSim=4073710.248871 m
ExternalSim.utmZoneSim=10 enum
ExternalSim.propThrustSim=-0.000000 N
ExternalSim.propTorqueSim=-0.000000 N-m
ExternalSim.netBuoySim=0.000000 N
ExternalSim.forceXSim=0.000000 N
ExternalSim.forceYSim=0.000000 N
ExternalSim.forceZSim=0.000000 N
ExternalSim.posXSim=0.000000 m
ExternalSim.posYSim=0.000000 m
ExternalSim.posZSim=0.000000 m
ExternalSim.rollSim=0.000000 rad
ExternalSim.pitchSim=0.000000 rad
ExternalSim.headingSim=0.000000 rad
ExternalSim.posXDotSim=0.000000 m
ExternalSim.posYDotSim=0.000000 m
ExternalSim.posZDotSim=0.000000 m
ExternalSim.rateUSim=0.000000 m/s
ExternalSim.rateVSim=0.000000 m/s
ExternalSim.rateWSim=0.000000 m/s
ExternalSim.ratePSim=0.000000 m/s
ExternalSim.rateQSim=0.000000 m/s
ExternalSim.rateRSim=0.000000 m/s
ExternalSim.homingSensorRangeSim=27.335945 m
ExternalSim.homingSensorAzimSim=-1.531450 rad
ExternalSim.homingSensorElevSim=1.073800 rad
```

To stop a mission:
```
>stop
```

To terminate:
```
>quit
```
