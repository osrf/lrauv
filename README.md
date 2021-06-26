# LRAUV Simulation

This repository contains packages for simulating the MBARI Tethys.

## Instructions to build

Make sure you have [ignition-fortress](https://ignitionrobotics.org/docs/fortress) and
[colcon](https://colcon.readthedocs.io/en/released/), on Ubuntu Focal or higher.

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

Send test commands to move some joints:
```
./build/lrauv_ignition_plugins/TestController
```

Keyboard teleop:
```
./build/lrauv_ignition_plugins/TeleopController
```

## Using docker

You may also choose to use docker for convenience. Make sure you have
a recent version of [docker](https://docs.docker.com/) and
[nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)
installed. Next to get started simply run the following command.
```
docker/build_and_run_docker.sh
```

To join in a separate terminal, remember to source Ignition and the workspace:
```
docker/join.sh mbari_lrauv
. /home/ign_ws/install/setup.bash
. /home/colcon_ws/install/setup.bash
```

## To test integration with MBARI LRAUV code base

Pull the Docker image on the [MBARI DockerHub](https://hub.docker.com/repository/docker/mbari/lrauv-ignition-sim) containing Ignition, MBARI LRAUV code base, and this repository.

Once inside a container, source the colcon workspaces:
```
. ~/ign_ws/install/setup.bash
. ~/lrauv_ws/install/setup.bash
```
This needs to be done for each terminal.

### Setting up for a run

Launch the Ignition simulation:
```
ign launch lrauv_world.ign
```
For verbose debug output, add `--verbose 4`.

Launch the MBARI command prompt:
```
cd ~/lrauv_ws/src/lrauv-application
bin/LRAUV
```

At the LRAUV command prompt:
```
>configset micromodem.loadatstartup 0 bool persist
>restart app
```
This sets the micromodem to not load at startup. `persist` means you only need
to do this once.
It will pause for a bit, you might not be able to type right away.

Speed up 100 times for a bit to finish loading, before returning to normal
speed.
This allows the commands to finish loading, before you overwrite them with
control commands.
Otherwise the preloaded commands can kick in after you issue control commands
and make the vehicle go to unexpected places
```
>quick on
>quick off
```
Alternatively, if you have access to the config files, set SBIT.loadAtStartup
to 0 bool in Config/BIT.cfg. This might already be set for you in the Docker
image on MBARI DockerHub.

Verify that it is running the default `GoToSurface` app:
```
>show stack
2021-03-03T18:24:46.699Z,1614795886.699 [Default](IMPORTANT): Priority 0: Default:B.GoToSurface
```
An app is always being run.
If no missions are specified, then it is running the default.

### Control commands

Control commands can be issued to overwrite mission controls.
For example, the rudder can be held at a constant angle like so:
```
>maintain control horizontalcontrol.rudderangleaction -15 degree
```
This overwrites the controller and maintains the rudder at -15 degrees
(-0.261799 radians), which is the joint limit.

Unit conversions are automatically done in the MBARI code.
Alternatively, you can specify in radians.
```
>maintain control horizontalcontrol.rudderangleaction -0.2 radian
```

A thruster command can then be issued to move the vehicle in a circle:
```
>maintain control SpeedControl.propOmegaAction 300 rpm
```
Currently, this is the tested and preferred method of control.

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

### Troubleshoot

After issuing control commands, for example, rudder and thrust, if you then
notice that the vehicle gets some commands by itself, such as a non-zero
elevator angle, this is because a preloaded mission is being loaded, and you
need to wait to issue the control commands after it is done loading.
Make sure to use
```
quick on
```
to let the system finish loading, before issuing control commands.

### Run the circle mission

This has not been tested thoroughly.

Load the circle mission, which will perform two circles:
```
load Engineering/circle_test.xml
```

Set some parameters as desired:
```
set circle_test.Depth01 10 meter;set circle_test.Depth02 15 meter;set circle_test.RudderAngle01 15 degree;set circle_test.RudderAngle02 10 degree;set circle_test.WaitDuration 10 minute
run;quick off
```

You can check variables like depth:
```
report touch depth
quick on
```

To clear the report and go back to normal speed:
```
report clear
quick off
```

To stop the mission and terminate:
```
stop
quit
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
