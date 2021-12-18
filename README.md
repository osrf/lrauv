# LRAUV Simulation

This repository contains packages for simulating the Tethys-class Long-Range
AUV (LRAUV) from the Monterey Bay Aquarium Research Institute (MBARI).

Disclaimer: This repository is in active development.
Stability is not guaranteed.

Source files, models, and plugins relevant to a general audience are upstreamed
on an irregular basis to Ignition libraries, the top-level library being
[ign-gazebo](https://github.com/ignitionrobotics/ign-gazebo/).
Upstreamed files may eventually be removed from this repository.

Standalone, this repository contains the environment and plugins necessary to
simulate an underwater vehicle in Ignition Gazebo.
Integrated with the real-world LRAUV controller code, the binaries of which are
provided to the public on MBARI's DockerHub (see below), the simulated robot
can be controlled using the same code executed on the real robot.
This enables the validation of scientific missions for oceanography research.

## Using Docker for this repository (optional)

Optionally, you may choose to build this repository using Docker, for
convenience.
Make sure you have a recent version of [Docker](https://docs.docker.com/) and
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

## To build

To run the code in this repository natively without Docker, make sure you have
[Ignition Garden](https://ignitionrobotics.org/docs/garden) and
[colcon](https://colcon.readthedocs.io/en/released/), on Ubuntu Focal or higher.

Install dependencies
```
sudo apt-get install libpcl-dev
```

Clone this repository then run
```
colcon build
```

Developers may want to build tests. Note that this would take longer:
```
colcon build --cmake-args "-DBUILD_TESTING=ON"
```

> You can pass `--cmake-args ' -DENABLE_PROFILER=1'` to use the profiler.
> See more on [this tutorial](https://ignitionrobotics.org/api/common/4.4/profiler.html)

## To test simulation in Ignition standalone (without MBARI integration)

This package comes with an empty example world. To run this example world simply
source the colcon workspace and run:
```
. install/setup.bash
ign gazebo buoyant_tethys.sdf
```

Send example commands to move some joints:
```
LRAUV_example_controller
```

Keyboard teleop:
```
LRAUV_keyboard_teleop
```

> Tip: Type `LRAUV_` and press tab for autocomplete to show more example examples.

## To test integration with MBARI LRAUV code base

MBARI's code base lives in a separate, private repository.
For people with access,
[here](https://bitbucket.org/mbari/lrauv-application/src/7b3b5fce1b0ad1af1734952adaf94f2a69193aec/docker_ignition/?at=feature%2F2021-02-12-ignition-sim)
are instructions on setting it up from source and compiling.

Alternatively, you can use the public Docker image (see below).

The integration assumes that this repository is cloned as a sibling of
the `lrauv-application` repository, i.e.:
```
<workspace>
|-- lrauv
└-- lrauv-application
```

For quick reference, compilation boils down to running this on the right branch:
```
make ignition
```

### MBARI public Docker image

A public Docker image is available for people without access to the MBARI
codebase.
[MBARI's image on DockerHub](https://hub.docker.com/r/mbari/lrauv-ignition-sim)
contains Ignition, MBARI's LRAUV code base, and this repository.
All the code has been compiled.
```
docker pull mbari/lrauv-ignition-sim
```
Note: To update that image, see
[instructions](https://bitbucket.org/mbari/lrauv-application/src/f89b2bf16e3e5e72cafe6b21252a1a6b3314fbaa/docker_ignition/README.md?at=feature%2F2021-02-12-ignition-sim)
in MBARI's private `lrauv-application` repository.

Once inside a container, source the colcon workspaces:
```
. ~/ign_ws/install/setup.bash
. ~/lrauv_ws/install/setup.bash
```
This needs to be done for each terminal.

### Run the Ignition simulation

For ease of development, the following world is set up to run at a real time
factor (RTF) of 0 (as fast as possible) and a step size of 0.02 seconds.
That is significantly faster than the default Ignition setting of RTF 1 and
step size 0.001 seconds, which will give real time performance and roughly the
nominal vehicle speed of 1 m/s.

The RTF and step size can be changed at run time via the GUI by going to the
Inspector panel and then Physics Group.
Alternatively, they can be changed prior to compilation in the world SDF under
`<physics><max_step_size>` and `<physics><real_time_factor>`.

Launch the Ignition simulation:
```
ign gazebo buoyant_tethys.sdf
```
For verbose debug output, add `--verbose 4`.

Unpause Ignition by clicking on the triangle play button in the lower-left
corner of the GUI, or by pressing the space bar.

### Run the MBARI LRAUV application

The MBARI LRAUV Main Vehicle Application (MVA) contains everything needed to
control and operate the vehicle in the real world and in simulation.

Time synchronization has been done between the MBARI application and the
Ignition simulation, such that in most cases, the controller and the simulation
should be running in synchronization.
There may be corner cases that still need to be resolved.

This section assumes you have either compiled the target from source or are
using the MBARI public Docker image, which has everything pre-compiled.
The paths assume you are using the MBARI public Docker image.

Run the LRAUV Main Vehicle Application:
```
cd ~/lrauv_ws/src/lrauv-application
bin/LRAUV
```
This will bring you to a command prompt.

At the LRAUV command prompt (you only need to do this once):
```
>configset micromodem.loadatstartup 0 bool persist
>restart app
```
This sets the micromodem to not load at startup.
`persist` means you only need to do this once.
It will pause for a bit, you might not be able to type right away.

On the vehicle, an app is always being run.
If no missions are specified, then it is running the default.
On the real vehicle, the default mission is `GoToSurface`.
(NOTE: we have removed the default app in the MBARI public image until
[this issue](https://github.com/osrf/lrauv/issues/38) is resolved.
Currently, nothing is being run by default. Skip this check.)
Verify that it is running the default `GoToSurface` app:
```
>show stack
2021-03-03T18:24:46.699Z,1614795886.699 [Default](IMPORTANT): Priority 0: Default:B.GoToSurface
```

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

### Run missions designed for Ignition integration tests

The following are "unit test" missions that test one or two actuators at a time.
Run one at a time, in separate runs of Ignition and the Main Vehicle Application
(`bin/LRAUV`):
```
run RegressionTests/IgnitionTests/testDepthVBS.xml
run RegressionTests/IgnitionTests/testPitchMass.xml
run RegressionTests/IgnitionTests/testPitchAndDepthMassVBS.xml
run RegressionTests/IgnitionTests/testYoYoCircle.xml
```
Some example behaviors are documented [here](https://github.com/osrf/lrauv/issues/21).

Some parameters can be adjusted - see the mission XML file.
For example, to change the commanded depth in the `testDepthVBS.xml` mission:
```
load RegressionTests/IgnitionTests/testDepthVBS.xml
set buoy_test_vbs.DepthCmd 20 meter
run
```

To stop a mission, run
```
stop
```

You can automate typing into the command prompt by issuing `-x`.
For example, this will run the yoyo mission and terminate after the mission ends:
```
bin/LRAUV -x "run RegressionTests/IgnitionTests/testYoYoCircle.xml quitAtEnd"
```

### To run the original MBARI LRAUV command-line simulation (optional)

The original simulation (`SimDaemon`) is the baseline comparison for the
Ignition simulation.

For developers, it helps to troubleshoot the Ignition simulation by comparing
its values to the original MBARI simulation, which is a pure command-line
interface.

Do not run both Ignition and SimDaemon at the same time.
Choose only one.

```
cd ~/lrauv_ws/src/lrauv-application
```

In the MBARI code base, open `Config/sim/Simulator.cfg`, change these lines to
look like this:
```
   ExternalSim.loadAtStartup = 1 bool;
   ExternalSimIgnition.loadAtStartup = 0 bool;
```
This enables the original ExternalSim and disables the interface with Ignition.

Run the original command-line simulation:
```
bin/SimDaemon
```
The SimDaemon runs in the background by default.

Then run the Main Vehicle Application as usual:
```
bin/LRAUV
```

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

### Using Debug Container to debug the simulator

A simple dockerfile and tmux config exists that makes launching and debugging the different components of the project a lot easier. To use it simply run

$ docker/debug_integration.sh

This will build a new container with the source code and launch a tmux session. The tmux session has 2 windows: 0:simulation and 1:logging. In the simulation window you will see the top pane runs the ignition simulation while the bottom pane runs the actual `bin/LRAUV` controller. The logging pane on the other hand will automatically convert the sim slate and write it to the results directory on your computer one layer above the directory to where you checked out.

![tmux_debug](https://user-images.githubusercontent.com/542272/137456870-a0eed740-7206-43c1-8ccf-215148ad4675.gif)

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

### Multiple vehicles

Each instance of `bin/LRAUV` is tied to a single vehicle. In order to work
with multiple vehicles, multiple instances of `bin/LRAUV` must be spun up.

The first vehicle spun up will be placed at the origin of the world, and
the latitude / longitude of the world's origin will be set to coincide with it.
Subsequent vehicles will be spawned at positions relative to the initial one,
according to their latitude / longitude.

Information about the vehicle is hardcoded on the `lrauv-application` code,
within the `Config` folder. Here's a recommended setup assuming that you have
`lrauv-application` cloned under `~/lrauv_ws/src`:

1. Copy the `lrauv-application` folder for each robot to be spawned:

    ```
    cp -r ~/lrauv_ws/src/lrauv-application ~/lrauv_ws/src/lrauv-application-2
    cp -r ~/lrauv_ws/src/lrauv-application ~/lrauv_ws/src/lrauv-application-3
    ...
    ```

1. Edit the vehicle name (in `Config/vehicle.cfg`) and initial location (in
   `Config/workSite.cfg`) for each instance. For example:

    ```diff
    --- lrauv-application/Config/vehicle.cfg        2021-09-27 16:17:09.816305451 -0700
    +++ lrauv-application-2/Config/vehicle.cfg      2021-09-29 14:53:57.480185748 -0700
    @@ -10,7 +10,7 @@
     ////////////////////////////////////////////////////////////////////

     //   Vehicle.name            = "Tethys";
    -   Vehicle.name            = "tethys";  // Use name to match Ignition default SDF
    +   Vehicle.name            = "daphne";  // Use name to match Ignition default SDF
        Vehicle.id              = 0 enum;
        Vehicle.kmlColor        = "ff0055ff"; // 4 hex bytes indicating alpha, blue, green, and red
                                            // In this case, orange.
    --- lrauv-application/Config/workSite.cfg       2021-09-27 14:16:43.622409403 -0700
    +++ lrauv-application-2/Config/workSite.cfg     2021-09-29 14:53:06.887476472 -0700
    @@ -14,8 +14,8 @@
     //  initLat        =   36.806966 arcdeg; // Initial latitude
     //  initLon        = -121.824326 arcdeg; // Initial longitude
     // initial position same as for regression tests
    -  initLat       =   36.8034 arcdeg; // Initial latitude
    -  initLon       = -121.8222 arcdeg; // Initial longitude
    +  initLat       =   36.8033 arcdeg; // Initial latitude
    +  initLon       = -121.8223 arcdeg; // Initial longitude

       startupScript = "Missions/Startup.xml";  // Mission to run on power-up
       defaultScript = "Missions/Default.xml";  // Mission to run when no other mission is running.
    ```

To run simulation, use the empty environment, and start vehicles in order,
for example:

1. `ign gazebo empty_environment.sdf -v 4`
1. `~/lrauv_ws/src/lrauv-application/bin/LRAUV`
1. `~/lrauv_ws/src/lrauv-application-2/bin/LRAUV`
1. Start more `bin/LRAUV` as needed

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

#### Unserializing and plotting values

On the MBARI Main Vehicle Application side, all values during the run are
stored to disk.
They can be retrieved after the run and plotted for debugging purposes.

See [`lrauv_ignition_plugins/plots/README.md`](https://github.com/osrf/lrauv/blob/main/lrauv_ignition_plugins/plots/README.md)
for instructions to unserialize and scripts for plotting.

## Science data

Science data can be read from a csv file with the following recognized field
names in the first line of the file:
```
elapsed_time_second
latitude_degree
longitude_degree
depth_meter
sea_water_temperature_degC
sea_water_salinity_psu
mass_concentration_of_chlorophyll_in_sea_water_ugram_per_liter
eastward_sea_water_velocity_meter_per_sec
northward_sea_water_velocity_meter_per_sec
```
