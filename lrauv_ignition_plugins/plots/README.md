Plot variables from missions for validation

### Dependencies

```
$ pip3 install matplotlib numpy
```

### Batch unserialize variables to inspect

This assumes you have run a mission in the LRAUV application, which would
generate some log files in `lrauv-application/Logs/`:
```
$ bin/LRAUV
> run RegressionTests/IgnitionTests/testYoYoCircle.xml
```

Compile the unserialized target if you haven't done so:
```
cd ~/lrauv_ws/src/lrauv-application
make unserialize
```

This unserializes the relevant variables, and outputs the csv file to the
output directory specified in the script:
```
./unserialize_for_plotting.sh
```

By default, it unserializes `latest` directory, which is a symbolic link to the
log directory from the last run. The output goes to `./missions/tmp` directory.

To specify a different input log directory and a different output directory,
you can pass in the specific log directory under `Logs/` and the mission name,
like so:
```
./unserialize_for_plotting.sh 20210811T002224 testYoYoCircle
```
The unserialized output will go to `./missions/<missionName>`.

### Batch plot

In `plot_missions.py`, uncomment the block for the mission you want to plot.
Each mission has different plot configurations.

Then, run the script to plot the relevant variables in appropriate subplots:
```
python3 plot_missions.py
```
