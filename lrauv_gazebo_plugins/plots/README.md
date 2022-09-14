Plot variables from missions for validation

### Dependencies

```
$ pip3 install matplotlib numpy
```

### Pre-requisites

Compile the `unserialized` target if you haven't done so:
```
cd ~/lrauv_ws/src/lrauv-application
make unserialize
```

### Usage

1. Run a mission

    Every time you run a mission in the LRAUV application, it generates
    some log files in `lrauv-application/Logs/`. For example, run a mission:

    ```
    $ bin/LRAUV
    > run RegressionTests/IgnitionTests/testYoYoCircle.xml
    ```

1. Generate CSV


    Use a script to unserialize the relevant variables and output a csv file:

    ```
    ./unserialize_for_plotting.sh
    ```

    By default, it unserializes `lrauv-application/Logs/latest` directory, which
    is a symbolic link to the log directory from the last run. The output goes
    to `./missions/tmp/tmp.csv`.

    To specify a different input log directory and a different output directory,
    you can pass in the specific log directory under `lrauv-application/Logs/`
    and the mission name, like so:

    ```
    ./unserialize_for_plotting.sh 20210811T002224 testYoYoCircle
    ```

    The unserialized output will go to `./missions/<missionName>`.

    See more information on the top of `unserialize_for_plotting.sh`.

1. Generate plots

    Run a script to plot the relevant variables in appropriate subplots:

    ```
    python3 plot_missions.py <mission_name> <tmp>
    ```

    Where `<mission_name>` is the name of the mission, and `<tmp>` is the `tmp`
    string, which if present, will plot values from `missions/tmp/tmp.csv`.

    See more information on the top of `unserialize_for_plotting.sh`.
