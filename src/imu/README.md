IMU orientation test
====================

## Prerequisites

If you want to test multiple IMUs at once, the robot must expose in its `yarprobotinterface` configuration file a `multipleanalogsensorsserver` that publishes the orientation measurements for all of available sensors, with a `prefix` that matches exactly the `port` parameter of the test (the default one is `${portprefix}/alljoints/inertials`). 

An example of `alljoints-inertials_wrapper.xml` for iCubV2_* models:

```sh
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE devices PUBLIC "-//YARP//DTD yarprobotinterface 3.0//EN" "http://www.yarp.it/DTD/yarprobotinterfaceV3.0.dtd">

<device xmlns:xi="http://www.w3.org/2001/XInclude" name="alljoints-inertials_wrapper" type="multipleanalogsensorsserver">
    <param name="period">      10                           </param>
    <param name="name"> ${portprefix}/alljoints/inertials   </param>

    <action phase="startup" level="10" type="attach">
        <paramlist name="networks">
            <elem name="FirstStrain"> alljoints-inertials_remapper </elem>
        </paramlist>
       </action>

    <action phase="shutdown" level="15" type="detach" />
</device>
```

and `alljoints-inertials_remapper.xml`:

```sh
<?xml version="1.0" encoding="UTF-8" ?>
<!DOCTYPE devices PUBLIC "-//YARP//DTD yarprobotinterface 3.0//EN" "http://www.yarp.it/DTD/yarprobotinterfaceV3.0.dtd">


    <device xmlns:xi="http://www.w3.org/2001/XInclude" name="alljoints-inertials_remapper" type="multipleanalogsensorsremapper">
        <param name="OrientationSensorsNames">
            (l_arm_ft r_arm_ft head_imu_0)
        </param>
        <action phase="startup" level="5" type="attach">
            <paramlist name="networks">
                <elem name="head_imu">  head_inertial_hardware_device </elem>
                <elem name="left_arm_imu">  left_arm_inertial_hardware_device </elem>
                <elem name="right_arm_imu">  right_arm_inertial_hardware_device </elem>
            </paramlist>
        </action>

        <action phase="shutdown" level="20" type="detach" />
    </device>

```

## Usage

There are essentially two methods to install and run the IMU orientation test, which are going to be described in the following sections. In both cases, after launching the test, a .mat file containing the relevant measurements involved in the test will be generated.

### robotology-superbuild

If you have installed `icub-tests` as a part of the `robotology superbuild` framework with the [Robot Testing profile](https://github.com/robotology/robotology-superbuild/blob/master/doc/cmake-options.md#robot-testing) activation, i.e. by enabling the `ROBOTOLOGY_ENABLE_ROBOT_TESTING` CMake option of the superbuild, this test, like the others, is already available.

In this case, after compiling and being sure that `yarpserver` is up, you can run:

```sh
cd robotology-superbuild/src/icub-tests
robottestingframework-testrunner --suite suites/imu.xml
```

Otherwise, if you want to launch the test in a simulation environment, in a shell open a `gazebo` environment and import, for example, your iCub model. After that, open a separated shell, and:

```sh
#set YARP_ROBOT_NAME env variable with the name of the model you imported in gazebo
#example:
export YARP_ROBOT_NAME=iCubGazeboV2_7
cd robotology-superbuild/src/icub-tests
robottestingframework-testrunner --suite suites/imu-icubGazeboSim.xml
```

### pixi

If you want to run the test without depending on the whole robotology-superbuild, you can use `pixi`. First of all, be sure to have [`pixi`](https://pixi.sh/#installation) installed. Then, clone `icub-tests` and run the test as a pixi task:

```sh
git clone https://github.com/robotology/icub-tests
cd icub-tests
pixi run imu_test
```

In simulation, instead, import your model in `gazebo` and then run:

```sh
#set YARP_ROBOT_NAME env variable with the name of the model you imported in gazebo
#example:
export YARP_ROBOT_NAME=iCubGazeboV2_7
cd icub-tests
pixi run imu_sim_test
```

## Generate report

The IMU test is based on [`robometry`](https://github.com/robotology/robometry) that allows logging data from the robot sensors and saving them into a .mat file that will be generated at the end of the test execution. 

To generate the report containing the plots with the acquired data and the results of the test, you can launch the `generate_report.mlx` MATLAB Live Script under `icub-tests/src/imu/report/` folder. After opening this file with MATLAB, there is a `File Browser control` with a `File` button, from which you can navigate your folders and select the .mat file.

Then, `run` the script and wait until it's done, it may take some time to finish. Finally, you can manually export the report in HTML format or, in the Command Window, run:

```matlab
    export('report.mlx', format='html', HideCode=true);
```