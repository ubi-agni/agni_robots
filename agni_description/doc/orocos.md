# Orocos Deployment

## Overview

This package contains lua scripts to deploy orocos components to control the kuka robots, among others.

By default, the following scripts start a RTT environment with a deployer and a ROS service to run new lua scripts

* `arm.launch`
* `arm\_and\_hand.launch`
* `famula.launch`

Use `RTT:=0` if you want no rtt environment to be started.

Note, that `RTT:=0` does not stop the `kuka\_bringup.launch` to try load the lua file instantiating kuka controller components.
If you start your own rttlua environment, this launch file will still be waiting for the rosservice to appear.

## Test setup

A test setup, deploying a MotionManager, Loggers and connecting to kuka controllers is provided
in `launch/rtt\_deployement\_test.launch`
`>> roslaunch agni\_description rtt\_deployment\_test.launch`

## Usage

In order to run your own scripts you can deploy them using the rosservice or a launch file using the rosservice

* command line:
  `rosservice call /rtt\_environment/Deployer/run\_lua\_script "PATHTO\_agni\_description/scripts/cleanup\_component.lua"`
* launch file: 
  look at the example in `launch/rtt\_deployement\_test.launch`

If you are interested in deploying the kuka controller set only, without connecting to anything,
use `kuka/kuka\_deploy.launch` namespace:=ra|la

In case you crash or want to restart the rttlua environment, use
`roslaunch agni\_description rtt\_bringup.launch`
