Describe  2 arms + 2 hands + 1 frame in REAL world and in SIM
-------------------------------------------------------------

- Linking several robots together from several files (using fixed joints in between) is not possible
  within URDF. One can only create a single URDF with all joint and links in a single <robot> tag
- Gazebo plugins (e.g. sr\_gazebo\_plugin) require the robot\_description for the joints they control
  (and only those). One needs to create 4 robot\_descriptions for each single robot (2 for arms,
  2 for hands) in respective ra/la/rh/lh namespaces.
- sr\_gazebo\_plugin cannot handle dual hands (new real hand driver can now, using serial id to distiguish hands). For this reason, the hands cannot be spawned in the same robot.
- To ensure the hands are attached with a fixed link to the arm, an arm+hand robot\_description is created
  and spawned into gazebo. Hence, we have 2 more robot\_description for each side (in right/left namespaces).
  
- right/left are connected together in the tf tree by linking them to the world with a calibration of the base.

As a summary (right): There are 3 namespaces: right/ra/rh.

* `/right/robot\_description` : for planning and spawning (so that the hand and arm are physically linked together)
* `/ra/robot\_description` : used for `sr\_gazebo\_plugin` (for pa10 controllers) and lwrcontrollers plugin
* `/rh/robot\_description` : used for `sr\_gazebo\_plugin` / in real, loaded by default by the shadow driver with name rh/sh\_description

* `/right/joint\_states` : merged version of the two below
* `/ra/joint\_states` : in real world we have a separate js per robot, so create them in sim too
* `/rh/joint\_states` : in real world we have a separate js per robot, so create them in sim too

Similar for left side.

NOTE: when spawning the left/right side, the groupNS is left/right, but the namespace for hand and arm controllers should be ra/la and rh/lf

hand controllers need ns:rh 
kuka gazebo bringup needs ns:ra

There is a `/robot\_description` containing the bimanual setup + frame (+ others) that would be used for planning etc.





