# Launch files

## constraints
* To allow merging of joint states, e.g. from left and right hands, 
  we need to have name prefixes 'lh_' and 'rh_' for joints and links in the robot_description.

## structural design choices

We follow the standard naming convention for launch files:

* robot_upload.launch:      upload `robot_description` to the parameter server
* robot_bringup.launch:     start simulated (load/start controllers) or real robot (start drivers)


We have generic and specific platform launch files:

* generic :
  * `arm_and_hand.launch`:    to start any arm + shadowhand, left/right/both, real/sim
  * `arm.launch`:    to start any arm, left/right/both, real/sim
  * `hand.launch`:    to start shadowhand, left/right/both, real/sim
  * `platforms_setup.launch`:  to upload a full description on any frame + any arm + any tool

* specific :
  * sfb.launch / famula.launch : to start the sfb or famula platform
	No parameters needed. These use pre-generated URDFs to speed up the launch process.
      
## Notes on robot_description, joint_states, robot_state_publishers

There are good reasons why there are multiple `robot_description` 
`joint_states` and robot_state_publishers, on several namespaces rh/ra/lh/la/left/right and root called /

* Linking several robots together from several files (using fixed joints in between) is not possible
  within URDF. One can only create a single URDF with all joint and links in a single <robot> tag.
* Gazebo plugins (e.g. sr_gazebo_plugin providing joint access in physics) require a separate `robot_description` for the joints they control
  (and only those). Hence, 4 `robot_descriptions` are needed, one for each single robot (2 for arms, 2 for hands) in respective ra/la/rh/lh namespaces.
* sr_gazebo_plugin cannot handle dual hands (new real hand driver can now, using serial id to distiguish hands). 
  For this reason, the hands cannot be spawned in the same robot.
* To ensure the hands are attached with a fixed link to the arm, an arm+hand `robot_description` is created
  and spawned into gazebo. Hence, we have 2 more `robot_description` for each side (in right/left namespaces).
* right/left are connected together in the tf tree by linking them to the world with a calibration of the base.
* There is a `/robot_description` containing the bimanual setup + frame (+ others) that would be used for planning and display of the full setup.

For instance (right): There are 3 namespaces: right/ra/rh.

* `/right/robot_description` : for planning and spawning (so that the hand and arm are physically linked together)
  with joint and link names prefixed with rh_ resp. ra_ for hand and arm
* `/ra/robot_description` : used for `sr_gazebo_plugin` (for pa10 controllers and lwrcontrollers plugin)
  with name prefix ra_ for links and joints
* `/rh/robot_description` : used for `sr_gazebo_plugin` / in real, loaded by default by the shadow driver 
  with name prefix rh_ for links and joints

Because names have the same prefixes in all these `robot_descriptions`, gazebo can associate them together.
Hence, controllers and the joint_states publisher exist in namespaces /ra and /la.

* `/right/joint_states` : merged version of the two below
* `/ra/joint_states` : in real world we have a separate js per robot, so create them in sim too
* `/rh/joint_states` : in real world we have a separate js per robot, so create them in sim too

Similar for left side.

The following table summarizes the use of the various `robot_description` in real and simulated environments.
Abbreviations used:

* robot_description : rd
* robot_state_publisher : rsp
* namespace : ns


|launch file         | /rd            | rh/rd and lh/rd       |  ra/rd and la/rd   | right/rd & left/rd |
|-------------------------------------------------------------------------------------------------------|
|arm.launch     real | \    display   |                       |                    |                    |
|                    |  } + main rsp  |                       |                    |                    |
|               sim  | /              |                       | gazebo spawn+plugin|                    |
|-------------------------------------------------------------------------------------------------------|
|hand.launch    real |\ no rd, no rsp | driver +rsp in driver |                    |                    |
|                    | } rsp would be |                       |                    |                    |
|               sim  |/  duplicate    | spawn/plugin + ns/rsp |                    |                    |
|-------------------------------------------------------------------------------------------------------|
|arm_and_hand   real | \  display     | driver +rsp in driver |                    |                    |
|                    |  } + main rsp  |                       |                    |                    |
|               sim  | /              | plugin only           | plugin only        |  gazebo spawn      |
|-------------------------------------------------------------------------------------------------------|

The number of robot_state_publishers was reduced to a minimum. However, some constraints in the shadow hand driver creates some other ones. 
They need to have tf-prefix = namespace to avoid conflicts with global rsp.
For consistency reason, in the standalone hands, there is no main rsp, only namespaced ones.

`robot_description` in namespace is often for plugins that do not support multiple trees (2 hands)
or for simplicity of urdf generation (in current nested structure, it is hard to generate 1 arm or 2
arms but only one single controllers plugin, with correct prefix/namespace for its joint state to be
on /arm or /hand for instance)
