# Launch files

## structural design choices

We follow the standard naming convention for launch files:

* robot\_upload.launch:      upload robot\_description to the parameter server
* robot\_bringup.launch:     start simulated (load/start controllers) or real robot (start drivers)

We have generic and specific platform launch files:

* generic :
  * arm\_and\_hand.launch:    to start any arm + shadowhand, left/right/both, real/sim
  * arm.launch:    to start any arm, left/right/both, real/sim
  * hand.launch:    to start shadowhand, left/right/both, real/sim
  * platforms\_setup.launch:  to upload a full description on any frame + any arm + any tool

* specific :
  * sfb.launch / famula.launch : to start the sfb or famula platform
	No parameters needed. These use pre-generated URDFs to speed up the launch process.
      
## Notes on robot\_description, joint\_states, robot\_state_publishers

There are good reasons why there are multiple robot\_description (also see bimanual\_description.txt),
joint\_states and robot\_state\_publishers, on several namespaces rh/ra/lh/la/left/right and root called /

They are summarized in the following table. Abbreviations used:

* robot\_description : rd
* robot\_state\_publisher : rsp
* namespace : ns


|launch file         | /rd            | rh/rd and lh/rd       |  ra/rd and la/rd   | right/left     |
|---------------------------------------------------------------------------------------------------|
|arm.launch     real | \  display     |                       | \  for spawn       |                |
|                    |  > + main rsp  |                       |  > + plugin        |                |
|               sim  | /              |                       | /  no rsp here     |                |
|---------------------------------------------------------------------------------------------------|
|hand.launch    real |\ no rd, no rsp | driver +rsp in driver |                    |                |
|                    | > rsp would be |                       |                    |                |
|               sim  |/  duplicate    | spawn/plugin + ns/rsp |                    |                |
|---------------------------------------------------------------------------------------------------|
|arm\_and\_hand real | \  display     | driver +rsp in driver |                    |                |
|                    |  > + main rsp  |                       |                    |                |
|               sim  | /              | plugin only           | plugin only        | spawn, no rsp  |
|---------------------------------------------------------------------------------------------------|

The number of robot\_state\_publishers was reduced to a minimum. However, some constraints in the shadow hand driver require to have some other ones. They then have tf-prefix = namespace to avoid conflicts (arm+hand) in hand alone, for consistency reason there is no main rsp, only namespaced ones.

robot\_descriptions in namespace are often for plugins that do not support multiple trees (2 hands)
or for simplicity of urdf generation (in current nested structure, it is hard to generate 1 arm or 2
arms but only one single controllers plugin, with correct prefix/namespace for its joint state to be
on /arm or /hand for instance)
