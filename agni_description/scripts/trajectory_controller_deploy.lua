require "rttlib"
require "rttros"

tc=rtt.getTC()
tcName=tc:getName()
print (tcName)
if tcName=="lua" then
		d=tc:getPeer("Deployer")
elseif tcName=="Deployer" then
		d=tc
end

-- ROS integration
d:import("rtt_actionlib")
d:import("rtt_actionlib_msgs")
d:import("rtt_rosnode")
d:import("rtt_roscomm")
d:import("rtt_std_msgs")
d:import("rtt_sensor_msgs")
d:import("rtt_diagnostic_msgs")
d:import("rtt_control_msgs")
d:import("rtt_trajectory_msgs")

--d:import("rtt_dot_service")
--d:loadService("Deployer","dot")

-- Start of user code imports
d:import("internal_space_spline_trajectory_generator")
d:import("internal_space_spline_trajectory_action")


d:loadComponent("JntTrajGen","InternalSpaceSplineTrajectoryGenerator")
d:setActivity("JntTrajGen", 0.001, 5, rtt.globals.ORO_SCHED_RT)
JntTrajGen = d:getPeer("JntTrajGen")
number_of_joints=JntTrajGen:getProperty("number_of_joints")
number_of_joints:set(7)
JntTrajGen:configure()


d:loadComponent("JntTrajAction", "InternalSpaceSplineTrajectoryAction")
d:setActivity("JntTrajAction", 0.1, 2, rtt.globals.ORO_SCHED_RT)
JntTrajAct = d:getPeer("JntTrajAction")
nbr_of_joints=JntTrajAct:getProperty("number_of_joints")
nbr_of_joints:set(7)
joint_names=JntTrajAct:getProperty("joint_names")
lower_limits=JntTrajAct:getProperty("lower_limits")
upper_limits=JntTrajAct:getProperty("upper_limits")
joint_names:get():resize(7)
lower_limits:get():resize(7)
upper_limits:get():resize(7)

lowlims = {-2.96 , -2.09, -2.96, -2.09, -2.96, -2.09, -2.96}
uplims = {2.96 , 2.09, 2.96, 2.09, 2.96, 2.09, 2.96}

namespace="ra"
for i=0,6,1 do 
  joint_names[i]=namespace.."_arm_"..i.."_joint"
  lower_limits[i] = lowlims[i+1]
  upper_limits[i] = uplims[i+1]
end 

-- TODO move that into a yaml and load it via the launch files that also spawns the lua
--JntTrajAct.loadService("rosparam")
--JntTrajAct.rosparam.getAll()
--JntTrajAct.rosparam.getParam("~/JntTrajAction/upper_limits", "upper_limits")
--JntTrajAct.rosparam.getParam("~/JntTrajAction/lower_limits", "lower_limits")
-- yaml file
--JntTrajGen:
--  number_of_joints: 7
--JntTrajAction:
--  joint_names: [joint1, joint2, joint3, joint4, joint5, joint6, joint7]
--  lower_limits: [-2.96 , -2.09, -2.96, -2.09, -2.96, -2.09, -2.96]
--  upper_limits: [2.96 , 2.09, 2.96, 2.09, 2.96, 2.09, 2.96]

JntTrajAct:loadService("actionlib")
JntTrajAct:provides("actionlib"):connect("/ra_arm_trajectory_action")      

JntTrajAct:configure()

d:addPeer("JntTrajAction","JntTrajGen")

d:connect("JntTrajAction.trajectoryPtr", "JntTrajGen.trajectoryPtr", rtt.Variable("ConnPolicy"))

JntTrajAct:start()

print("finished starting trajectory controller")
