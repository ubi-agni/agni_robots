require "rttlib"
require "rttros"

-- get parameter from arguments
local namespace=...

print ("starting "..namespace.." trajectory controller")

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


d:loadComponent(namespace.."JntTrajGen","InternalSpaceSplineTrajectoryGenerator")
d:setActivity(namespace.."JntTrajGen", 0.001, 5, rtt.globals.ORO_SCHED_RT)
JntTrajGen = d:getPeer(namespace.."JntTrajGen")
number_of_joints=JntTrajGen:getProperty("number_of_joints")
number_of_joints:set(7)
JntTrajGen:configure()

d:loadComponent(namespace.."JntTrajAction", "InternalSpaceSplineTrajectoryAction")
d:setActivity(namespace.."JntTrajAction", 0.1, 2, rtt.globals.ORO_SCHED_RT)
JntTrajAct = d:getPeer(namespace.."JntTrajAction")
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
JntTrajAct:provides("actionlib"):connect("/"..namespace.."/arm_trajectory_controller/follow_joint_trajectory")


JntTrajAct:configure()

d:addPeer(namespace.."JntTrajAction",namespace.."JntTrajGen")

d:connect(namespace.."JntTrajAction.trajectoryPtr", namespace.."JntTrajGen.trajectoryPtr", rtt.Variable("ConnPolicy"))


-- ROS in out
ros=rtt.provides("ros")
d:stream(namespace.."JntTrajAction.command",ros:topic("/"..namespace.."/arm_trajectory_controller/command"))
d:stream(namespace.."JntTrajAction.state",ros:topic("/"..namespace.."/arm_trajectory_controller/state"))

JntTrajAct:start()

print("finished starting trajectory controller")
