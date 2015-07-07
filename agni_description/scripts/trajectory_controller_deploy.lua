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
d:import("rtt_rosnode")
d:import("rtt_roscomm")
d:import("rtt_std_msgs")
d:import("rtt_sensor_msgs")
d:import("rtt_diagnostic_msgs")
d:import("rtt_control_msgs")
--d:import("rtt_dot_service")
--d:loadService("Deployer","dot")

-- Start of user code imports
d:import("s_motion_manager")
d:import("joint_spline_trajectory_generator")
d:import("oro_joint_trajectory_action")

d:loadComponent("MotionManager", "SMotionManager")
d:setActivity("MotionManager", 0.001, 60, rtt.globals.ORO_SCHED_RT)
MotionManager = d:getPeer("MotionManager")
MotionManager:configure()

d:loadComponent("JntTrajGen","JointSplineTrajectoryGenerator")
d:setActivity("JntTrajGen", 0.001, 5, rtt.globals.ORO_SCHED_RT)
JntTrajGen = d:getPeer("JntTrajGen")
number_of_joints=JntTrajGen:getProperty("number_of_joints")
number_of_joints:set(7)
JntTrajGen:configure()

d:loadComponent("JntTrajGen","JointSplineTrajectoryGenerator")
d:setActivity("JntTrajGen", 0.001, 5, rtt.globals.ORO_SCHED_RT)
JntTrajGen = d:getPeer("JntTrajGen")
number_of_joints=JntTrajGen:getProperty("number_of_joints")
number_of_joints:set(7)
JntTrajGen:configure()

d:loadComponent("JntTrajAction", "JointTrajectoryAction")
d:setActivity("JntTrajAction", 0.1, 2, rtt.globals.ORO_SCHED_RT)
JntTrajAct = d:getPeer("JntTrajAction")
nbr_of_joints=JntTrajAct:getProperty("number_of_joints")
nbr_of_joints:set(7)
for i=0,6,1 do 
		jntnameprop=rtt.Property("string", "joint"..i.."_name", "")
		jntnameprop:set("right_arm_"..i.."_joint")
		JntTrajAct:addProperty(jntnameprop)
end
JntTrajAct:configure()


d:addPeer("JntTrajAction","JntTrajGen")

d:connect("JntTrajGen.JointPositionCommand", "FRI.JointPositionCommand", rtt.Variable("ConnPolicy"))
d:connect("FRI.JointPosition","JntTrajGen.DesiredJointPosition", rtt.Variable("ConnPolicy"))
d:connect("JntTrajAction.trajectory_point", "JntTrajGen.trajectory_point", rtt.Variable("ConnPolicy"))
d:connect("JntTrajGen.buffer_ready", "JntTrajAction.buffer_ready", rtt.Variable("ConnPolicy"))
d:connect("JntTrajGen.trajectory_compleat", "JntTrajAction.trajectory_compleat", rtt.Variable("ConnPolicy"))





-- ROS in out
ros=rtt.provides("ros")
d:stream("JntTrajAction.command",rtt.provides("ros"):topic("joint_trajectory_action/command"))
d:stream("JntTrajAction.goal",rtt.provides("ros"):topic("joint_trajectory_action/goal"))
d:stream("JntTrajAction.cancel",rtt.provides("ros"):topic("joint_trajectory_action/cancel"))
d:stream("JntTrajAction.feedback",rtt.provides("ros"):topic("joint_trajectory_action/feedback"))
d:stream("JntTrajAction.result",rtt.provides("ros"):topic("joint_trajectory_action/result"))
d:stream("JntTrajAction.status",rtt.provides("ros"):topic("joint_trajectory_action/status"))



JntTrajAct:start()

LogLA:start()
LogRA:start()
MotionManager:start()
Grasp:start()
print("finished starting manager")
