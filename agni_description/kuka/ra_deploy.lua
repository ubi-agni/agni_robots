require "rttlib"
require "rttros"

tc=rtt.getTC()
d=tc:getPeer("Deployer")

-- ROS integration
d:import("rtt_rosnode")
d:import("rtt_roscomm")
d:import("rtt_std_msgs")
d:import("rtt_sensor_msgs")
d:import("rtt_diagnostic_msgs")

-- Start of user code imports
d:import("lwr_fri")
d:import("oro_joint_state_publisher")
d:import("rtt_control_msgs")

d:import("flwr_filter")
d:import("s_motion_manager")
d:import("s_log_saver")

d:import("gazebo_attach_controller")

-- End of user code

ros=false
prio=0
schedpol=rtt.globals.ORO_SCHED_OTHER

local opttab=utils.proc_args(arg)
local cp=rtt.Variable("ConnPolicy")

function conn2ros(depl, port, topic)
   depl:stream(port,rtt.provides("ros"):topic(topic))
end


d:loadComponent("Grasp", "GazeboAttachController")
d:setActivity("Grasp", 0, 20, rtt.globals.ORO_SCHED_RT)
Grasp = d:getPeer("Grasp")

-- FIXME: retrieving the arm name is a problem,
-- as this deployer is used for arm and arm+hand with different model names
Grasp:getProperty("ref_model_name"):set("right_kuka_shadow")
Grasp:getProperty("ref_link_name"):set("ra_arm_7_link")
-- TODO: the target model should be included in the attach call
Grasp:getProperty("tgt_model_name"):set("coke_can")
Grasp:getProperty("tgt_link_name"):set("link")
Grasp:configure()


d:loadComponent("LWRDiag", "FRIDiagnostics")
d:setActivity("LWRDiag", 0.01, 2, rtt.globals.ORO_SCHED_RT)
LWRDiag = d:getPeer("LWRDiag")
LWRDiag:configure()

d:loadComponent("FRIRA", "FRIComponent")
--d:setActivity("FRI", 0.001, 80, rtt.globals.ORO_SCHED_RT)
d:setActivity("FRIRA", 0, 80, rtt.globals.ORO_SCHED_RT)
FRIRA = d:getPeer("FRIRA")
FRIRA:getProperty("fri_port"):set(49938)
FRIRA:configure()


d:loadComponent("FilterRA", "FLWRFilter")
--d:setActivity("JointMotionGen", 0.001, 70, rtt.globals.ORO_SCHED_RT)
d:setActivity("FilterRA", 0, 70, rtt.globals.ORO_SCHED_RT)
FilterRA = d:getPeer("FilterRA")
FilterRA:getProperty("FREQUENCY"):set(1.0)
FilterRA:configure()


d:loadComponent("MotionManager", "SMotionManager")
d:setActivity("MotionManager", 0.001, 60, rtt.globals.ORO_SCHED_RT)
MotionManager = d:getPeer("MotionManager")
MotionManager:configure()


d:loadComponent("LogRA", "SLogSaver")
--d:setActivity("LogRA", 0.001, 3, rtt.globals.ORO_SCHED_RT)
d:setActivity("LogRA", 0, 20, rtt.globals.ORO_SCHED_RT)
LogRA = d:getPeer("LogRA")
LogRA:configure()


d:loadComponent("JntPub", "JointStatePublisher")
d:setActivity("JntPub", 0.01, 10, rtt.globals.ORO_SCHED_RT)
JntPub = d:getPeer("JntPub")
sched_order=JntPub:getProperty("joint_names")
sched_order:get():resize(7)
for i=0,6,1 do 
sched_order[i]="ra_arm_"..i.."_joint"
end 
JntPub:configure()


d:connect("FRIRA.RobotState", "LWRDiag.RobotState", rtt.Variable("ConnPolicy"))
d:connect("FRIRA.FRIState", "LWRDiag.FRIState", rtt.Variable("ConnPolicy"))
d:connect("FRIRA.JointPosition", "JntPub.JointPosition", rtt.Variable("ConnPolicy"))
d:connect("FRIRA.JointVelocity", "JntPub.JointVelocity", rtt.Variable("ConnPolicy"))
d:connect("FRIRA.JointTorque", "JntPub.JointEffort", rtt.Variable("ConnPolicy"))


d:connect("FRIRA.RobotState", "FilterRA.RobotState", rtt.Variable("ConnPolicy"))
d:connect("FRIRA.FRIState", "FilterRA.FRIState", rtt.Variable("ConnPolicy"))
d:connect("FRIRA.JointPosition", "FilterRA.FRIJointPos", rtt.Variable("ConnPolicy"))

d:connect("FRIRA.JointPosition", "MotionManager.FRIRealJointPosRA", rtt.Variable("ConnPolicy"))

d:connect("MotionManager.DesiredJointPosRA", "FilterRA.DesiredJointPos", rtt.Variable("ConnPolicy"))
d:connect("FilterRA.FilteredJointPos", "FRIRA.JointPositionCommand", rtt.Variable("ConnPolicy"))


d:connect("FilterRA.Log", "LogRA.Log", rtt.Variable("ConnPolicy"))
--d:connect("FRI.FRIState", "LogRA.FRIState", rtt.Variable("ConnPolicy"))


-- ROS in out
ros=rtt.provides("ros")
d:stream("LWRDiag.Diagnostics",ros:topic("diagnostics"))

d:stream("JntPub.joint_state",ros:topic("joint_states"))


d:stream("Grasp.Attach",ros:topic("/gazebo_attach"))
d:stream("Grasp.Attached",ros:topic("/gazebo_attached"))

--d:stream("FRIRA.KRL_CMD",rtt.provides("ros"):topic("lwr_arm_controller/fri_set_mode"))
--d:stream("FRIRA.CartesianWrench",rtt.provides("ros"):topic("cartesian_wrench"))
--d:stream("FilterRA.Log",rtt.provides("ros"):topic("log"))

-- Start of user code usercode
FRIRA:start()
LWRDiag:start()
JntPub:start()
FilterRA:start()
LogRA:start()
MotionManager:start()
Grasp:start()

print("finished starting")
