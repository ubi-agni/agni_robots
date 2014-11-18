require "rttlib"
require "rttros"

tc=rtt.getTC()
d=tc:getPeer("Deployer")

d:import("rtt_rosnode")

-- Start of user code imports
d:import("lwr_fri")
d:import("oro_joint_state_publisher")
d:import("rtt_control_msgs")

d:import("SKukaComTest")
d:import("SLogSaver")
-- End of user code

ros=false
prio=0
schedpol=rtt.globals.ORO_SCHED_OTHER

local opttab=utils.proc_args(arg)
local cp=rtt.Variable("ConnPolicy")

function conn2ros(depl, port, topic)
   depl:stream(port,rtt.provides("ros"):topic(topic))
end


d:loadComponent("LWRDiag", "FRIDiagnostics")
d:setActivity("LWRDiag", 0.01, 2, rtt.globals.ORO_SCHED_RT)
LWRDiag = d:getPeer("LWRDiag")
LWRDiag:configure()

d:loadComponent("FRI", "FRIComponent")
--d:setActivity("FRI", 0.001, 80, rtt.globals.ORO_SCHED_RT)
d:setActivity("FRI", 0, 80, rtt.globals.ORO_SCHED_RT)
FRI = d:getPeer("FRI")

FRI:getProperty("fri_port"):set(49938) -- for real
--FRI:getProperty("fri_port"):set(49940)
FRI:configure()



d:loadComponent("JointMotionGen", "SKukaComTest")
--d:setActivity("JointMotionGen", 0.001, 70, rtt.globals.ORO_SCHED_RT)
d:setActivity("JointMotionGen", 0, 70, rtt.globals.ORO_SCHED_RT)
JointMotionGen = d:getPeer("JointMotionGen")
JointMotionGen:getProperty("DOF"):set(7)
JointMotionGen:configure()

d:loadComponent("LogSaver", "SLogSaver")
--d:setActivity("LogSaver", 0.001, 3, rtt.globals.ORO_SCHED_RT)
d:setActivity("LogSaver", 0, 20, rtt.globals.ORO_SCHED_RT)
LogSaver = d:getPeer("LogSaver")
LogSaver:configure()


d:loadComponent("JntPub", "JointStatePublisher")
d:setActivity("JntPub", 0.01, 10, rtt.globals.ORO_SCHED_RT)
JntPub = d:getPeer("JntPub")
sched_order=JntPub:getProperty("joint_names")
sched_order:get():resize(7)
for i=0,6,1 do 
sched_order[i]="right_arm_"..i.."_joint"
end 
JntPub:configure()


d:connect("FRI.RobotState", "LWRDiag.RobotState", rtt.Variable("ConnPolicy"))
d:connect("FRI.FRIState", "LWRDiag.FRIState", rtt.Variable("ConnPolicy"))
d:connect("FRI.JointPosition", "JntPub.msrJntPos", rtt.Variable("ConnPolicy"))

d:connect("FRI.RobotState", "JointMotionGen.RobotState", rtt.Variable("ConnPolicy"))
d:connect("FRI.FRIState", "JointMotionGen.FRIState", rtt.Variable("ConnPolicy"))
d:connect("FRI.JointPosition", "JointMotionGen.FRIRealJointPos", rtt.Variable("ConnPolicy"))

d:connect("JointMotionGen.DesiredJointPos", "FRI.JointPositionCommand", rtt.Variable("ConnPolicy"))

d:connect("JointMotionGen.Log", "LogSaver.Log", rtt.Variable("ConnPolicy"))
--d:connect("FRI.FRIState", "LogSaver.FRIState", rtt.Variable("ConnPolicy"))


-- ROS in out
d:stream("LWRDiag.Diagnostics",rtt.provides("ros"):topic("diagnostics"))

d:stream("JntPub.joints_state",rtt.provides("ros"):topic("joint_states"))
d:stream("FRI.KRL_CMD",rtt.provides("ros"):topic("lwr_arm_controller/fri_set_mode"))
d:stream("FRI.CartesianWrench",rtt.provides("ros"):topic("cartesian_wrench"))
--d:stream("JointMotionGen.Log",rtt.provides("ros"):topic("log"))

-- Start of user code usercode
FRI:start()
LWRDiag:start()
JntPub:start()
JointMotionGen:start()
LogSaver:start()

print("finished starting")
