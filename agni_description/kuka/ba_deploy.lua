require "rttlib"
require "rttros"

tc=rtt.getTC()
d=tc:getPeer("Deployer")

d:import("rtt_rosnode")

-- Start of user code imports
d:import("lwr_fri")
d:import("oro_joint_state_publisher")
d:import("rtt_control_msgs")

d:import("FLWRFilter")
d:import("SMotionManager")
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


d:loadComponent("LWRDiagLA", "FRIDiagnostics")
d:setActivity("LWRDiagLA", 0.01, 2, rtt.globals.ORO_SCHED_RT)
LWRDiagLA = d:getPeer("LWRDiagLA")
LWRDiagLA:configure()

d:loadComponent("LWRDiagRA", "FRIDiagnostics")
d:setActivity("LWRDiagRA", 0.01, 2, rtt.globals.ORO_SCHED_RT)
LWRDiagRA = d:getPeer("LWRDiagRA")
LWRDiagRA:configure()




d:loadComponent("FRILA", "FRIComponent")
d:setActivity("FRILA", 0, 80, rtt.globals.ORO_SCHED_RT)
FRILA = d:getPeer("FRILA")
FRILA:getProperty("fri_port"):set(49940) -- for real
--FRILA:getProperty("fri_port"):set(49941)
FRILA:configure()

d:loadComponent("FRIRA", "FRIComponent")
d:setActivity("FRIRA", 0, 80, rtt.globals.ORO_SCHED_RT)
FRIRA = d:getPeer("FRIRA")
FRIRA:getProperty("fri_port"):set(49938) -- for real
--FRIRA:getProperty("fri_port"):set(49940)
FRIRA:configure()





d:loadComponent("FilterLA", "FLWRFilter")
d:setActivity("FilterLA", 0, 70, rtt.globals.ORO_SCHED_RT)
FilterLA = d:getPeer("FilterLA")
FilterLA:getProperty("FREQUENCY"):set(1.0)
FilterLA:configure()

d:loadComponent("FilterRA", "FLWRFilter")
d:setActivity("FilterRA", 0, 70, rtt.globals.ORO_SCHED_RT)
FilterRA = d:getPeer("FilterRA")
FilterRA:getProperty("FREQUENCY"):set(1.0)
FilterRA:configure()


d:loadComponent("MotionManager", "SMotionManager")
d:setActivity("MotionManager", 0.001, 60, rtt.globals.ORO_SCHED_RT)
MotionManager = d:getPeer("MotionManager")
MotionManager:configure()


d:loadComponent("LogLA", "SLogSaver")
d:setActivity("LogLA", 0, 20, rtt.globals.ORO_SCHED_RT)
LogLA = d:getPeer("LogLA")
LogLA:getProperty("ID"):set(0)
LogLA:configure()

d:loadComponent("LogRA", "SLogSaver")
d:setActivity("LogRA", 0, 20, rtt.globals.ORO_SCHED_RT)
LogRA = d:getPeer("LogRA")
LogRA:getProperty("ID"):set(1)
LogRA:configure()


d:connect("FRILA.RobotState", "LWRDiagLA.RobotState", rtt.Variable("ConnPolicy"))
d:connect("FRILA.FRIState", "LWRDiagLA.FRIState", rtt.Variable("ConnPolicy"))
d:connect("FRIRA.RobotState", "LWRDiagRA.RobotState", rtt.Variable("ConnPolicy"))
d:connect("FRIRA.FRIState", "LWRDiagRA.FRIState", rtt.Variable("ConnPolicy"))

d:connect("FRILA.RobotState"   , "FilterLA.RobotState" , rtt.Variable("ConnPolicy"))
d:connect("FRILA.FRIState"     , "FilterLA.FRIState"   , rtt.Variable("ConnPolicy"))
d:connect("FRILA.JointPosition", "FilterLA.FRIJointPos", rtt.Variable("ConnPolicy"))

d:connect("FRIRA.RobotState"   , "FilterRA.RobotState" , rtt.Variable("ConnPolicy"))
d:connect("FRIRA.FRIState"     , "FilterRA.FRIState"   , rtt.Variable("ConnPolicy"))
d:connect("FRIRA.JointPosition", "FilterRA.FRIJointPos", rtt.Variable("ConnPolicy"))


d:connect("FRILA.JointPosition", "MotionManager.FRIRealJointPosLA", rtt.Variable("ConnPolicy"))
d:connect("FRIRA.JointPosition", "MotionManager.FRIRealJointPosRA", rtt.Variable("ConnPolicy"))

d:connect("MotionManager.DesiredJointPosLA", "FilterLA.DesiredJointPos", rtt.Variable("ConnPolicy"))
d:connect("MotionManager.DesiredJointPosRA", "FilterRA.DesiredJointPos", rtt.Variable("ConnPolicy"))

d:connect("FilterLA.FilteredJointPos", "FRILA.JointPositionCommand", rtt.Variable("ConnPolicy"))
d:connect("FilterRA.FilteredJointPos", "FRIRA.JointPositionCommand", rtt.Variable("ConnPolicy"))


d:connect("FilterLA.Log", "LogLA.Log", rtt.Variable("ConnPolicy"))
d:connect("FilterRA.Log", "LogRA.Log", rtt.Variable("ConnPolicy"))

-- ROS in out
--d:stream("LWRDiag.Diagnostics",rtt.provides("ros"):topic("diagnostics"))

--d:stream("JntPub.joints_state",rtt.provides("ros"):topic("joint_states"))
--d:stream("FRIRA.KRL_CMD",rtt.provides("ros"):topic("lwr_arm_controller/fri_set_mode"))
--d:stream("FRIRA.CartesianWrench",rtt.provides("ros"):topic("cartesian_wrench"))
--d:stream("FilterRA.Log",rtt.provides("ros"):topic("log"))

-- Start of user code usercode
FRILA:start()
FRIRA:start()

LWRDiagLA:start()
LWRDiagRA:start()

FilterLA:start()
FilterRA:start()

LogLA:start()
LogRA:start()

MotionManager:start()

print("finished starting")
