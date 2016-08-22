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
d:import("rtt_std_msgs") --for gazebo_attach
d:import("rtt_sensor_msgs")
d:import("rtt_diagnostic_msgs")

-- Start of user code imports
d:import("famula_active_inspection") 
d:import("flwr_filter")
d:import("s_log_saver")
d:import("gazebo_attach_controller")

d:import("lwr_fri")

d:loadComponent("LWRDiag", "FRIDiagnostics")
d:setActivity("LWRDiag", 0.01, 2, rtt.globals.ORO_SCHED_RT)
LWRDiag = d:getPeer("LWRDiag")
LWRDiag:configure()

d:loadComponent("FRIRA", "FRIComponent")
d:setActivity("FRIRA", 0, 80, rtt.globals.ORO_SCHED_RT)
FRIRA = d:getPeer("FRIRA")
FRIRA:getProperty("fri_port"):set(49938) 
FRIRA:configure()

d:loadComponent("FilterRA", "FLWRFilter")
d:setActivity("FilterRA", 0, 70, rtt.globals.ORO_SCHED_RT)
FilterRA = d:getPeer("FilterRA")
FilterRA:getProperty("CUTOFF_FREQUENCY"):set(30.0)
FilterRA:getProperty("TIMESTEP"):set(0.001)
FilterRA:getProperty("MODE"):set(1)
FilterRA:configure()

d:loadComponent("LogRA", "SLogSaver")
d:setActivity("LogRA", 0, 20, rtt.globals.ORO_SCHED_RT)
LogRA = d:getPeer("LogRA")
LogRA:getProperty("ID"):set(0)
LogRA:configure()

d:loadComponent("Attach", "GazeboAttachController")
d:setActivity("Attach", 0.1, 10, rtt.globals.ORO_SCHED_RT)
Attach = d:getPeer("Attach")
Attach:getProperty("ref_model_name"):set("r_kukaR")
Attach:getProperty("ref_link_name"):set("ra_arm_7_link")
Attach:getProperty("tgt_model_name"):set("beer_line")
Attach:getProperty("tgt_link_name"):set("beer_line_link")
Attach:configure()

d:loadComponent("FTest", "FamulaActiveInspection")
d:setActivity("FTest", 0.001, 60, rtt.globals.ORO_SCHED_RT)
FTest = d:getPeer("FTest")
d:addPeer("FTest", "Attach")
FTest:configure()

d:connect("FRIRA.RobotState", "LWRDiag.RobotState", rtt.Variable("ConnPolicy"))
d:connect("FRIRA.FRIState", "LWRDiag.FRIState", rtt.Variable("ConnPolicy"))

d:connect("FRIRA.RobotState", "FilterRA.RobotState", rtt.Variable("ConnPolicy"))
d:connect("FRIRA.FRIState", "FilterRA.FRIState", rtt.Variable("ConnPolicy"))
d:connect("FRIRA.JointPosition", "FilterRA.FRIJointPos", rtt.Variable("ConnPolicy"))


d:connect("FilterRA.CurrentJoint", "FTest.RealJointRA", rtt.Variable("ConnPolicy"))

d:connect("FTest.DesiredJointRA", "FilterRA.DesiredJoint", rtt.Variable("ConnPolicy"))

d:connect("FilterRA.FilteredJointPos", "FRIRA.JointPositionCommand", rtt.Variable("ConnPolicy"))



d:connect("FilterRA.Log", "LogRA.Log", rtt.Variable("ConnPolicy"))
--d:connect("FRI.FRIState", "LogRA.FRIState", rtt.Variable("ConnPolicy"))

-- ROS in out
ros=rtt.provides("ros")
d:stream("LWRDiag.Diagnostics",ros:topic("diagnostics"))
d:stream("Attach.Attach", rtt.provides("ros"):topic("/gazebo_attach"))
d:stream("Attach.Attached", rtt.provides("ros"):topic("/gazebo_attached"))

-- Start of user code usercode
FRIRA:start()
LWRDiag:start()
FilterRA:start()
LogRA:start()
FTest:start()
Attach:start()

print("finished starting")
