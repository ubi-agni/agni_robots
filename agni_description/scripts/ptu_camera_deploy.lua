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

-- Start of user code imports
d:import("trajectory_filter")
d:import("ptu_controller")
d:import("motion_generator")

-- ptu 
ptuname = "PTUflir"
d:loadComponent(ptuname, "Ptu_controller")
d:setActivity(ptuname, 0, 10, rtt.globals.ORO_SCHED_RT)
ptu = d:getPeer(ptuname)

max_vel=ptu:getProperty("max_velocity")
max_vel:set(2.0)
ptu:configure()

-- connect to ptu_driver
local ros=rtt.provides("ros")
d:stream(ptuname..".JointState",ros:topic("/joint_states"))
d:stream(ptuname..".JointStateCmd",ros:topic("/ptu/cmd"))

-- joint filter
d:loadComponent("TrajFilter", "TrajectoryFilter")
d:setActivity("TrajFilter", 0, 60, rtt.globals.ORO_SCHED_RT)
TrajFilter = d:getPeer("TrajFilter")
TrajFilter:getProperty("Frequency"):set(20.0)
TrajFilter:getProperty("Ndof"):set(2)
TrajFilter:configure()

-- motion generator
d:loadComponent("MotionGen", "MotionGenerator")
d:setActivity("MotionGen", 0.01, 60, rtt.globals.ORO_SCHED_RT)
MotionGen = d:getPeer("MotionGen")
MotionGen:getProperty("Ndof"):set(2)
MotionGen:configure()

-- ptu to filter connection
d:connect("PTUflir.JointPos", "TrajFilter.JointPos", rtt.Variable("ConnPolicy"))
d:connect("TrajFilter.FilteredJointPos", "PTUflir.DesiredJointPos", rtt.Variable("ConnPolicy"))
d:connect("TrajFilter.FilteredJointVel", "PTUflir.DesiredJointVel", rtt.Variable("ConnPolicy"))

d:connect("MotionGen.DesiredJointPos", "TrajFilter.DesiredJointPos", rtt.Variable("ConnPolicy"))
d:connect("PTUflir.JointPos", "MotionGen.JointPos", rtt.Variable("ConnPolicy"))

TrajFilter:start()
ptu:start()
MotionGen:start()
print("finished starting ptu controller and filter")
