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
d:import("rtt_control_msgs")

-- Start of user code imports
d:import("s_motion_manager")
d:import("s_log_saver")
d:import("gazebo_attach_controller")


local pathOfThisFile = ...
print("path of this file :"..pathOfThisFile)

package.path = pathOfThisFile..'/kuka/?.lua;' .. package.path 

d:loadComponent("Grasp", "GazeboAttachController")
d:setActivity("Grasp", 0, 20, rtt.globals.ORO_SCHED_RT)
Grasp = d:getPeer("Grasp")

d:loadComponent("LogRA", "SLogSaver")
d:setActivity("LogRA", 0, 20, rtt.globals.ORO_SCHED_RT)
LogRA = d:getPeer("LogRA")
LogRA:configure()

d:loadComponent("MotionManager", "SMotionManager")
d:setActivity("MotionManager", 0.001, 60, rtt.globals.ORO_SCHED_RT)
MotionManager = d:getPeer("MotionManager")
MotionManager:configure()

require("kuka")
ra_kuka = KukaControllers.create("ra",49938)
ra_kuka:init(d)
ra_kuka:deploy(d)
ra_kuka:connectIn(d,"FilteredJointPosition","MotionManager.DesiredJointPosRA")
ra_kuka:connectOut(d,"JointPosition","MotionManager.FRIRealJointPosRA")
ra_kuka:connectOut(d,"Log","LogRA.Log")


-- ROS in out
ros=rtt.provides("ros")
d:stream("Grasp.Attach",ros:topic("/gazebo_attach"))
d:stream("Grasp.Attached",ros:topic("/gazebo_attached"))


ra_kuka:start()
LogRA:start()
MotionManager:start()

print("finished starting")
