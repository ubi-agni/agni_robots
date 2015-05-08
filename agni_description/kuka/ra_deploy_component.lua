require "rttlib"
require "rttros"

local pathOfThisFile =debug.getinfo(1,"S").source:match[[^@?(.*[\/])[^\/]-$]]

if pathOfThisFile then
  print("path of this file :"..pathOfThisFile)
  package.path = pathOfThisFile..'?.lua;' .. package.path 
end

require("kuka")

tc=rtt.getTC()
tcName=tc:getName()
print (tcName)
if tcName=="lua" then
		d=tc:getPeer("Deployer")
elseif tcName=="Deployer" then
		d=tc
end

local prefix="ra"

-- create LuaComponents
d:loadComponent(prefix.."kuka_controller", "OCL::LuaComponent")
d:addPeer(prefix.."kuka_controller", "Deployer")
--... and get references to them
local kuka_controller = d:getPeer(prefix.."kuka_controller")
--add service lua 
d:loadService(prefix.."kuka_controller","Lua")
 
-- load the Lua hooks
kuka_controller:exec_file(pathOfThisFile.."kuka_controller.lua")

-- configure the component
kuka_controller:getProperty("namespace"):set(prefix)
kuka_controller:getProperty("port"):set(49938)
kuka_controller:configure()

-- add Connect services -- DOES NOT WORK.
--kuka_controller:exec_file(pathOfThisFile.."connect_services.lua")
--kuka_controller:connectIn(d,"FilteredJointPosition","MotionManager.DesiredJointPosLA")
--kuka_controller:connectOut(d,"JointPosition","MotionManager.FRIRealJointPosLA")
--kuka_controller:connectOut(d,"Log","LogLA.Log")

d:connect(prefix.."FRI.JointPosition", "MotionManager.FRIRealJointPosRA", rtt.Variable("ConnPolicy"))
d:connect("MotionManager.DesiredJointPosRA", prefix.."Filter.DesiredJointPos", rtt.Variable("ConnPolicy"))
d:connect(prefix.."Filter.Log", "LogRA.Log", rtt.Variable("ConnPolicy"))

-- stat the component
kuka_controller:start()

print("finished loading "..prefix.."kuka")
