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
-- find the deployer
-- script might be started from deployer directly or from any other component, hopefully having a deployer as peer
if tcName=="Deployer" then
		d=tc
else
		d=tc:getPeer("Deployer")
    -- TODO complain and exit if deployer not found
end

local prefix="la"

d:import("agni_rtt_services")

-- create LuaComponent
name = prefix.."kuka_controller"
d:loadComponent(name, "OCL::LuaComponent")
d:addPeer(name, "Deployer")
-- ... and get a handle to it
local kuka_controller = d:getPeer(name)
-- add service lua to new component named name
d:loadService(name,"Lua")
-- add controller services -- 
d:loadService(name,"controllerService")

 
-- load the Lua hooks
kuka_controller:exec_file(pathOfThisFile.."kuka_controller.lua")

-- configure the component
kuka_controller:getProperty("namespace"):set(prefix)
kuka_controller:getProperty("port"):set(49940)
kuka_controller:configure()

-- connect the component
kuka_controller:provides("controllerService"):connectIn("FILJNTPOS","MotionManager.DesiredJointPosLA")
kuka_controller:provides("controllerService"):connectOut("JNTPOS","MotionManager.FRIRealJointPosLA")
kuka_controller:provides("controllerService"):connectOut("LOG","LogLA.Log")

-- stat the component
kuka_controller:start()

print("finished loading "..prefix.."kuka")
