require "rttlib"
require "rttros"

local pathOfThisFile =debug.getinfo(1,"S").source:match[[^@?(.*[\/])[^\/]-$]]

if pathOfThisFile then
  print("path of this file :"..pathOfThisFile)
  package.path = pathOfThisFile..'?.lua;' .. package.path 
end

require("kuka")

function contains(tab, element)
  for _, value in pairs(tab) do
    --print ("peer: ",value)
    if value == element then
      return true
    end
  end
  return false
end

function sleep(s)
  local ntime = os.time() + s
  repeat until os.time() > ntime
end


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

-- wait for motion manager to appear
i=0
found_motion_manager=false
while not found_motion_manager do
      print("waiting MotionManager ", i)
      peers = d:getPeers()
      if contains(peers,"MotionManager") then 
--      ret= pcall(d:getPeer("MotionManager")) 
        found_motion_manager=true
        break
      else
        print("MotionManager not found")
      end
      i = i + 1
      sleep(1)

      if i > 5 then
        break
      end
end
if found_motion_manager then
  m=d:getPeer("MotionManager")
  while not m:isRunning() do
      print("waiting MotionManager running", i)
      i = i + 1
      sleep(1)
  end
  -- connect the component
  print("connecting to MotionManager")
  kuka_controller:provides("controllerService"):connectIn("FILJNTPOS","MotionManager.DesiredJointPosLA")
  kuka_controller:provides("controllerService"):connectOut("JNTPOS","MotionManager.FRIRealJointPosLA")
  kuka_controller:provides("controllerService"):connectOut("LOG","LogLA.Log")
  
end


-- stat the component
kuka_controller:start()

print("finished loading "..prefix.."kuka")
