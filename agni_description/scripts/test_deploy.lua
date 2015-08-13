require "rttlib"
require "rttros"

-- get local path and add to package path to find our local scripts
local pathOfThisFile =debug.getinfo(1,"S").source:match[[^@?(.*[\/])[^\/]-$]]
if pathOfThisFile then
  package.path = pathOfThisFile..'/?.lua;' .. package.path 
end
-- agni functionalities (wait for peer, etc)
require "agni_tools"
-- motion manager include
dofile(pathOfThisFile.."/motion_manager_logger_deploy.lua")

tc=rtt.getTC()
tcName=tc:getName()
print (tcName)
if tcName=="lua" then
  d=tc:getPeer("Deployer")
elseif tcName=="Deployer" then
  d=tc
end

-- wait for motion manager to appear
found_motion_manager=wait_for("MotionManager",5)
-- wait for kuka to appear
found_rakuka_controller=wait_for("rakuka_controller",2)

if (found_motion_manager and found_rakuka_controller) then

  rakuka_controller = d:getPeer("rakuka_controller")
  -- add controller services -- 
  d:loadService("rakuka_controller","controllerService")
  -- connect the controller wrapper and the motion manager (this should stop the controller and restart it)
  print("rakuka connecting to MotionManager")
  rakuka_controller:provides("controllerService"):connectInTo("FILJNTPOS","MotionManager.DesiredJointPosRA")
  rakuka_controller:provides("controllerService"):connectOutTo("JNTPOS","MotionManager.FRIRealJointPosRA")
  rakuka_controller:provides("controllerService"):connectOutTo("LOG","LogRA.Log")
  
end

-- wait for kuka to appear
found_lakuka_controller=wait_for("lakuka_controller",2)

if (found_motion_manager and found_lakuka_controller) then

  lakuka_controller = d:getPeer("lakuka_controller")
  -- add controller services -- 
  d:loadService("lakuka_controller","controllerService")
  -- connect the controller wrapper and the motion manager
  print("lakuka connecting to MotionManager")
  lakuka_controller:provides("controllerService"):connectInTo("FILJNTPOS","MotionManager.DesiredJointPosLA")
  lakuka_controller:provides("controllerService"):connectOutTo("JNTPOS","MotionManager.FRIRealJointPosLA")
  lakuka_controller:provides("controllerService"):connectOutTo("LOG","LogLA.Log")
  
end



