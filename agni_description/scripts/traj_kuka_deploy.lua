require "rttlib"
require "rttros"

-- get local path and add to package path to find our local scripts
pathOfThisFile =debug.getinfo(1,"S").source:match[[^@?(.*[\/])[^\/]-$]]
if pathOfThisFile then
  package.path = pathOfThisFile..'/?.lua;' .. package.path 
end
-- agni functionalities (wait for peer, etc)
require "agni_tools"

function traj_controller_deploy(d,namespace)
  -- wait for kuka to appear
  found_kuka_controller=wait_for(namespace.."kuka_controller",2)
  if (found_kuka_controller) then

    -- create LuaComponent
    name = namespace.."traj_controller"
    d:loadComponent(name, "OCL::LuaComponent")
    d:addPeer(name, "Deployer")
    -- ... and get a handle to it
    local traj_controller = d:getPeer(name)
    -- add service lua to new component named name
    d:loadService(name,"Lua")
     
    -- load the Lua hooks
    traj_controller:exec_file(pathOfThisFile.."/traj_controller.lua")

    -- configure the component
    traj_controller:getProperty("namespace"):set(namespace)
    traj_controller:getProperty("controller_name"):set("arm_trajectory_controller")
    traj_controller:configure()

    -- stat the component
    traj_controller:start()

    -- wait for motion manager to appear
    found_traj_controller=wait_for(name,3)
    
    if (found_traj_controller) then
    
      kuka_controller = d:getPeer(namespace.."kuka_controller")
      -- add controller services -- 
      d:loadService(namespace.."kuka_controller","controllerService")
      -- connect the controller wrapper and the traj controller (this should stop the controller and restart it)
      print(namespace.."kuka connecting to "..name)

      kuka_controller:provides("controllerService"):connectIn("FILJNTPOS",name,"JNTPOS")
      kuka_controller:provides("controllerService"):connectOut("JNTPOS",name,"JNTPOS")
      kuka_controller:provides("controllerService"):connectOut("JNTPOS",name,"JNTPOS2")

    end
  end
end

tc=rtt.getTC()
tcName=tc:getName()
print (tcName)
if tcName=="lua" then
  d=tc:getPeer("Deployer")
elseif tcName=="Deployer" then
  d=tc
end

traj_controller_deploy(d,"ra")
traj_controller_deploy(d,"la")



