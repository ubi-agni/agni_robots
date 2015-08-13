require "rttlib"
require "rttros"

-- get local path and add to package path to find our local scripts
pathOfThisFile =debug.getinfo(1,"S").source:match[[^@?(.*[\/])[^\/]-$]]
if pathOfThisFile then
  package.path = pathOfThisFile..'/?.lua;' .. package.path 
end
-- agni functionalities (wait for peer, etc)
require "agni_tools"

function controller_manager_deploy(d,namespace)
  
  d:import("rtt_controller_manager_msgs")
  
  -- create LuaComponent
  name = namespace.."controller_manager"
  d:loadComponent(name, "OCL::LuaComponent")
  d:addPeer(name, "Deployer")
  -- ... and get a handle to it
  local cm = d:getPeer(name)
  -- add service lua to new component named name
  -- d:loadService(name,"Lua")
   
  -- load the Lua hooks
  --traj_controller:exec_file(pathOfThisFile.."/traj_controller.lua")

  -- configure the component
  prop=rtt.Property("string", "namespace", "Namespace")
  cm:addProperty(prop)
  cm:getProperty("namespace"):set(namespace)
  cm:configure()

  -- add controller manager services and ros access to it  -- 
  d:loadService(name,"controllerManagerService")
  --d:loadService(name,"rosservice")

  --cm:provides("rosservice"):connect("controllerManagerService.listControllers",
  --  namespace.."/controller_manager/list_controllers", "controller_manager_msgs/ListControllers")

  -- stat the component
  cm:start()
  
  -- wait for kuka to appear
  found_kuka_controller=wait_for(namespace.."kuka_controller",2)
  if (found_kuka_controller) then
    d:addPeer(name, namespace.."kuka_controller")
  end
  -- wait for traj controller to appear
  found_traj_controller=wait_for(namespace.."traj_controller",2)
  
  if (found_traj_controller) then
    d:addPeer(name, namespace.."traj_controller")
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

controller_manager_deploy(d,"ra")
controller_manager_deploy(d,"la")



