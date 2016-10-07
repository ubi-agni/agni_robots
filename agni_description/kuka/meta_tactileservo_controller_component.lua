require "rttlib"
require "rttros"
agni_description_path = rttros.find_rospack("agni_description") -- ##CHANGE ME##
package.path = agni_description_path..'/?.lua;' .. package.path 
lua_path = rttros.find_rospack("agni_rtt_rosdeployment")
package.path = lua_path..'/scripts'..'/?.lua;' .. package.path 
require "agni_tools"

function tactileservo_controller_deploy(d, namespace, meta_component_name, meta_component_file, lowlevelctrl_name)  -- ##CHANGE ME##\

  name = namespace..meta_component_name
  if hasPeer(d, name) then
    print(name.." already loaded")
  else
    -- create a LuaComponent
    d:loadComponent(name, "OCL::LuaComponent") 
    d:addPeer(name, "Deployer")
    -- ... and get a handle to it
    local mc = d:getPeer(name)
    -- add service lua to new component named name
    d:loadService(name, "Lua")
 
    -- load the Lua hooks
    mc:exec_file(agni_description_path..meta_component_file) -- ##CHANGE ME##
    
    -- configure the component
    mc:getProperty("namespace"):set(namespace)  
    mc:getProperty("controller_name"):set(name)
    mc:configure()

    -- stat the component
    mc:start()

    -- optionally wait, find and connect to other known meta-components here 
    -- a controller meta-component could directly connect to the lowlevel-driver here
	-- wait for traj controller to appear
      mc=wait_for(name,3)
      
      if (mc) then
      
        lowlevel_controller = d:getPeer(lowlevelctrl_name)
        -- add controller services -- 
        d:loadService(lowlevelctrl_name,"controllerService")
        -- connect the controller wrapper and the traj controller (this should stop the controller and restart it)
        print(lowlevelctrl_name.." connecting to "..name)

        lowlevel_controller:provides("controllerService"):connectIn("CMDJNT",name,"CMDJNT")
		lowlevel_controller:provides("controllerService"):connectOut("CURJNT",name,"CURJNT")
		lowlevel_controller:provides("controllerService"):connectOut("LOG",name,"LOG")

      end

  end
end

function tactileservo_controller_undeploy(d, namespace, meta_component_name)  -- ##CHANGE AT LEAST THE function NAME##
  -- access the mc
  name = namespace..meta_component_name
  if hasPeer(d, name) then
    mc = d:getPeer(name)
    -- stop, and clen-up the meta-component
    mc:stop()
    mc:cleanup()
    d:unloadComponent(name)
    print ("undeployed "..name)
  else
    print (name.." not found")
  end
end
