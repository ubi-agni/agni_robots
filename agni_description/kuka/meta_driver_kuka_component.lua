require "rttlib"
require "rttros"

agni_description_path = rttros.find_rospack("agni_description")
package.path = agni_description_path..'/?.lua;' .. package.path 
lua_path = rttros.find_rospack("agni_rtt_rosdeployment")
package.path = lua_path..'/scripts'..'/?.lua;' .. package.path 
-- agni functionalities (wait for peer, etc)
require "agni_tools"

function kuka_driver_deploy(d, namespace, driver_name, driver_script, port, timestep, cutoff_freq, fake)
  name = namespace..driver_name
  -- backward compatibility with default values for timestep and cutoff_freq
  timestep = timestep or 0.001
  cutoff_freq = cutoff_freq or 10.0
  fake = fake or false
  
  if hasPeer(d, name) then
    print(name.." already loaded")
  else
    d:import("agni_rtt_services" )
    -- create LuaComponent
    d:loadComponent(name, "OCL::LuaComponent")
    d:addPeer(name, "Deployer")
    -- ... and get a handle to it
    local driver = d:getPeer(name)
    -- add service lua to new component named name
    d:loadService(name,"Lua")
     
    -- load the Lua hooks
    driver:exec_file(agni_description_path..driver_script)
    
    -- configure the component
    driver:getProperty("namespace"):set(namespace)
    driver:getProperty("fake"):set(fake)
    driver:getProperty("port"):set(port)
    driver:getProperty("timestep"):set(timestep)
    driver:getProperty("filter_cutoff_freq"):set(cutoff_freq)
    driver:getProperty("controller_name"):set(name)

    if driver:configure() then
      -- stat the component
      driver:start()
      print("finished loading "..name)
    else
      d:unloadComponent(name)
      print(name.." could not be loaded")
    end
  end
end

function kuka_driver_undeploy(d, namespace, driver_name)
  -- access the cm
  name = namespace..driver_name
  if hasPeer(d, name) then
    driver = d:getPeer(name)
    -- stat the component
    driver:stop()
    driver:cleanup()
    d:unloadComponent(name)
    print ("undeployed "..name)
  else
    print (name.." not found")
  end
end


