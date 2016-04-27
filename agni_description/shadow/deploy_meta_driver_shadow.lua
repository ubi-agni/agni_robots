require "rttlib"
require "rttros"

local pathOfThisFile =debug.getinfo(1,"S").source:match[[^@?(.*[\/])[^\/]-$]]

if pathOfThisFile then
  print("path of this file :"..pathOfThisFile)
  package.path = pathOfThisFile..'?.lua;' .. package.path 
end

tc=rtt.getTC()
tcName=tc:getName()
-- find the deployer
-- script might be started from deployer directly or from any other component, hopefully having a deployer as peer
if tcName=="Deployer" then
  d=tc
else
  d=tc:getPeer("Deployer")
    -- TODO complain and exit if deployer not found
end

-- get the prefix and port from args
local prefix=""
local prefix_arg,port_arg=...
if prefix_arg then
  print("using prefix "..prefix_arg)
  prefix = prefix_arg
end

-- create LuaComponent
name = prefix.."shadow_driver"
d:loadComponent(name, "OCL::LuaComponent")
d:addPeer(name, "Deployer")
-- ... and get a handle to it
local shadow_driver = d:getPeer(name)
-- add service lua to new component named name
d:loadService(name,"Lua")
 
-- load the Lua hooks
shadow_driver:exec_file(pathOfThisFile.."meta_driver_shadow.lua")

-- configure the component
shadow_driver:getProperty("namespace"):set(prefix)
shadow_driver:getProperty("controller_name"):set(prefix.."/shadow_driver")

shadow_driver:configure()

-- stat the component
shadow_driver:start()

print("finished loading "..prefix.."shadow_driver")
