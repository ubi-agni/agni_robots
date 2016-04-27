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

d:import("agni_rtt_services")

-- get the prefix and port from args
local prefix=""
local prefix_arg,port_arg=...
if prefix_arg then
  print("using prefix "..prefix_arg)
  prefix = prefix_arg
end

if port_arg then
  print("using port "..port_arg)
  port = port_arg
else
  port = 49938
end

-- create LuaComponent
name = prefix.."kuka_driver"
d:loadComponent(name, "OCL::LuaComponent")
d:addPeer(name, "Deployer")
-- ... and get a handle to it
local kuka_driver = d:getPeer(name)
-- add service lua to new component named name
d:loadService(name,"Lua")
 
-- load the Lua hooks
kuka_driver:exec_file(pathOfThisFile.."meta_driver_kuka.lua")

-- configure the component
kuka_driver:getProperty("namespace"):set(prefix)
kuka_driver:getProperty("port"):set(port)
kuka_driver:getProperty("controller_name"):set(prefix.."/kuka_driver")

kuka_driver:configure()

-- stat the component
kuka_driver:start()

print("finished loading "..prefix.."kuka_driver")
