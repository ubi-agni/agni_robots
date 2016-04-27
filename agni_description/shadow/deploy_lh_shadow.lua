require "rttlib"
require "rttros"
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
d:import("rtt_rospack")
ros = rtt.provides("ros")
agni_description_path = ros:find("agni_description")
assert(loadfile(agni_description_path.."/shadow/deploy_meta_driver_shadow.lua"))("lh")

