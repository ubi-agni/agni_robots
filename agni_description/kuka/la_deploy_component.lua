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
lwr_bringup_path = ros:find("lwr_bringup")
assert(loadfile(lwr_bringup_path.."/scripts/kuka_deploy.lua"))("la",49940)
