require "rttlib"
require "rttros"


local pathOfThisFile =debug.getinfo(1,"S").source:match[[^@?(.*[\/])[^\/]-$]]

if pathOfThisFile then
  print("path of this file :"..pathOfThisFile)
  package.path = pathOfThisFile..'kuka/?.lua;' .. package.path 
end


tc=rtt.getTC()
d=tc:getPeer("Deployer")

-- ROS integration
d:import("rtt_rosnode")
d:import("rtt_roscomm")
d:import("rtt_std_msgs")
d:import("rtt_sensor_msgs")
d:import("rtt_diagnostic_msgs")
d:import("rtt_control_msgs")
d:import("agni_rtt_rosdeployment")
-- End of user code

d:loadService("Deployer","rosluadeployment")

print("finished starting")
