require "rttlib"
require "rttros"

tc=rtt.getTC()
tcName=tc:getName()
print (tcName)
if tcName=="lua" then
  d=tc:getPeer("Deployer")
elseif tcName=="Deployer" then
  d=tc
end

-- load the robot description on the parameters server
rd_found=os.execute("rosparam list | grep /lh/robot_description -q")
if rd_found ~= 0 then
  print ("Please wait, loading robot_description for left hand")
  r=os.execute("ROS_NAMESPACE=lh roslaunch agni_description tool_upload.launch tool_type:=shadow_motor_left prefix:=l")
  if r ~= 0 then
    print ("Error uploading Left shadow robot description")
    return false
  end
end

-- import a generic meta_driver component utility (instantiate)
agni_description_path = rttros.find_rospack("agni_description")
package.path = agni_description_path..'/shadow'..'/?.lua;' .. package.path 
require("meta_driver_shadow_component")

shadow_driver_deploy(d, "lh", "shadow_driver", "/shadow/meta_driver_shadow.lua", true)
