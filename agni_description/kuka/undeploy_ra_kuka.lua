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

-- import a generic meta_driver component utility (instantiate)
agni_description_path = rttros.find_rospack("agni_description")
package.path = agni_description_path..'/kuka'..'/?.lua;' .. package.path 
require("meta_driver_kuka_component")

kuka_driver_undeploy(d, "ra", "kuka_driver")


