require "rttlib"
require "rttros"

tc=rtt.getTC()
tcName=tc:getName()
if tcName=="lua" then
		d=tc:getPeer("Deployer")
elseif tcName=="Deployer" then
		d=tc
end
-- stop, cleanup and unload all the components
d:kickOutAll() --crashes if one component already unloaded other components within its cleanup hook

print("finished cleaning up")
