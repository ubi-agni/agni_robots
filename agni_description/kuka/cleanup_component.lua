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
d:kickOutAll()
-- stop all peers except lua
--local peers=tc:getPeers()
--for _,peername in pairs(peers) do
  --if peername~="lua" then
    --d:getPeer(peername):stop()
  --end
--end


--for _,peername in pairs(peers) do
  --if peername~="lua" then
    --d:getPeer(peername):cleanup()
  --end
--end


--for _,peername in pairs(peers) do
  --if peername~="lua" then
    --d:unloadComponent(peername)
  --end
--end


print("finished cleaning up")
