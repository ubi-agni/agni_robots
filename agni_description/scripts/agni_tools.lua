-- agni lua functionalities 

require "rttlib"
function contains(tab, element)
  for _, value in pairs(tab) do
    --print ("peer: ",value)
    if value == element then
      return true
    end
  end
  return false
end

function sleep(s)
  local ntime = os.time() + s
  repeat until os.time() > ntime
end

-- timeout is seconds integer
function wait_for(peer,timeout)
  i=0
  ret=false
  print("waiting for ",peer)
  while true do
    peers = d:getPeers()
    if contains(peers,peer) then 
      m=d:getPeer(peer)
      print("waiting until ",peer," is running")
      i=0
      while not m:isRunning() do
          i = i + 1
          sleep(1)
          print (".")
          if i > timeout then
            print(peer," is not running")
            break
          end
      end
      if m:isRunning() then ret=true end
      break      
    end
    i = i + 1
    sleep(1)
    print (".")
    if i > timeout then
      print(peer," not found")
      break
    end
  end
  return ret
end
