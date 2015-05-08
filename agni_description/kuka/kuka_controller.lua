require("rttlib")
require "rttros"
tc=rtt.getTC()
fri = 0
diag = 0
jsp = 0
filter = 0
in_portmap = {}
out_portmap = {}
  
iface_spec = {
   ports={},
   properties={
      { name='namespace', datatype='string', desc="namespace as prefix" },
      { name='port', datatype='int', desc="port on which FRI listens" },
   }
}
 
-- this create the interface
iface=rttlib.create_if(iface_spec)

function configureHook()
  iface=rttlib.create_if(iface_spec)
  namespace = iface.props.namespace:get()
  port = iface.props.port:get()
  
  local d=tc:getPeer("Deployer")
  tcName=tc:getName()
  
  d:import("lwr_fri")
  d:import("oro_joint_state_publisher")
  d:import("flwr_filter")
  
  diagname = namespace.."LWRDiag"
  d:loadComponent(diagname, "FRIDiagnostics")
  d:setActivity(diagname, 0.01, 2, rtt.globals.ORO_SCHED_RT)
  diag = d:getPeer(diagname)
  diag:configure()
  -- add dia to the parent component peers
  d:addPeer(tcName, diagname)        
  
 -- deploy FRI and advertize its input/output
  friname = namespace.."FRI"
  d:loadComponent(friname, "FRIComponent")
  d:setActivity(friname, 0, 80, rtt.globals.ORO_SCHED_RT)
  fri = d:getPeer(friname)
  fri:getProperty("fri_port"):set(port)
  fri:configure()
  -- add fri to the parent component peers
  d:addPeer(tcName, friname)  
  
  in_portmap['JointPosition'] = friname..".JointPositionCommand"
  out_portmap['JointPosition'] = friname..".JointPosition"

  -- deploy joint state publisher
  jspname = namespace.."JntPub"
  d:loadComponent(jspname, "JointStatePublisher")
  d:setActivity(jspname, 0.01, 10, rtt.globals.ORO_SCHED_RT)
  jsp = d:getPeer(jspname)
  joint_names=jsp:getProperty("joint_names")
  joint_names:get():resize(7)
  for i=0,6,1 do 
    joint_names[i]=namespace.."_arm_"..i.."_joint"
  end 
  jsp:configure()
  -- add jsp to the parent component peers
  d:addPeer(tcName, jspname) 

  -- deploy the filter and advertize its input/output
  filtername = namespace.."Filter"
  d:loadComponent(filtername, "FLWRFilter")
  d:setActivity(filtername, 0, 70, rtt.globals.ORO_SCHED_RT)
  filter = d:getPeer(filtername)
  filter:getProperty("FREQUENCY"):set(1.0)
  filter:configure()
  in_portmap['FilteredJointPosition'] = filtername..".DesiredJointPos"
  out_portmap['Log'] = filtername..".Log"
  -- add filter to the parent component peers
  d:addPeer(tcName, filtername) 
        
      
  d:connect(friname..".RobotState", diagname..".RobotState", rtt.Variable("ConnPolicy"))
  d:connect(friname..".FRIState", diagname..".FRIState", rtt.Variable("ConnPolicy"))
  d:connect(friname..".JointPosition", jspname..".JointPosition", rtt.Variable("ConnPolicy"))
  d:connect(friname..".JointVelocity", jspname..".JointVelocity", rtt.Variable("ConnPolicy"))
  d:connect(friname..".JointTorque", jspname..".JointEffort", rtt.Variable("ConnPolicy"))
  d:connect(friname..".RobotState", filtername..".RobotState", rtt.Variable("ConnPolicy"))
  d:connect(friname..".FRIState", filtername..".FRIState", rtt.Variable("ConnPolicy"))
  d:connect(friname..".JointPosition", filtername..".FRIJointPos", rtt.Variable("ConnPolicy"))
  d:connect(filtername..".FilteredJointPos", friname..".JointPositionCommand", rtt.Variable("ConnPolicy"))
    
  -- ROS in out
  local ros=rtt.provides("ros")
  d:stream(diagname..".Diagnostics",ros:topic(namespace.."/diagnostics"))
  d:stream(jspname..".joint_state",ros:topic(namespace.."/joint_states"))
        
  print(namespace.."kuka_controller configured")
  return true
end

function connectIn(intype,peerportname)
    local d=tc:getPeer("Deployer")
    local previous_running=tc:isActive()
    if tc:isActive() then
      stopHook()
    end
    if in_portmap[intype] then
      d:connect(in_portmap[intype], peerportname, rtt.Variable("ConnPolicy"))
    else
      print ("this type is not supported in this controller")
    end
   
    if tc:isActive() ~= previous_running then
      startHook()
    end
end

function connectOut(outtype,peerportname)
    local d=tc:getPeer("Deployer")
    local previous_running=tc:isActive()
    if tc:isActive() then
      stopHook()
    end
    if out_portmap[outtype] then
      d:connect(out_portmap[outtype], peerportname, rtt.Variable("ConnPolicy"))
    else
      print ("this type is not supported in this controller")
    end
   
    if tc:isActive() ~= previous_running then
      startHook()
    end
end

 
function startHook()
  -- stop all peers except Deployer
  local peers=tc:getPeers()
  for _,peername in pairs(peers) do
    if peername~="Deployer" then
      tc:getPeer(peername):start()
    end
  end
  --self.running=true
  return true
end


function stopHook()
  -- stop all peers except Deployer
  local peers=tc:getPeers()
  for _,peername in pairs(peers) do
    if peername~="Deployer" then
      tc:getPeer(peername):stop()
    end
  end
  --self.running=false
  return true
end

 
-- Ports and properties are the only elements which are not
-- automatically cleaned up. This means this must be done manually for
-- long living components:
function cleanupHook()
  -- unload all peers except Deployer
  local d=tc:getPeer("Deployer")
  local peers=tc:getPeers()
  for _,peername in pairs(peers) do
    if peername~="Deployer" then
      d:unloadComponent(peername)
    end
  end

  rttlib.tc_cleanup()
   
end

