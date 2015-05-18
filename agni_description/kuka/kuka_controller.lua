require("rttlib")
require "rttros"
tc=rtt.getTC()
fri = 0
diag = 0
jsp = 0
filter = 0
  
iface_spec = {
   ports={},
   properties={
      { name='namespace', datatype='string', desc="namespace as prefix" },
      { name='port', datatype='int', desc="port on which FRI listens" },
      { name='in_portmap', datatype='agni_rtt_services/ControlIOMap', desc="input port mapping" },
      { name='out_portmap', datatype='agni_rtt_services/ControlIOMap', desc="output port mapping" },
   }
}
 
-- this create the interface
iface=rttlib.create_if(iface_spec)

function configureHook()
  
  local d=tc:getPeer("Deployer")
  tcName=tc:getName()
  
  d:import("lwr_fri")
  d:import("oro_joint_state_publisher")
  d:import("flwr_filter")
  d:import("agni_rtt_services")
  
  iface=rttlib.create_if(iface_spec)
  namespace = iface.props.namespace:get()
  port = iface.props.port:get()
  local in_portmap={}
  local out_portmap={}
  
 
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
  
  
  in_portmap['JNTPOS'] = friname..".JointPositionCommand"
  out_portmap['JNTPOS'] = friname..".JointPosition"

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
  
  
  in_portmap['FILJNTPOS'] = filtername..".DesiredJointPos"
  out_portmap['LOG'] = filtername..".Log"
  -- add filter to the parent component peers
  d:addPeer(tcName, filtername) 
        
        
  -- store the mapping in the properties
  
  storeMapping("in_portmap",in_portmap)
  storeMapping("out_portmap",out_portmap)
  
      
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

-- convert table into property of type ControlIOMap
function storeMapping(propname,mymap)

  portmap = tc:getProperty(propname):get() 
  local count = 0
  -- count entries
  for _ in pairs(mymap) do count = count + 1 end
  portmap.type:resize(count)
  portmap.portname:resize(count)
  count = 0
  count = 0
  -- assign entries
  for k,v in pairs(mymap) do 
    portmap.type[count]=k
    portmap.portname[count]=v
    count=count+1
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
      d:getPeer(peername):cleanup()
    end
  end
  for _,peername in pairs(peers) do
    if peername~="Deployer" then
      d:unloadComponent(peername)
    end
  end

  rttlib.tc_cleanup()
   
end

