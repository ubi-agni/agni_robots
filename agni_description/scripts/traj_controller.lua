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
      { name='in_portmap', datatype='agni_rtt_services/ControlIOMap', desc="input port mapping" },
      { name='out_portmap', datatype='agni_rtt_services/ControlIOMap', desc="output port mapping" },
      { name='resources', datatype='strings', desc="joints controlled by this controller" },
      { name='controller_name', datatype='string', desc="controller name" },
      { name='controller_type', datatype='string', desc="controller type ex: position_controllers/JointTrajectoryController" },
   }
}
 
-- this create the interface
iface=rttlib.create_if(iface_spec)

function configureHook()
  
  local d=tc:getPeer("Deployer")
  tcName=tc:getName()
  
  if tcName=="lua" then
    d=tc:getPeer("Deployer")
  elseif tcName=="Deployer" then
    d=tc
  end

  d:import("rtt_actionlib")
  d:import("rtt_actionlib_msgs")
  d:import("rtt_rosnode")
  d:import("rtt_roscomm")
  d:import("rtt_std_msgs")
  d:import("rtt_sensor_msgs")
  d:import("rtt_diagnostic_msgs")
  d:import("rtt_control_msgs")
  d:import("rtt_trajectory_msgs")

  d:import("agni_rtt_services")

  iface=rttlib.create_if(iface_spec)
  namespace = iface.props.namespace:get()

  controller_name = iface.props.controller_name:get()
  controller_type = tc:getProperty("controller_type")
  controller_type:set("position_controllers/JointTrajectoryController")
  resources = tc:getProperty("resources")
  
  local in_portmap={}
  local out_portmap={}
  
  
  print ("starting "..namespace.." trajectory controller")

  -- ROS integration

  --d:import("rtt_dot_service")
  --d:loadService("Deployer","dot")

  -- Start of user code imports
  d:import("internal_space_spline_trajectory_generator")
  d:import("internal_space_spline_trajectory_action")

  jtg_name = namespace.."JntTrajGen"
  d:loadComponent(jtg_name,"InternalSpaceSplineTrajectoryGenerator")
  d:setActivity(jtg_name, 0.001, 5, rtt.globals.ORO_SCHED_RT)
  JntTrajGen = d:getPeer(jtg_name)
  number_of_joints=JntTrajGen:getProperty("number_of_joints")
  number_of_joints:set(7)
  JntTrajGen:configure()

  -- add traj gen to the parent component peers
  d:addPeer(tcName, jtg_name)

  jta_name = namespace.."JntTrajAction"
  d:loadComponent(jta_name, "InternalSpaceSplineTrajectoryAction")
  d:setActivity(jta_name, 0.1, 2, rtt.globals.ORO_SCHED_RT)
  JntTrajAct = d:getPeer(jta_name)
  nbr_of_joints=JntTrajAct:getProperty("number_of_joints")
  nbr_of_joints:set(7)
  joint_names=JntTrajAct:getProperty("joint_names")
  lower_limits=JntTrajAct:getProperty("lower_limits")
  upper_limits=JntTrajAct:getProperty("upper_limits")
  joint_names:get():resize(7)
  lower_limits:get():resize(7)
  upper_limits:get():resize(7)

  lowlims = {-2.96 , -2.09, -2.96, -2.09, -2.96, -2.09, -2.96}
  uplims = {2.96 , 2.09, 2.96, 2.09, 2.96, 2.09, 2.96}

  resources:get():resize(7)

  for i=0,6,1 do 
    joint_names[i] = namespace.."_arm_"..i.."_joint"
    resources[i] = joint_names[i]
    lower_limits[i] = lowlims[i+1]
    upper_limits[i] = uplims[i+1]
  end


  -- TODO move that into a yaml and load it via the launch files that also spawns the lua
  --JntTrajAct.loadService("rosparam")
  --JntTrajAct.rosparam.getAll()
  --JntTrajAct.rosparam.getParam("~/JntTrajAction/upper_limits", "upper_limits")
  --JntTrajAct.rosparam.getParam("~/JntTrajAction/lower_limits", "lower_limits")
  -- yaml file
  --JntTrajGen:
  --  number_of_joints: 7
  --JntTrajAction:
  --  joint_names: [joint1, joint2, joint3, joint4, joint5, joint6, joint7]
  --  lower_limits: [-2.96 , -2.09, -2.96, -2.09, -2.96, -2.09, -2.96]
  --  upper_limits: [2.96 , 2.09, 2.96, 2.09, 2.96, 2.09, 2.96]

  -- create the action server
  JntTrajAct:loadService("actionlib")
  JntTrajAct:provides("actionlib"):connect("/"..namespace.."/arm_trajectory_controller/follow_joint_trajectory")
  JntTrajAct:configure()

  -- add traj action to the parent component peers
  d:addPeer(tcName, jta_name)
  
  -- internal connections
  d:addPeer(jta_name, jtg_name)
  d:connect(jta_name..".trajectoryPtr", jtg_name..".trajectoryPtr", rtt.Variable("ConnPolicy"))
  d:connect(jta_name..".JointPositionCommand", jtg_name..".JointPositionCommand", rtt.Variable("ConnPolicy"))

  -- ROS in out
  ros=rtt.provides("ros")
  d:stream(jta_name..".command",ros:topic("/"..namespace.."/arm_trajectory_controller/command"))
  d:stream(jta_name..".state",ros:topic("/"..namespace.."/arm_trajectory_controller/state"))

  -- advertize ports for the compound controller
  out_portmap['JNTPOS'] = jtg_name..".JointPositionVelocityCommand"
  in_portmap['JNTPOS'] = jtg_name..".JointPosition"
  -- we want one outter input to connect to two inner inputs, so we need a second jntpos
  in_portmap['JNTPOS2'] = jta_name..".JointPosition"

  -- store the mapping in the properties
  storeMapping("in_portmap",in_portmap)
  storeMapping("out_portmap",out_portmap)

  print(namespace..controller_name.." configured")
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
  -- assign entries
  for k,v in pairs(mymap) do 
    portmap.type[count]=k
    portmap.portname[count]=v
    count=count+1
  end
  
end
 
function startHook()
  -- stop all peers except Deployer
  local d=tc:getPeer("Deployer")
  local peers=tc:getPeers()
  for _,peername in pairs(peers) do
    if peername~="Deployer" then
      p=d:getPeer(peername)
      if p then
        p:start()
      end
    end
  end
  --self.running=true
  return true
end


function stopHook()
  -- stop all peers except Deployer
  local d=tc:getPeer("Deployer")
  local peers=tc:getPeers()
  for _,peername in pairs(peers) do
    if peername~="Deployer" then
      p=d:getPeer(peername)
      if p then
        p:stop()
      else --already removed
        print (tc:getName().." removing already unloaded "..peername)
        tc:removePeer(peername)
      end
    end
  end
  --self.running=false
  return true
end

 
-- Ports and properties are the only elements which are not
-- automatically cleaned up. This means this must be done manually for
-- long living components:
function cleanupHook()
  print ("Cleaning up "..tc:getName())
  -- unload all peers except Deployer
  local d=tc:getPeer("Deployer")
  local peers=tc:getPeers()
    
  for _,peername in pairs(peers) do
    if peername~="Deployer" then
      p=d:getPeer(peername)
      if p then
        print (tc:getName().." cleaning up "..peername)
        p:cleanup()
      else --already removed
        print (tc:getName().." removing already unloaded "..peername)
        tc:removePeer(peername)
      end
    end
  end

  -- should not unload the component here, otherwise kickout function crash
  -- due to not refreshing the peer list after cleaning up

  rttlib.tc_cleanup()
  print ("Cleaned up "..tc:getName())
   
end

