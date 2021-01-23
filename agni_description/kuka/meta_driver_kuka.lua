require("rttlib")
require "rttros"
tc=rtt.getTC()
fri = 0
diag = 0
jsp = 0
filter = 0

metacontroller_path = rttros.find_rospack("agni_rtt_services")
package.path = metacontroller_path..'/scripts'..'/?.lua;' .. package.path 
require("meta-component_common")

iface_spec = {
   ports={},
   properties={
      { name='namespace', datatype='string', desc="namespace as prefix" },
      { name='port', datatype='int', desc="port on which FRI listens (default 49938)" },
      { name='fake', datatype='bool', desc="if true, FRI loops cmds to msr" },
      { name='filter_cutoff_freq', datatype='float', desc="filter cutoff frequency (default 10.0)" },
      { name='timestep', datatype='float', desc="timestep of FRI (default 0.001)" },
      { name='in_portmap', datatype='agni_rtt_services/ControlIOMap', desc="input port mapping" },
      { name='out_portmap', datatype='agni_rtt_services/ControlIOMap', desc="output port mapping" },
      { name='resources', datatype='strings', desc="joints controlled by this controller" },
      { name='controller_name', datatype='string', desc="controller name" },
      { name='controller_type', datatype='string', desc="controller type ex: position_controllers/JointTrajectoryController" },
   }
}

-- this create the interface (must be global)
iface=rttlib.create_if(iface_spec)

-- defines the configureHook of the wrapper
function configureHook()
  -- find the deployer
  local d=tc:getPeer("Deployer")
  tcName=tc:getName()

  -- import the required libraries and typekits here
  d:import("lwr_fri")
  d:import("oro_joint_state_publisher")
  d:import("flwr_filter")
  d:import("agni_rtt_services")
  d:import("rtt_sensor_msgs")
  d:import("rtt_geometry_msgs")
  d:import("rtt_diagnostic_msgs")
  d:import("rtt_trajectory_msgs")

  -- retrieve the properties from the interface parameters
  namespace = iface.props.namespace:get()
  port = iface.props.port:get()
  if port == 0 then
    port = 49938
  end
  timestep = iface.props.timestep:get()
  if timestep == 0 then
    timestep = 0.001
  end
  cutoff = iface.props.filter_cutoff_freq:get()
  if cutoff == 0 then
    cutoff = 10.0
  end
  fake = iface.props.fake:get()
  if fake then
    print("FRI will be started in fake mode")
  else
    print("FRI will be started in normal mode")
  end
 
  -- advertize the type of controller you set (useful for controller_manager)
  controller_type = tc:getProperty("controller_type")
  -- only position is advertized here as the filter is sending positions
  controller_type:set("position_controllers/JointPositionController")
  resources = tc:getProperty("resources")

  local in_portmap={}
  local out_portmap={}

  -- diag timestep is 10 times slower than timestep
  diagtimestep = timestep * 10.0
  diagname = namespace.."LWRDiag"
  d:loadComponent(diagname, "FRIDiagnostics")
  d:setActivity(diagname, diagtimestep, 2, rtt.globals.ORO_SCHED_RT)
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
  fri:getProperty("fake_fri"):set(fake)
  fri:getProperty("timestep"):set(timestep) -- used for fake AND for other component to read from it
  if fri:configure() then

    -- add fri to the parent component peers
    d:addPeer(tcName, friname)  

    -- register ports for the compound controller
    -- fri should not advertize this port to the exterior, it must be through the filter for protection
    --register_port(in_portmap, 'CMDJNTPOS', friname..".JointPositionCommand")
    register_port(out_portmap, 'CURJNTPOS', friname..".JointPosition")
    register_port(out_portmap, 'CURCARTPOSE', friname..".CartesianPosition")
    register_port(out_portmap, 'CURCARTWRE', friname..".CartesianWrench")
    register_port(out_portmap, 'CURJNT', friname..".CurrentJoint")
    register_port(in_portmap, 'CMDJNT', friname..".JointCommand")

    -- deploy joint state publisher
    jspname = namespace.."JntPub"
    d:loadComponent(jspname, "JointStatePublisher")
    d:setActivity(jspname, 0.01, 10, rtt.globals.ORO_SCHED_RT)
    jsp = d:getPeer(jspname)
    joint_names = jsp:getProperty("joint_names")
    joint_names:get():resize(7)

    -- set the resource to match the controlled joints
    resources:get():resize(7)

    for i=0,6,1 do 
      joint_names[i] = namespace.."_arm_"..i.."_joint"
      resources[i] = joint_names[i]
    end 

    if jsp:configure() then
      -- add jsp to the parent component peers
      d:addPeer(tcName, jspname) 

      -- deploy the filter and advertize its input/output
      filtername = namespace.."Filter"
      d:loadComponent(filtername, "FLWRFilter")
      d:setActivity(filtername, 0, 70, rtt.globals.ORO_SCHED_RT)

      -- set velocity and acceleration limits
      filter = d:getPeer(filtername)
      filter:getProperty("CUTOFF_FREQUENCY"):set(cutoff)
      filter:getProperty("TIMESTEP"):set(timestep)
      filter:getProperty("MODE"):set(1)
      vel_limits=filter:getProperty("VELOCITY_LIMITS")
      vel_limits:get():resize(7)
      acc_limits=filter:getProperty("ACCEL_LIMITS")
      acc_limits:get():resize(7)
      -- TODO: Read from file of better request from URDF
      for i=0,6,1 do 
        vel_limits[i] = 0.8
       acc_limits[i] = 4.0
      end

      filter:configure()

      -- register ports on the compound controller
      -- AvoidFilterCompletely
      --register_port(in_portmap, 'CMDJNT', filtername..".DesiredJoint")
      register_port(out_portmap, 'LOG', filtername..".Log")

      -- add filter to the parent component peers
      d:addPeer(tcName, filtername)


      -- deploy the converter and advertize its output
      --convertname = namespace.."Convert"
      --d:loadComponent(convertname, "FLWRConvert")
      --d:setActivity(convertname, 0, 70, rtt.globals.ORO_SCHED_RT)

      -- set velocity and acceleration limits
      --convert = d:getPeer(convertname)
      --convert:configure()

      -- register ports on the compound controller
      --register_port(out_portmap, 'CURJNT', convertname..".CurrentJoint")

      -- add converter to the parent component peers
      --d:addPeer(tcName, convertname) 

      -- store the mapping in the properties
      storeMapping("in_portmap",in_portmap)
      storeMapping("out_portmap",out_portmap)

      -- internal connection
      d:connect(friname..".RobotState", diagname..".RobotState", rtt.Variable("ConnPolicy"))
      d:connect(friname..".FRIState", diagname..".FRIState", rtt.Variable("ConnPolicy"))
      d:connect(friname..".JointPosition", jspname..".JointPosition", rtt.Variable("ConnPolicy"))
      d:connect(friname..".JointVelocity", jspname..".JointVelocity", rtt.Variable("ConnPolicy"))
      --d:connect(friname..".JointPosition", convertname..".FRIJointPos", rtt.Variable("ConnPolicy"))
      --d:connect(friname..".JointVelocity", convertname..".FRIJointVel", rtt.Variable("ConnPolicy"))
      d:connect(friname..".JointTorque", jspname..".JointEffort", rtt.Variable("ConnPolicy"))
      -- AvoidFilterCompletely
      --d:connect(friname..".RobotState", filtername..".RobotState", rtt.Variable("ConnPolicy"))
      --d:connect(friname..".FRIState", filtername..".FRIState", rtt.Variable("ConnPolicy"))
      --d:connect(friname..".JointPosition", filtername..".FRIJointPos", rtt.Variable("ConnPolicy"))
      -- Guillaume:No idea why we did not connect vel to the filter, maybe because velocity data in sim was bad ?
      ---- d:connect(friname..".JointVelocity", convertname..".JointVelocity", rtt.Variable("ConnPolicy"))
      --d:connect(filtername..".FilteredJointPos", friname..".JointPositionCommand", rtt.Variable("ConnPolicy"))
      --d:connect(filtername..".JointImpedance", friname..".JointImpedanceCommand", rtt.Variable("ConnPolicy"))

      -- ROS in out
      local ros=rtt.provides("ros")
      d:stream(diagname..".Diagnostics",ros:topic(namespace.."/diagnostics"))
      d:stream(jspname..".joint_state",ros:topic(namespace.."/joint_states"))
      d:stream(friname..".CartesianPosition",ros:topic(namespace.."/cartesian_position"))
      d:stream(friname..".CartesianWrench",ros:topic(namespace.."/cartesian_wrench"))
      d:stream(friname..".CartesianPositionStamped",ros:topic(namespace.."/cartesian_position_stamped"))
      d:stream(friname..".fromKRL",ros:topic(namespace.."/fromKRL"))
      d:stream(friname..".toKRL",ros:topic(namespace.."/toKRL"))
      --d:stream(convertname..".CurrentJoint",ros:topic(namespace.."/current_joint"))
      d:stream(friname..".CurrentJoint",ros:topic(namespace.."/current_joint"))
      -- AvoidFilterCompletely
      --d:stream(filtername..".DesiredImpedance",ros:topic(namespace.."/filter_impedance"))
      --d:stream(filtername..".FilteredJoint",ros:topic(namespace.."/filter_desired_joint"))
      d:stream(friname..".JointCommand",ros:topic(namespace.."/joint_command"))

      print(namespace.."kuka_controller configured")
      return true
    else
      print(namespace.."JSP could not be configured, unloading it")
      d:unloadComponent(jspname)
      d:unloadComponent(friname)
      return false
    end
  else
    print(namespace.."FRI could not be configured, unloading it")
    d:unloadComponent(friname)
    return false
  end
end

