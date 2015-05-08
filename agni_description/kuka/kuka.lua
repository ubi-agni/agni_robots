
-- Kuka Controller set of components
-- provides generic connecting system with generic controller in/out names
-- that are internally mapped to the set of components providing these inputs

function KukaControllers(namespace,port)
   local self = {             -- our new object
    running=false
   }
   
  local namespace = namespace      -- initialize our object
  local port = port
  local in_portmap = {}
  local out_portmap = {}
  local fri = 0
  local diag = 0
  local jsp = 0
  local filter = 0
         
  function self.deploy(d)
    -- deploy diagnostics
    local diagname = namespace.."LWRDiag"
    d:loadComponent(diagname, "FRIDiagnostics")
    d:setActivity(diagname, 0.01, 2, rtt.globals.ORO_SCHED_RT)
    diag = d:getPeer(diagname)
    diag:configure()
    
    -- deploy FRI and advertize its input/output
    local friname = namespace.."FRI"
    d:loadComponent(friname, "FRIComponent")
    d:setActivity(friname, 0, 80, rtt.globals.ORO_SCHED_RT)
    fri = d:getPeer(friname)
    fri:getProperty("fri_port"):set(port)
    fri:configure()
    in_portmap['JointPosition'] = friname..".JointPositionCommand"
    out_portmap['JointPosition'] = friname..".JointPosition"

    -- deploy joint state publisher
    local jspname = namespace.."JntPub"
    d:loadComponent(jspname, "JointStatePublisher")
    d:setActivity(jspname, 0.01, 10, rtt.globals.ORO_SCHED_RT)
    jsp = d:getPeer(jspname)
    joint_names=jsp:getProperty("joint_names")
    joint_names:get():resize(7)
    for i=0,6,1 do 
      joint_names[i]=namespace.."_arm_"..i.."_joint"
    end 
    jsp:configure()

    -- deploy the filter and advertize its input/output
    local filtername = namespace.."Filter"
    d:loadComponent(filtername, "FLWRFilter")
    d:setActivity(filtername, 0, 70, rtt.globals.ORO_SCHED_RT)
    filter = d:getPeer(filtername)
    filter:getProperty("FREQUENCY"):set(1.0)
    filter:configure()
    in_portmap['FilteredJointPosition'] = filtername..".DesiredJointPos"
    out_portmap['Log'] = filtername..".Log"

    -- internal connections
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
    
    print("finished "..namespace.."kuka starting")
  end

  function self.start()
    fri:start()
    diag:start()
    jsp:start()
    filter:start()
    self.running=true
  end

  function self.stop()
    diag:stop()
    jsp:stop()
    filter:stop()
    fri:stop()
    self.running=false
  end

  function self.connectIn(d,intype,peerportname)
    local previous_running=self.running
    if self.running then
      stop()
    end
    if in_portmap[intype] then
      d:connect(in_portmap[intype], peerportname, rtt.Variable("ConnPolicy"))
    else
      print ("this type is not supported in this controller")
    end
   
    if self.running ~= previous_running then
      start()
    end
  end

  function self.connectOut(d,outtype,peerportname)
    local previous_running=self.running
    if self.running then
      stop()
    end
    if out_portmap[outtype] then
      d:connect(out_portmap[outtype], peerportname, rtt.Variable("ConnPolicy"))
    else
      print ("this type is not supported in this controller")
    end
   
    if self.running ~= previous_running then
      start()
    end
  end

  function self.init(d)
      print ("Importing kuka required components")
      d:import("rtt_rosnode")
      d:import("rtt_roscomm")
      d:import("rtt_std_msgs")
      d:import("rtt_sensor_msgs")
      d:import("rtt_diagnostic_msgs")
      
      d:import("lwr_fri")
      d:import("oro_joint_state_publisher")
      d:import("flwr_filter")
  end

  return self
   
end
