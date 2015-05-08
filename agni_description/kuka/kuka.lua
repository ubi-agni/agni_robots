KukaControllers = {}
KukaControllers.__index = KukaControllers

--toto = false
--kukacontrollers_imports_done=false

function KukaControllers.create(namespace,port)
   local k = {}             -- our new object
   setmetatable(k,KukaControllers)  -- make KukaControllers handle lookup
   k.namespace = namespace      -- initialize our object
   k.port = port
   k.in_portmap = {}
   k.out_portmap = {}
   return k
end
       
function KukaControllers:deploy(d)
  -- deploy diagnostics
  local diagname = self.namespace.."LWRDiag"
  d:loadComponent(diagname, "FRIDiagnostics")
  d:setActivity(diagname, 0.01, 2, rtt.globals.ORO_SCHED_RT)
  self.diag = d:getPeer(diagname)
  self.diag:configure()
  
  -- deploy FRI and advertize its input/output
  local friname = self.namespace.."FRI"
  d:loadComponent(friname, "FRIComponent")
  d:setActivity(friname, 0, 80, rtt.globals.ORO_SCHED_RT)
  self.fri = d:getPeer(friname)
  self.fri:getProperty("fri_port"):set(self.port)
  self.fri:configure()
  self.in_portmap['JointPosition'] = friname..".JointPositionCommand"
  self.out_portmap['JointPosition'] = friname..".JointPosition"

  -- deploy joint state publisher
  local jspname = self.namespace.."JntPub"
  d:loadComponent(jspname, "JointStatePublisher")
  d:setActivity(jspname, 0.01, 10, rtt.globals.ORO_SCHED_RT)
  self.jsp = d:getPeer(jspname)
  joint_names=self.jsp:getProperty("joint_names")
  joint_names:get():resize(7)
  for i=0,6,1 do 
    joint_names[i]=self.namespace.."_arm_"..i.."_joint"
  end 
  self.jsp:configure()

  -- deploy the filter and advertize its input/output
  local filtername = self.namespace.."Filter"
  d:loadComponent(filtername, "FLWRFilter")
  d:setActivity(filtername, 0, 70, rtt.globals.ORO_SCHED_RT)
  self.filter = d:getPeer(filtername)
  self.filter:getProperty("FREQUENCY"):set(1.0)
  self.filter:configure()
  self.in_portmap['FilteredJointPosition'] = filtername..".DesiredJointPos"
  self.out_portmap['Log'] = filtername..".Log"

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
  ros=rtt.provides("ros")
  d:stream(diagname..".Diagnostics",ros:topic(self.namespace.."/diagnostics"))
  d:stream(jspname..".joint_state",ros:topic(self.namespace.."/joint_states"))
  
  
  print("finished "..self.namespace.."kuka starting")
end

function KukaControllers:start()
  self.fri:start()
  self.diag:start()
  self.jsp:start()
  self.filter:start()
  self.running=true
end

function KukaControllers:stop()
  self.diag:stop()
  self.jsp:stop()
  self.filter:stop()
  self.fri:stop()
  self.running=false
end

function KukaControllers:connectIn(d,intype,peerportname)
  local previous_running=self.running
  if self.running then
    self.stop()
  end
  if self.in_portmap[intype] then
    d:connect(self.in_portmap[intype], peerportname, rtt.Variable("ConnPolicy"))
  else
    print ("this type is not supported in this controller")
  end
 
  if self.running ~= previous_running then
    self.start()
  end
end

function KukaControllers:connectOut(d,outtype,peerportname)
  local previous_running=self.running
  if self.running then
    self.stop()
  end
  if self.out_portmap[outtype] then
    d:connect(self.out_portmap[outtype], peerportname, rtt.Variable("ConnPolicy"))
  else
    print ("this type is not supported in this controller")
  end
 
  if self.running ~= previous_running then
    self.start()
  end
end

function KukaControllers:init(d)
  --if kukacontrollers_imports_done~=true then
     -- ROS integration
    d:import("rtt_rosnode")
    d:import("rtt_roscomm")
    d:import("rtt_std_msgs")
    d:import("rtt_sensor_msgs")
    d:import("rtt_diagnostic_msgs")
    
    d:import("lwr_fri")
    d:import("oro_joint_state_publisher")
    d:import("flwr_filter")
    print ("Importing kuka required components")
    kukacontrollers_imports_done=true
 --else
    --print ("Components already imported")
 --end

end
