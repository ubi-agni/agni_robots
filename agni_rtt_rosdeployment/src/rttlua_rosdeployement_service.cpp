#include <rtt/RTT.hpp>
#include <rtt/plugin/Plugin.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/internal/GlobalService.hpp>
#include <rtt/OperationCaller.hpp>

#include <rtt_ros_msgs/RunScript.h>

#include <ocl/DeploymentComponent.hpp>

#include <ros/ros.h>

using namespace RTT;
using namespace std;

class ROSLuaDeploymentService : public RTT::Service 
{
public:
  ROSLuaDeploymentService(OCL::DeploymentComponent* deployer);

private:
  OCL::DeploymentComponent *deployer_;

  ros::NodeHandle nh_;

  ros::ServiceServer run_lua_script_service_;

  bool run_lua_script_cb(
      rtt_ros_msgs::RunScript::Request& request,
      rtt_ros_msgs::RunScript::Response& response);

};


ROSLuaDeploymentService::ROSLuaDeploymentService(OCL::DeploymentComponent* deployer) :
  Service("rosluadeployment", static_cast<RTT::TaskContext*>(deployer)),
  deployer_(deployer),
  nh_("~"+deployer->getName())
{
  if(deployer_) {
    // Create services
    run_lua_script_service_ = nh_.advertiseService("run_lua_script",&ROSLuaDeploymentService::run_lua_script_cb,this);
  } else {
    RTT::log(RTT::Error) << "Attempted to load the rosluadeployment service on a TaskContext which is not an OCL::DeploymentComponent. No ROS services will be advertised." << RTT::endlog();
  }
}

bool ROSLuaDeploymentService::run_lua_script_cb(
    rtt_ros_msgs::RunScript::Request& request,
    rtt_ros_msgs::RunScript::Response& response)
{
	deployer_->loadService("Deployer","Lua");
  OperationCaller<bool(std::string)> ex_file = deployer_->provides("Lua")->getOperation("exec_file");
  
	response.success =  ex_file(request.file_path);
  return true;
}


bool loadROSLuaDeploymentService(RTT::TaskContext *tc) {
  OCL::DeploymentComponent *deployer = dynamic_cast<OCL::DeploymentComponent*>(tc);

  if(!deployer) {
    RTT::log(RTT::Error) << "The rosluadeployment service must be loaded on a valid OCL::DeploymentComponent" <<RTT::endlog();
    return false; 
  }

  deployer->import("rtt_rosnode");

  if(!ros::isInitialized()) {
    RTT::log(RTT::Error) << "The agni_rtt_rosdeployment plugin cannot be used without the rtt_rosnode plugin. Please load rtt_rosnode." << RTT::endlog();

    return false;
  }

  RTT::Service::shared_ptr sp( new ROSLuaDeploymentService( deployer ) ); 
  return tc->provides()->addService( sp ); 
}

extern "C" {
  RTT_EXPORT bool loadRTTPlugin(RTT::TaskContext* tc);  
  bool loadRTTPlugin(RTT::TaskContext* tc) {    
    if(tc == 0) return true;
    return loadROSLuaDeploymentService(tc);
  } 
  RTT_EXPORT RTT::Service::shared_ptr createService();  
  RTT::Service::shared_ptr createService() {    
    RTT::Service::shared_ptr sp; 
    return sp; 
  } 
  RTT_EXPORT std::string getRTTPluginName(); 
  std::string getRTTPluginName() { 
    return "rosluadeployment"; 
  } 
  RTT_EXPORT std::string getRTTTargetName(); 
  std::string getRTTTargetName() { 
    return OROCOS_TARGET_NAME; 
  } 
}

