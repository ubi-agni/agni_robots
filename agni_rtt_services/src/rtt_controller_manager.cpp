// derived from conman_ros/ros_interface_service.cpp
/** Copyright (c) 2013, Jonathan Bohren, all rights reserved.
 Redistribution and use in source and binary forms, with or without 
 modification, are permitted provided that the following conditions are met: 

 * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer. 
 * Redistributions in binary form must reproduce the above copyright 
   notice, this list of conditions and the following disclaimer in the 
   documentation and/or other materials provided with the distribution. 
 * Neither the name of The Johns Hopkins University nor the names of its 
   contributors may be used to endorse or promote products derived from 
   this software without specific prior written permission. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
POSSIBILITY OF SUCH DAMAGE. 
 */
 
// modified by Guillaume Walck (2015) to act as a dummy controller_manager
// that retrieves the info of controller wrapper and their properties 
// if peers are of type controller wrapper

#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/TaskContext.hpp>
#include <controller_manager_msgs/ListControllerTypes.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/ReloadControllerLibraries.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <rtt_roscomm/rosservice.h>


using namespace RTT;
using namespace std;

class RTTControllerManager : public RTT::Service 
{
public:
  RTTControllerManager(TaskContext* owner);
  bool listControllerTypesCB(       controller_manager_msgs::ListControllerTypes::Request &req,       controller_manager_msgs::ListControllerTypes::Response& resp);
  bool listControllersCB(           controller_manager_msgs::ListControllers::Request &req,           controller_manager_msgs::ListControllers::Response& resp);
  bool loadControllerCB(            controller_manager_msgs::LoadController::Request &req,            controller_manager_msgs::LoadController::Response& resp);
  bool reloadControllerLibrariesCB( controller_manager_msgs::ReloadControllerLibraries::Request &req, controller_manager_msgs::ReloadControllerLibraries::Response& resp);
  bool switchControllerCB(          controller_manager_msgs::SwitchController::Request &req,          controller_manager_msgs::SwitchController::Response& resp);
  bool unloadControllerCB(          controller_manager_msgs::UnloadController::Request &req,          controller_manager_msgs::UnloadController::Response& resp);
  
  boost::shared_ptr<RTT::Service> cm;
  boost::shared_ptr<rtt_rosservice::ROSService> rosservice;
};


RTTControllerManager::RTTControllerManager(TaskContext* owner) :
  Service("controllerManagerService", owner)
{
  cm = owner->provides("cm");
  cm->addOperation("listControllerTypes", &RTTControllerManager::listControllerTypesCB, this);
  cm->addOperation("listControllers", &RTTControllerManager::listControllersCB, this).doc("Returns the controllers list.");
  cm->addOperation("loadController", &RTTControllerManager::loadControllerCB, this);
  cm->addOperation("reloadControllerLibraries", &RTTControllerManager::reloadControllerLibrariesCB, this);
  cm->addOperation("switchController", &RTTControllerManager::switchControllerCB, this);
  cm->addOperation("unloadController", &RTTControllerManager::unloadControllerCB, this);
  
  base::PropertyBase* pb = owner->getProperty("namespace");
  std::string ns="";
  if(pb)
  {
    ns = "/"+pb->getDataSource()->toString();
  }
  
  // Load the rosservice service
  RTT::log(RTT::Debug) << "Getting rtt_roscomm service service..." << RTT::endlog();
  rosservice = owner->getProvider<rtt_rosservice::ROSService>("rosservice");
  RTT::log(RTT::Debug) << "Connecting ros_control service servers..." << RTT::endlog();
  rosservice->connect("cm.listControllerTypes",
                     ns+"/controller_manager/list_controller_types",
                     "controller_manager_msgs/ListControllerTypes");

  rosservice->connect("cm.listControllers",
                     ns+"/controller_manager/list_controllers",
                     "controller_manager_msgs/ListControllers");

  rosservice->connect("cm.loadController",
                     ns+"/controller_manager/load_controller",
                     "controller_manager_msgs/LoadController");

  rosservice->connect("cm.reloadControllerLibraries",
                     ns+"/controller_manager/reload_controller_libraries",
                     "controller_manager_msgs/ReloadControllerLibraries");

  rosservice->connect("cm.switchController",
                     ns+"/controller_manager/switch_controller",
                     "controller_manager_msgs/SwitchController");

  rosservice->connect("cm.unloadController",
                     ns+"/controller_manager/unload_controller",
                     "controller_manager_msgs/UnloadController");
  
}


bool RTTControllerManager::listControllerTypesCB(
    controller_manager_msgs::ListControllerTypes::Request &req,
    controller_manager_msgs::ListControllerTypes::Response& resp)
{
  return false;
}

bool RTTControllerManager::loadControllerCB(
    controller_manager_msgs::LoadController::Request &req,
    controller_manager_msgs::LoadController::Response& resp)
{
  return false;
}
bool RTTControllerManager::reloadControllerLibrariesCB(
    controller_manager_msgs::ReloadControllerLibraries::Request &req,
    controller_manager_msgs::ReloadControllerLibraries::Response& resp)
{
  return false;
}

bool RTTControllerManager::switchControllerCB(
    controller_manager_msgs::SwitchController::Request &req,
    controller_manager_msgs::SwitchController::Response& resp)
{
  return false;
}
bool RTTControllerManager::unloadControllerCB(
    controller_manager_msgs::UnloadController::Request &req,
    controller_manager_msgs::UnloadController::Response& resp)
{
  return false;
}

// Retrieve the list of controllers among the peer list
bool RTTControllerManager::listControllersCB(
    controller_manager_msgs::ListControllers::Request &req,
    controller_manager_msgs::ListControllers::Response &resp)
{
  controller_manager_msgs::ControllerState controller;

  TaskContext::PeerList peer_list = getOwner()->getPeerList();
  if(peer_list.size() > 0)
  {
    
    resp.controller.reserve(peer_list.size());
    for(size_t i = 0; i < peer_list.size(); ++i)
    {
      TaskContext* peer_tc = getOwner()->getPeer(peer_list[i]);
      if(peer_tc)
      {
        base::PropertyBase* pb = peer_tc->getProperty("controller_name");
        
        if(pb)
        {
          controller.name = pb->getDataSource()->toString();
        }
        else
        {
          RTT::Logger::log(RTT::Logger::Debug) << "missing controller_name property for peer " << peer_tc->getName() <<  RTT::endlog();
          continue;
        }

        pb = peer_tc->getProperty("controller_type");
        if(pb)
        {
          controller.type = pb->getDataSource()->toString();
        }
        else
        {
          RTT::Logger::log(RTT::Logger::Debug) << "missing controller_type property for peer " << peer_tc->getName() <<  RTT::endlog();
          continue;
        }

        controller.state = peer_tc->isRunning() ? "running" : "stopped";

        pb = peer_tc->getProperty("resources");
        std::vector<std::string>* resources_ptr = NULL;
        if(pb)
        {
          resources_ptr = static_cast< std::vector<std::string>* >(pb->getDataSource()->getRawPointer());
        }
        else
        {
          RTT::Logger::log(RTT::Logger::Debug) << "missing resources property for peer " << peer_tc->getName() <<  RTT::endlog();
          continue;
        }

        if (resources_ptr)
        {
          controller.resources.clear();          
          for (size_t j = 0; j < resources_ptr->size(); ++j)
          {
            controller.resources.push_back(resources_ptr->at(j));
          }
        }
        resp.controller.push_back(controller);
      }
    }
  }
  return true;
}


ORO_SERVICE_NAMED_PLUGIN(RTTControllerManager, "controllerManagerService")

