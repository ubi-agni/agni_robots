#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <agni_rtt_services/ControlIOMap.h>
#include <ocl/DeploymentComponent.hpp>

using namespace RTT;
using namespace std;

class ControllerService : public RTT::Service 
{
public:
  ControllerService(TaskContext* owner);
  std::string getOwnerName();
  std::vector<std::string> getInTypes();
  std::vector<std::string> getOutTypes();
  std::vector<std::string> connectIn(std::string type, std::string peer_portname);
  
private:
  std::string getInternalPortName(std::string dir, std::string type);
};


ControllerService::ControllerService(TaskContext* owner) :
  Service("controllerService", owner)
{
  this->addOperation("getName", &ControllerService::getOwnerName, this).doc("Returns the name of the owner of this object.");
  this->addOperation("getInTypes", &ControllerService::getInTypes, this).doc("Returns the inport type list.");
  this->addOperation("getOutTypes", &ControllerService::getOutTypes, this).doc("Returns the outport type list.");
  this->addOperation("connectIn", &ControllerService::connectIn, this).doc("Connect inputs.");
}

std::string ControllerService::getOwnerName() {
  return getOwner()->getName();
}

std::vector<std::string> ControllerService::getInTypes() {
  
  agni_rtt_services::ControlIOMap *ciomap;
  base::PropertyBase* pb = getOwner()->getProperty( "in_portmap" );
  ciomap = static_cast<agni_rtt_services::ControlIOMap*>(pb->getDataSource()->getRawPointer());
  return ciomap->type;
}


std::vector<std::string> ControllerService::connectIn(std::string type, std::string peer_portname) {
  bool prev_running=getOwner()->isActive();
  if(prev_running)
    getOwner()->stop();
  
  // get the deployer
  OCL::DeploymentComponent *deployer = dynamic_cast<OCL::DeploymentComponent*>(getOwner()->getPeer("Deployer"));
  
  
  // get name and port of peer;
  size_t pos=peer_portname.find_first_of('.');
  std::string otherpeername=peer_portname.substr(0,pos);
  std::string otherportname=peer_portname.substr(pos+1,std::string::npos);
  //RTT::log(RTT::Error) << peer_portname <<"peer "<< otherpeername <<" port " <<  otherportname <<RTT::endlog();
  
  // Create an unbuffered 'shared data' connection:
  ConnPolicy policy = RTT::ConnPolicy::data();
  
  // get the internal name and port
  std::string internal_name = getInternalPortName("in",type);
  if (internal_name!="")
  {
    pos=internal_name.find_first_of('.');
    std::string onepeername=internal_name.substr(0,pos);
    std::string oneportname=internal_name.substr(pos+1,std::string::npos);
    //RTT::log(RTT::Error) << internal_name <<"ipeer "<< onepeername <<" iport " <<  oneportname <<RTT::endlog();
    //deployer->connectPorts(onepeername,oneportname,otherpeername,otherportname);
  }
  else
    RTT::log(RTT::Error) << "No port of type "<< type <<" found" << RTT::endlog();

  if(prev_running)
    getOwner()->start();

}

std::string ControllerService::getInternalPortName(std::string dir, std::string type) {
  agni_rtt_services::ControlIOMap *ciomap;
  base::PropertyBase* pb = getOwner()->getProperty( dir+"_portmap" );
  ciomap = static_cast<agni_rtt_services::ControlIOMap*>(pb->getDataSource()->getRawPointer());
  
  for (size_t i=0 ; i<ciomap->type.size() ; ++i)
  {
    if( ciomap->type[i]==type)
      return ciomap->portname[i];
  }
  return "";
}

std::vector<std::string> ControllerService::getOutTypes() {
  
  agni_rtt_services::ControlIOMap *ciomap;
  base::PropertyBase* pb = getOwner()->getProperty( "out_portmap" );
  ciomap = static_cast<agni_rtt_services::ControlIOMap*>(pb->getDataSource()->getRawPointer());
  return ciomap->type;
}

ORO_SERVICE_NAMED_PLUGIN(ControllerService, "controllerService")

