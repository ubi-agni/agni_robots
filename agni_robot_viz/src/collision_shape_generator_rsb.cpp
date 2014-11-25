/**
 * @file   collision_shape_generator_rsb.cpp
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Nov 2014
 *
 * @brief  RSB wrapper to generate shape primitives for collision objects in the robot description
 *
 */
 
 
#include <ros/ros.h>
#include <rsb/Factory.h>

#include <rst/geometry/Primitive3DFloatSet.pb.h>
#include <rst/geometry/Primitive3DFloat.pb.h>
#include <string>
#include <rsb/Informer.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

#include <agni_robot_viz/collision_shape_generator.hpp>
#include <boost/shared_ptr.hpp>

using namespace std;
using namespace rsb;

Informer<rst::geometry::Primitive3DFloatSet>::Ptr informer;
    
CollisionShapeGenerator *csg;
ros::Publisher robot_state_publisher;

// callback function receiving joint_states and generating collision primitives
void callback(const sensor_msgs::JointStatePtr &msg)
{
  ROS_DEBUG("received a joint_state ");
  visualization_msgs::MarkerArray markerarray;
  Informer<rst::geometry::Primitive3DFloatSet>::DataPtr prim_set(new rst::geometry::Primitive3DFloatSet());
  
  uint64_t usec = msg->header.stamp.sec * 1000000 + msg->header.stamp.nsec / 1000;

  csg->updatePosition(msg);
  csg->getCollisionShape(markerarray);
  
  // convert the marker array to a protobuf object
  for (unsigned int i=0; i<markerarray.markers.size();i++) {
    rst::geometry::Primitive3DFloat *s = prim_set->add_primitives();
    s->mutable_timestamp()->set_time(usec);
    bool unknown_type=false;
    switch(markerarray.markers[0].type){
      
      case visualization_msgs::Marker::CYLINDER :
        s->set_type(rst::geometry::Primitive3DFloat_PrimitiveType_CYLINDER);
        break;
      
      case visualization_msgs::Marker::SPHERE :
        s->set_type(rst::geometry::Primitive3DFloat_PrimitiveType_SPHERE);
        break;
      
      case visualization_msgs::Marker::CUBE :
        s->set_type(rst::geometry::Primitive3DFloat_PrimitiveType_CUBE);
        break;
      default:
      // only process known types
        unknown_type=true;
        break;
    }
    if(!unknown_type)
    {
      s->mutable_pose()->mutable_translation()->set_x(markerarray.markers[i].pose.position.x);
      s->mutable_pose()->mutable_translation()->set_y(markerarray.markers[i].pose.position.y);
      s->mutable_pose()->mutable_translation()->set_z(markerarray.markers[i].pose.position.z);
      s->mutable_pose()->mutable_rotation()->set_qx(markerarray.markers[i].pose.orientation.x);
      s->mutable_pose()->mutable_rotation()->set_qy(markerarray.markers[i].pose.orientation.y);
      s->mutable_pose()->mutable_rotation()->set_qz(markerarray.markers[i].pose.orientation.z);
      s->mutable_pose()->mutable_rotation()->set_qw(markerarray.markers[i].pose.orientation.w);
      s->mutable_scale()->set_x(markerarray.markers[i].scale.x);
      s->mutable_scale()->set_y(markerarray.markers[i].scale.y);
      s->mutable_scale()->set_z(markerarray.markers[i].scale.z);
      s->set_description(markerarray.markers[i].text);
    }
  }
  informer->publish(prim_set);

  robot_state_publisher.publish( markerarray );
}

int loadParams(ros::NodeHandle &nh, std::string &move_group, std::vector<std::string> &link_names)
{
  nh.param("collision_shape_generator/filter_primitive/move_group", move_group, std::string("upper_body"));
  ROS_INFO("using move_group %s",move_group.c_str());
  link_names.clear();
  using namespace XmlRpc;
  XmlRpc::XmlRpcValue param_xmlrpc;
  if(nh.getParam("collision_shape_generator/filter_primitive/links",param_xmlrpc))
  {
    if(param_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      unsigned int name_size = param_xmlrpc.size();
      for (unsigned int i = 0; i < name_size; ++i)
      {
        if(param_xmlrpc[i].getType() == XmlRpc::XmlRpcValue::TypeString)
        {
          link_names.push_back( static_cast<std::string>(param_xmlrpc[i]) );
        }
        else
        {
          ROS_ERROR("links value %d is not a string, not loading",i+1);
          return -1;
        }
      }
      return 0;
    }
    else 
    {
      if (param_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeString)
      {
        std::string single_link_name = static_cast<std::string>(param_xmlrpc) ;
        link_names.push_back( single_link_name );
        return 0;
      }
      else
      {
        ROS_ERROR("links must be either a string or an array of strings");
        return -1;
      }
    }
  }
  else
  {
    ROS_WARN("missing parameter links");
    return -1;
  }
}

int main(int argc, char** argv)
{
  // converter cannot be in global !!!
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::Primitive3DFloatSet> >
        converter(new rsb::converter::ProtocolBufferConverter<rst::geometry::Primitive3DFloatSet>());

  rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  Factory& factory = getFactory();
  
  informer = factory.createInformer<rst::geometry::Primitive3DFloatSet> ("/nirobots/primitivesetScope");
  
  ros::init(argc, argv, "collision_shape_generator");
  ros::NodeHandle nh, nh_tilde("~");
  std::string move_group;
  std::vector<std::string> link_names;
  if(loadParams(nh,move_group,link_names)==0)
  {
    robot_state_publisher = nh.advertise<visualization_msgs::MarkerArray>( "robot_collision_shape", 1 );
    csg = new CollisionShapeGenerator();
    csg->init(move_group,link_names);
    
    ros::Subscriber sub = nh.subscribe("joint_states",1,callback);

    ros::spin();

    delete csg;
    return 0;
  }
  else
  { 
    return -1;
  }
}
