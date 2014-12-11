/**
 * @file   collision_shape_generator_ros.cpp
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Nov 2014
 *
 * @brief  Ros wrapper to generate shape primitives for collision objects in the robot description
 *
 */

#include <ros/ros.h>
#include <agni_robot_viz/collision_shape_generator.hpp>

CollisionShapeGenerator *csg;
ros::Publisher robot_state_publisher;

// callback function receiving joint_states and generating collision primitives
void callback(const sensor_msgs::JointStatePtr &msg)
{
    visualization_msgs::MarkerArray markerarray;
    // update the kinematics to current robot_state
    csg->updatePosition(msg);
    // get collision shape markers
    csg->getCollisionShape(markerarray);
    // publish to ROS
    robot_state_publisher.publish( markerarray );
}

int loadParams(ros::NodeHandle &nh, std::string &move_group, std::vector<std::string> &link_names)
{
  nh.param("collision_shape_generator/filter_primitive/move_group", move_group, std::string("upper_body"));
  ROS_INFO("using move_group %s",move_group.c_str());
  
  link_names.clear();
  
  //parse the config file
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
  // init ROS
  ros::init(argc, argv, "collision_shape_generator");
  ros::NodeHandle nh, nh_tilde("~");
  std::string move_group;
  std::vector<std::string> link_names;
  if(loadParams(nh,move_group,link_names)==0)
  {
    //  ros::Publisher robot_state_publisher = nh.advertise<moveit_msgs::DisplayRobotState>( "robot_collision_shape", 1 );
    robot_state_publisher = nh.advertise<visualization_msgs::MarkerArray>( "robot_collision_shape", 1 );
    csg = new CollisionShapeGenerator();
    csg->init(move_group,link_names);
    
    ros::Subscriber sub = nh.subscribe("joint_states",1,callback);
     /* loop at 10 Hz */
    //ros::Rate loop_rate(10);
    ros::spin();
    /*while (ros::ok())
    {
      
      //moveit_msgs::DisplayRobotState msg=csg.getCollisionShape(shape);
      visualization_msgs::MarkerArray markerarray;
      csg.getCollisionShape(markerarray);
      
      robot_state_publisher.publish( markerarray );
      markerarray.markers.clear();
      ros::spinOnce();
      loop_rate.sleep();
    }*/
    delete csg;
    return 0;
  }
  else
  { 
    return -1;
  }
}
