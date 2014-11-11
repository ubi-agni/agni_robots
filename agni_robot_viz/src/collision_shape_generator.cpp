/**
 * @file   collision_shape_generator.cpp
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Nov 2014
 *
 * @brief  generate shape primitives for collision objects in the robot description
 *
 */

#include <agni_robot_viz/collision_shape_generator.hpp>

CollisionShapeGenerator::CollisionShapeGenerator() :robot_model_loader("robot_description")
{
  initialized_ = false;
}

CollisionShapeGenerator::~CollisionShapeGenerator() 
{

}

void CollisionShapeGenerator::init(const std::string &move_group, const std::vector<std::string> &link_names ) 
{
  move_group_ = move_group;
  /* Get a shared pointer to the model */
  robot_model = robot_model_loader.getModel();
  /* Create a kinematic state - this represents the configuration for the robot represented by kinematic_model */
  kinematic_state.reset(new robot_state::RobotState(robot_model));
  /* Get the configuration for the correct joint group */
    
  joint_model_group = robot_model->getJointModelGroup(move_group_);
  
  if(link_names[0]=="all")
  {
    //get all joints
    link_names_.clear();
    link_names_=joint_model_group->getLinkModelNames();
  }
  else
  {
    link_names_= link_names;
  }
  initialized_=true;
}

void CollisionShapeGenerator::updatePosition(const sensor_msgs::JointStatePtr &msg)
{
  if (initialized_)
  {
    kinematic_state->setVariablePositions(msg->name,msg->position);
  }
}

void CollisionShapeGenerator::getCollisionShape(visualization_msgs::MarkerArray &arr)
{
  if (initialized_)
  {
    ros::Time tm = ros::Time::now();
    //kinematic_state->setToRandomPositions(joint_model_group);
    /* get a robot state message describing the pose in kinematic_state */
    //std::vector<std::string> link_names;
    //link_names.push_back("r_forearm");
    std_msgs::ColorRGBA color;
    color.r=1.0;
    color.a=1.0;

    
    //kinematic_state->getRobotMarkers(arr,link_names,color,"robot",ros::Duration(0),false);
    for (std::size_t i = 0; i < link_names_.size(); ++i)
    {
      const moveit::core::LinkModel* lm = robot_model->getLinkModel(link_names_[i]);
      if (!lm)
        continue;
      if (lm->getShapes().empty())
        continue;
      for (std::size_t j = 0 ; j < lm->getShapes().size() ; ++j)
      {
        visualization_msgs::Marker mark;
        mark.header.frame_id = robot_model->getModelFrame();
        mark.header.stamp = tm;
        mark.ns="robot";
        mark.id=i*100+j;
        mark.color = color;
        mark.text = link_names_[i];
        // we prefer using the collision shape
        if (lm->getShapes().size() > 0)
        {
          if (!shapes::constructMarkerFromShape(lm->getShapes()[j].get(), mark))
            continue;
          // if the object is invisible (0 volume) we skip it
          if (fabs(mark.scale.x * mark.scale.y * mark.scale.z) < std::numeric_limits<float>::epsilon())
          {
            ROS_INFO("missing scale");
            continue;
          }
          // make the marker a little larger to get a margin
          mark.scale.x*=1.5;
          mark.scale.y*=1.5;
          mark.scale.z*=1.5;
          tf::poseEigenToMsg(kinematic_state->getCollisionBodyTransform(lm, j), mark.pose);
        }
        arr.markers.push_back(mark);
      }
    }
    //robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);
    //return msg;
  }
}

CollisionShapeGenerator *csg;
ros::Publisher robot_state_publisher;

void callback(const sensor_msgs::JointStatePtr &msg)
{
    visualization_msgs::MarkerArray markerarray;
    csg->updatePosition(msg);
    csg->getCollisionShape(markerarray);
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
