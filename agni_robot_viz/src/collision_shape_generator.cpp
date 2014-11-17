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
            std::cerr<<"missing scale"<<std::endl;
            continue;
          }
          // make the marker a little larger to get a margin
          mark.scale.x*=1.0;
          mark.scale.y*=1.0;
          mark.scale.z*=1.0;
          tf::poseEigenToMsg(kinematic_state->getCollisionBodyTransform(lm, j), mark.pose);
        }
        //if(mark.type == visualization_msgs::Marker::MESH_RESOURCE && mark.type != visualization_msgs::Marker::TRIANGLE_LIST)
        arr.markers.push_back(mark);
      }
    }
    //robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);
    //return msg;
  }
}
