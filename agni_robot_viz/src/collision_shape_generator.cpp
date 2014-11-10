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
  
  init();
}

CollisionShapeGenerator::~CollisionShapeGenerator() 
{

}

void CollisionShapeGenerator::init() 
{
  /* Get a shared pointer to the model */
  robot_model = robot_model_loader.getModel();
  /* Create a kinematic state - this represents the configuration for the robot represented by kinematic_model */
  kinematic_state.reset(new robot_state::RobotState(robot_model));
  /* Get the configuration for the joints in the right arm of the PR2*/
  joint_model_group = robot_model->getJointModelGroup("upper_body");

}

void CollisionShapeGenerator::updatePosition(const sensor_msgs::JointStatePtr &msg)
{
  kinematic_state->setVariablePositions(msg->name,msg->position);
}

void CollisionShapeGenerator::getCollisionShape(visualization_msgs::MarkerArray &arr)
{
  ros::Time tm = ros::Time::now();
  //kinematic_state->setToRandomPositions(joint_model_group);
  /* get a robot state message describing the pose in kinematic_state */
  std::vector<std::string> link_names;
  link_names.push_back("r_forearm");
  std_msgs::ColorRGBA color;
  color.r=1.0;
  color.a=1.0;

  
  //kinematic_state->getRobotMarkers(arr,link_names,color,"robot",ros::Duration(0),false);
  for (std::size_t i = 0; i < link_names.size(); ++i)
  {
    const moveit::core::LinkModel* lm = robot_model->getLinkModel(link_names[i]);
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
      mark.color = color;
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
        tf::poseEigenToMsg(kinematic_state->getCollisionBodyTransform(lm, j), mark.pose);
      }
      arr.markers.push_back(mark);
    }
  }
  //robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);
  //return msg;
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_shape_generator");
  ros::NodeHandle nh;
//  ros::Publisher robot_state_publisher = nh.advertise<moveit_msgs::DisplayRobotState>( "robot_collision_shape", 1 );
  robot_state_publisher = nh.advertise<visualization_msgs::MarkerArray>( "robot_collision_shape", 1 );
  csg = new CollisionShapeGenerator();
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
