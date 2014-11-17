/**
 * @file   collision_shape_generator.hpp
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Nov 2014
 *
 * @brief  generate shape primitives for collision objects in the robot description
 *
 */

#ifndef COLLISION_SHAPE_GENERATOR_H
#define COLLISION_SHAPE_GENERATOR_H



// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <eigen_conversions/eigen_msg.h>


// Robot state publishing
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>
//#include <shape_msgs/SolidPrimitive.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometric_shapes/shape_operations.h>
#include <sensor_msgs/JointState.h>
// PI
#include <boost/math/constants/constants.hpp>

class CollisionShapeGenerator
  {
  public:
    CollisionShapeGenerator();
    ~CollisionShapeGenerator();
    //moveit_msgs::DisplayRobotState getCollisionShape(shape_msgs::SolidPrimitive &shape);
    void getCollisionShape(visualization_msgs::MarkerArray &arr);
    void updatePosition(const sensor_msgs::JointStatePtr &msg);
    void init(const std::string &move_group, const std::vector<std::string> &link_names );

  protected:

    robot_model_loader::RobotModelLoader robot_model_loader;
    robot_model::RobotModelPtr robot_model;
    robot_state::RobotStatePtr kinematic_state;
    robot_model::JointModelGroup* joint_model_group;
    std::string move_group_;
    std::vector<std::string> link_names_;
    bool initialized_;
  };//end class


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


#endif //COLLISION_SHAPE_GENERATOR_H
