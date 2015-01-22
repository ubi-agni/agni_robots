/**
 * @file   collision_shape_generator.cpp
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Nov 2014
 *
 * @brief  generate shape primitives for collision objects in the robot description
 *
 */

#include <agni_robot_viz/collision_shape_generator.hpp>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>

CollisionShapeGenerator::CollisionShapeGenerator() :robot_model_loader("robot_description")
{
  initialized_ = false;
}

CollisionShapeGenerator::~CollisionShapeGenerator() 
{

}

//! Initialize the internal robot model and kinematics
//! \param move_group Name of the move group considered in the kinematics
//! \param link_names vector of link names considered in the kinematics (should belong to the move_group)
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

//! Update the internal kinematic model posture
//! \param msg a JointState describing the current posture
void CollisionShapeGenerator::updatePosition(const sensor_msgs::JointStatePtr &msg)
{
  if (initialized_)
  {
    kinematic_state->setVariablePositions(msg->name,msg->position);
  }
}

//! Compute the collision shapes for the current posture
//! \param arr an array of markers as the output
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
        tf::Transform mesh_offset;
        
        mark.header.frame_id = robot_model->getModelFrame();
        mark.header.stamp = tm;
        mark.ns="robot";
        mark.id=i*100+j;
        mark.color = color;
        mark.text = link_names_[i];
        // we prefer using the collision shape
        if (lm->getShapes().size() > 0)
        {
          if (lm->getShapes()[j].get()->type == shapes::MESH)
          {
            //std::cerr<<"creating a bbox"<<std::endl;
            // find a bounding box 
            const shapes::Mesh *mesh = static_cast<const shapes::Mesh*>(lm->getShapes()[j].get());
            Eigen::Vector3d center,dim;
            if (mesh->vertex_count > 1)
            {
              std::vector<double> vmin(3, std::numeric_limits<double>::max());
              std::vector<double> vmax(3, -std::numeric_limits<double>::max());
              for (unsigned int i = 0; i < mesh->vertex_count ; ++i)
              {
                unsigned int i3 = i * 3;
                for (unsigned int k = 0 ; k < 3 ; ++k)
                {
                  unsigned int i3k = i3 + k;
                  if (mesh->vertices[i3k] > vmax[k])
                  vmax[k] = mesh->vertices[i3k];
                  if (mesh->vertices[i3k] < vmin[k])
                  vmin[k] = mesh->vertices[i3k];
                }
              }
              dim=Eigen::Vector3d(vmax[0] - vmin[0], vmax[1] - vmin[1], vmax[2] - vmin[2]);
              center=Eigen::Vector3d((vmax[0] + vmin[0])*0.5, (vmax[1] + vmin[1])*0.5, (vmax[2] + vmin[2])*0.5);
              //std::cerr<<"center z"<< center[2]<<std::endl;
           /* if (mesh->vertex_count > 1)
            {
              double mx = std::numeric_limits<double>::max();
              Eigen::Vector3d min( mx, mx, mx);
              Eigen::Vector3d max(-mx, -mx, -mx);
              unsigned int cnt = mesh->vertex_count * 3;
              for (unsigned int i = 0; i < cnt ; i+=3)
              {
                Eigen::Vector3d v(mesh->vertices[i+0], mesh->vertices[i+1], mesh->vertices[i+2]);
                min = min.cwiseMin(v);
                max = max.cwiseMax(v);
              }
            
              center = (min + max) * 0.5;
              dim = (max - min);*/
          
            }
            mark.type = visualization_msgs::Marker::CUBE;
            mark.scale.x=dim[0];
            mark.scale.y=dim[1];
            mark.scale.z=dim[2];
            
            mesh_offset= tf::Transform (tf::Quaternion (0.0,0.0,0.0,1.0).normalize(),
                        tf::Vector3 (center[0], center[1], center[2]));
          }
          else
          {
            //std::cerr<<"construc from shape type "<<lm->getShapes()[j].get()->type << std::endl;
            if (!shapes::constructMarkerFromShape(lm->getShapes()[j].get(), mark))
              continue;
            // if the object is invisible (0 volume) we skip it
            if (fabs(mark.scale.x * mark.scale.y * mark.scale.z) < std::numeric_limits<float>::epsilon())
            {
              std::cerr<<"missing scale"<<std::endl;
              continue;
            }
            mesh_offset.setIdentity(); 	
            
          }
          // first get the link pose
          tf::Transform link_pose;
          tf::poseEigenToTF (kinematic_state->getCollisionBodyTransform(lm, j) , link_pose);
          // apply the local mesh transform (if any)
          link_pose*=mesh_offset;
          mesh_offset.setRotation (mesh_offset.getRotation().normalize());
          // store that into the marker
          tf::poseTFToMsg(link_pose,mark.pose);
          //if(mark.type == visualization_msgs::Marker::MESH_RESOURCE && mark.type != visualization_msgs::Marker::TRIANGLE_LIST)
          arr.markers.push_back(mark);
        }
      
      }
    }
    //robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);
    //return msg;
  }
}
