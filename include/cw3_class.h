/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw3_team_<your_team_number> package */

// include guards, prevent .h file being defined multiple times (linker error)
#ifndef CW3_CLASS_H_
#define CW3_CLASS_H_

// system includes
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


// include services from the spawner package - we will be responding to these
#include "cw3_world_spawner/Task1Service.h"
#include "cw3_world_spawner/Task2Service.h"
#include "cw3_world_spawner/Task3Service.h"

// // include any services created in this package
// #include "cw3_team_x/example.h"


class cw3
{
public:

  double angle_offset_;
  double inspection_distance_;
  double camera_offset_;
  double gripper_open_ = 80e-3;
  double gripper_closed_ = 0.0;
  int cross_pick_grid_y_offset_;
  int cross_pick_grid_x_offset_;
  int naught_pick_grid_x_offset_;
  int naught_pick_grid_y_offset_;
  double drop_height_;

  /* ----- class member functions ----- */

  // constructor
  cw3(ros::NodeHandle nh);

  void
  load_config();

  // service callbacks for tasks 1, 2, and 3
  bool 
  t1_callback(cw3_world_spawner::Task1Service::Request &request,
    cw3_world_spawner::Task1Service::Response &response);
  bool 
  t2_callback(cw3_world_spawner::Task2Service::Request &request,
    cw3_world_spawner::Task2Service::Response &response);
  bool 
  t3_callback(cw3_world_spawner::Task3Service::Request &request,
    cw3_world_spawner::Task3Service::Response &response);

  /* ----- class member variables ----- */

  ros::NodeHandle nh_;
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;
  
  /** \brief MoveIt interface to move groups to seperate the arm and the gripper,
  * these are defined in urdf. */
  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};
    ////////////////////////////////////////////////////////////////////////////////
  // Task 1 functions
  ////////////////////////////////////////////////////////////////////////////////

  bool
  MoveArm(geometry_msgs::Pose target_pose);

  /** \brief Task 1 function, pick and place an object from a target
   * to a goal using the gripper. 
   *
   * \input[in] object_loc location of object to pick cube
   * \input[in] goal_loc location to place cube in basket
   *
   * \return true if object is picked and placed
  */
  bool
  PickAndPlace(geometry_msgs::PointStamped objectPoint, geometry_msgs::PointStamped objectGoal, std::string shapeType, double x );

  bool
  MoveGripper(float width);

  geometry_msgs::Pose
  Point2Pose(geometry_msgs::Point point);

  /** \brief MoveIt function for moving the move_group to the target position.
    *
    * \input[in] target_pose pose to move the arm to
    *
    * \return true if moved to target position 
    */

};

#endif // end of include guard for cw3_CLASS_H_
