/**
  **********************************************************************************
  * @file     cw3_class.cpp
  * @author   Colin Laganier, Jacob Nash, Carl Parsons
  * @date     2023-04-14
  * @brief   This file contains the constructor and methods for the cw2 class.
  *          The class advertises the services for the coursework tasks and 
  *          triggers the robot to perform the tasks.
  **********************************************************************************
  * @attention  Requires cw2_class header file.
  */

#include <cw3_class.h>

////////////////////////////////////////////////////////////////////////////////
// Constructor
////////////////////////////////////////////////////////////////////////////////

// pcl::visualization::CloudViewer viewer ("Viewer");

cw3::cw3(ros::NodeHandle nh):
g_cloud_ptr (new PointC), // input point cloud
  g_cloud_filtered (new PointC), // filtered point cloud
  g_cloud_filtered2 (new PointC), // filtered point cloud
  g_cloud_plane (new PointC), // plane point cloud
  g_cloud_cylinder (new PointC), // cylinder point cloud
  g_tree_ptr (new pcl::search::KdTree<PointT> ()), // KdTree
  g_cloud_normals (new pcl::PointCloud<pcl::Normal>), // segmentation
  g_cloud_normals2 (new pcl::PointCloud<pcl::Normal>), // segmentation
  g_inliers_plane (new pcl::PointIndices), // plane seg
  g_inliers_cylinder (new pcl::PointIndices), // cylidenr seg
  g_coeff_plane (new pcl::ModelCoefficients), // plane coeff
  g_coeff_cylinder (new pcl::ModelCoefficients), // cylinder coeff
  cloud_filtered (new pcl::PointCloud<PointT>)
{
  /* class constructor */

  nh_ = nh;

  // advertise solutions for coursework tasks
  t1_service_  = nh_.advertiseService("/task1_start", 
    &cw3::t1_callback, this);
  t2_service_  = nh_.advertiseService("/task2_start", 
    &cw3::t2_callback, this);
  t3_service_  = nh_.advertiseService("/task3_start",
    &cw3::t3_callback, this);

  // Define the publishers
  g_pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud", 1, true);
  g_pub_pose = nh.advertise<geometry_msgs::PointStamped> ("cyld_pt", 1, true);
  
  // Initialise ROS Subscribers //
  image_sub_ = nh_.subscribe("/r200/camera/color/image_raw", 1, &cw3::colorImageCallback, this);
  // Create a ROS subscriber for the input point cloud
  cloud_sub_ = nh_.subscribe("/r200/camera/depth_registered/points", 1, &cw3::pointCloudCallback, this);

  load_config();

  ROS_INFO("cw3 class initialised");

  // geometry_msgs::Pose scan_pose = point2Pose(scan_position_);
  // bool success = true;
  // success *= moveArm(scan_pose);
}

void cw3::load_config()
{
  // Define constants identified experimentally 
  // Issues with loading from config file (yaml, xml)

  // Pick and place constants
  inspection_distance_ = 0.6;
  // Angle offset to align gripper with cube
  angle_offset_ = 3.14159 / 4.0;

  drop_height_ = 0.30;


  cross_pick_grid_y_offset_ = 0;
  cross_pick_grid_x_offset_ = 2;
  naught_pick_grid_x_offset_ = 2;
  naught_pick_grid_y_offset_ = 2;



  // Pointcloud PassThrough threshold to restrict the pointcloud to the objects
  //g_pt_thrs_min = 0.0; 
  //g_pt_thrs_max = 0.77; 

  // Defining the Robot scanning position for task 3
  scan_position_.x = 0.3773;
  scan_position_.y = -0.0015;
  scan_position_.z = 0.8773;

  g_vg_leaf_sz = 0.01; // VoxelGrid leaf size: Better in a config file
  g_pt_thrs_min = 0.0; // PassThrough min thres: Better in a config file
  g_pt_thrs_max = 0.7; // PassThrough max thres: Better in a config file
  g_k_nn = 50; // Normals nn size: Better in a config file


  // Positions of the gripper for the different tasks 
  //basket_height_ = 0.40;
  camera_offset_ = 0.0425;
  // Pointcloud cutoff threshold for object identification
  //cube_basket_cutoff_ = 1000;
  // Precision of the estimated object position
  //position_precision_ = 1000.0;

}

///////////////////////////////////////////////////////////////////////////////

bool
cw3::t1_callback(cw3_world_spawner::Task1Service::Request &request,
  cw3_world_spawner::Task1Service::Response &response) 
{
  /* function which should solve task 1 */

  ROS_INFO("The coursework solving callback for task 1 has been triggered");

  double x = 0.04;

  bool success = pickAndPlace(request.object_point,request.goal_point,request.shape_type,x);

  return success;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw3::t2_callback(cw3_world_spawner::Task2Service::Request &request,
  cw3_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */

  ROS_INFO("The coursework solving callback for task 2 has been triggered");

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw3::t3_callback(cw3_world_spawner::Task3Service::Request &request,
  cw3_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */

  ROS_INFO("The coursework solving callback for task 3 has been triggered");

  return true;
}

///////////////////////////////////////////////////////////////////////////////

void
cw3::pointCloudCallback
  (const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  // Extract inout point cloud info
  g_input_pc_frame_id_ = cloud_input_msg->header.frame_id;
    
  // Convert to PCL data type
  pcl_conversions::toPCL (*cloud_input_msg, g_pcl_pc);
  pcl::fromPCLPointCloud2 (g_pcl_pc, *g_cloud_ptr);

   
  //pcl::PointCloud<PointT> cloud = *cloud.get();


  
  pass.setInputCloud (g_cloud_ptr);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.49, 0.51);
  // pass.setFilterLimits (0.45, 0.55);

    //pass.setFilterLimitsNegative (true);
    
  pass.filter(*g_cloud_filtered);

  // TODO: test if filter is empty
  // g_ne.setInputCloud(g_cloud_filtered);
  // g_ne.setSearchMethod(g_tree_ptr);
  // g_ne.setKSearch(g_k_nn);
  // g_ne.compute(*g_cloud_normals);

  // segPlane(g_cloud_filtered);
  // Perform the filtering
  //applyVX (g_cloud_ptr, g_cloud_filtered);
  //applyPT (g_cloud_ptr, g_cloud_filtered);
    
  // Publish the data
  //ROS_INFO ("Publishing Filtered Cloud 2");
  // if (g_cloud_plane->points.size() > 0)
  // {
  //   ROS_INFO ("Publishing Plane");
  //   pubFilteredPCMsg(g_pub_cloud, *g_cloud_plane);
  // }
  // else
  // {
  //   // ROS_INFO ("Publishing Filtered Cloud 4");
  //   pubFilteredPCMsg(g_pub_cloud, *g_cloud_filtered);
  // }
  pubFilteredPCMsg(g_pub_cloud, *g_cloud_filtered);

  // pcl::toROSMsg(*g_cloud_normals, g_cloud_normals_msg);
  // g_pub_cloud_normals.publish (g_cloud_normals_msg);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
// Point Cloud callback helper functions
////////////////////////////////////////////////////////////////////////////////

void
cw3::pubFilteredPCMsg (ros::Publisher &pc_pub,
                               PointC &pc)
{
  /* This function publishes the filtered pointcloud */

  // Publish the data
  pcl::toROSMsg(pc, g_cloud_filtered_msg);
  pc_pub.publish (g_cloud_filtered_msg);
  
  return;
}

void
cw3::segPlane (PointCPtr &in_cloud_ptr)
{
  // Create the segmentation object for the planar model
  // and set all the params
  g_seg.setOptimizeCoefficients (true);
  g_seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  g_seg.setNormalDistanceWeight (0.1); //bad style
  g_seg.setMethodType (pcl::SAC_RANSAC);
  g_seg.setMaxIterations (100); //bad style
  g_seg.setDistanceThreshold (0.03); //bad style
  g_seg.setInputCloud (in_cloud_ptr);
  g_seg.setInputNormals (g_cloud_normals);
  // Obtain the plane inliers and coefficients
  g_seg.segment (*g_inliers_plane, *g_coeff_plane);
  
  // Extract the planar inliers from the input cloud
  g_extract_pc.setInputCloud (in_cloud_ptr);
  g_extract_pc.setIndices (g_inliers_plane);
  g_extract_pc.setNegative (false);
  
  // Write the planar inliers to disk
  g_extract_pc.filter (*g_cloud_plane);
  
  // Remove the planar inliers, extract the rest
  g_extract_pc.setNegative (true);
  g_extract_pc.filter (*g_cloud_filtered2);
  g_extract_normals.setNegative (true);
  g_extract_normals.setInputCloud (g_cloud_normals);
  g_extract_normals.setIndices (g_inliers_plane);
  g_extract_normals.filter (*g_cloud_normals2);

  //ROS_INFO_STREAM ("Plane coefficients: " << *g_coeff_plane);
  ROS_INFO_STREAM ("PointCloud representing the planar component: "
                   << g_cloud_plane->size ()
                   << " data points.");
  // get number of elements in g_cloud_normals2
  ROS_INFO_STREAM ("Number of planes: "
                  << g_cloud_normals2->size () 
                  << " data points.");

}
    

////////////////////////////////////////////////////////////////////////////////

void
cw3::colorImageCallback(const sensor_msgs::Image& msg)
{
  /* This is the callback function for the RGB camera subscriber */ 
  
  // Setting up the static variables at the first callback 
  static bool setup = [&](){
        // Camera feed resolution
        cw3::color_image_width_ = msg.width;
        cw3::color_image_height_ = msg.height;

        // Computing the index of the middle pixel
        cw3::color_image_midpoint_ = cw3::color_channels_ * ((cw3::color_image_width_ * 
          (cw3::color_image_height_ / 2)) + (cw3::color_image_width_ / 2)) - cw3::color_channels_;

        return true;
    } ();

  this->color_image_data = msg.data;

  return;
}

////////////////////////////////////////////////////////////////////////////////
// void
// cw3::applyVX (PointCPtr &in_cloud_ptr,
//                       PointCPtr &out_cloud_ptr)
// {
//   g_vx.setInputCloud (in_cloud_ptr);
//   g_vx.setLeafSize (g_vg_leaf_sz, g_vg_leaf_sz, g_vg_leaf_sz);
//   g_vx.filter (*out_cloud_ptr);
  
//   return;
// }

////////////////////////////////////////////////////////////////////////////////
// void
// cw3::applyPT (PointCPtr &in_cloud_ptr,
//                       PointCPtr &out_cloud_ptr)
// {
//   g_pt.setInputCloud (in_cloud_ptr);
//   g_pt.setFilterFieldName ("y");
//   g_pt.setFilterLimits (g_pt_thrs_min, g_pt_thrs_max);
//   g_pt.filter (*out_cloud_ptr);
  
//   return;
// }

///////////////////////////////////////////////////////////////////////////////

bool
cw3::pickAndPlace(geometry_msgs::PointStamped object_point, 
                  geometry_msgs::PointStamped object_goal, 
                  std::string shape_type, 
                  double cube_size)
{
  geometry_msgs::Pose inspection_pose = point2Pose(object_point.point);
  inspection_pose.position.x -= camera_offset_;
  inspection_pose.position.z += inspection_distance_; // Position gripper above object


 ROS_INFO("%f",object_point.point.z);
  object_point.point.z = 0.15; 
  
  object_goal.point.z = drop_height_;
  if(shape_type == "nought"){
    object_point.point.x += naught_pick_grid_x_offset_ * cube_size;
    object_point.point.y += naught_pick_grid_y_offset_ * cube_size;
    object_goal.point.x += naught_pick_grid_x_offset_ * cube_size;
    object_goal.point.y += naught_pick_grid_y_offset_ * cube_size;
  }
  else if(shape_type == "cross"){
    object_point.point.x += cross_pick_grid_x_offset_ * cube_size;
    object_point.point.y += cross_pick_grid_y_offset_ * cube_size;
    object_goal.point.x += cross_pick_grid_x_offset_ * cube_size;
    object_goal.point.y += cross_pick_grid_y_offset_ * cube_size;
  }
  else{
    return false;
  }
  geometry_msgs::Pose drop_pose = point2Pose(object_goal.point); 
  geometry_msgs::Pose grasp_pose = point2Pose(object_point.point); ;



  // PERFORM PICK //
  bool success = true;
  ROS_INFO("%f",inspection_distance_);
  ROS_INFO("PERFORMING PICK");
  // Approach object
  success *= moveArm(inspection_pose);
  // TODO: Determine object orientation
  // TODO: Orient gripper
  // success *= moveGripper(gripper_open_);
  // success *= moveArm(grasp_pose);
  // // Grasp object
  // success *= moveGripper(gripper_closed_);
  // // Place object
  // success *= moveArm(drop_pose);
  // success *= moveGripper(gripper_open_);


  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw3::moveArm(geometry_msgs::Pose target_pose)
{
  /* This function moves the move_group to the target position */

  // setup the target pose
  ROS_INFO("Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  // create a movement plan for the arm
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // execute the planned path
  arm_group_.move();

  return success;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw3::moveGripper(float width)
{
  /* this function moves the gripper fingers to a new position. Joints are:
      - panda_finger_joint1
      - panda_finger_joint2
  */

  // safety checks
  if (width > gripper_open_) width = gripper_open_;
  if (width < gripper_closed_) width = gripper_closed_;

  // calculate the joint targets as half each of the requested distance
  double eachJoint = width / 2.0;

  // create a vector to hold the joint target for each joint
  std::vector<double> gripperJointTargets(2);
  gripperJointTargets[0] = eachJoint;
  gripperJointTargets[1] = eachJoint;

  // apply the joint target
  hand_group_.setJointValueTarget(gripperJointTargets);

  // move the robot hand
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (hand_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  hand_group_.move();

  return success;
}

///////////////////////////////////////////////////////////////////////////////

geometry_msgs::Pose
cw3::point2Pose(geometry_msgs::Point point){
  /* This function produces a "gripper facing down" pose given a xyz point */

  // Position gripper above point
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);
  // Gripper Orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion orientation = tf2::toMsg(q_result);

  // set the desired Pose
  geometry_msgs::Pose pose;
  pose.position = point;
  pose.orientation = orientation;

  return pose;
}