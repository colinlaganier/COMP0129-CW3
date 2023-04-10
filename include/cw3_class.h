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
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

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
  double cross_pick_grid_y_offset_;
  double cross_pick_grid_x_offset_;
  double naught_pick_grid_x_offset_;
  double naught_pick_grid_y_offset_;
  double drop_height_;


   ros::Subscriber cloud_sub_;


 
   /** \brief The input point cloud frame id. */
  std::string g_input_pc_frame_id_;

  /** \brief ROS publishers. */
  ros::Publisher g_pub_cloud;

  /** \brief ROS pose publishers. */
  ros::Publisher g_pub_pose;
 /** \brief cw1Q1: TF listener definition. */
  tf::TransformListener g_listener_;

  /** \brief Voxel Grid filter's leaf size. */
  double g_vg_leaf_sz;

  /** \brief Point Cloud (input) pointer. */
  PointCPtr g_cloud_ptr;
  
  /** \brief Point Cloud (filtered) pointer. */
  PointCPtr g_cloud_filtered, g_cloud_filtered2;
  
  /** \brief Point Cloud (filtered) sensros_msg for publ. */
  sensor_msgs::PointCloud2 g_cloud_filtered_msg;

  /** \brief ROS geometry message point. */
  geometry_msgs::PointStamped g_cyl_pt_msg;
  
  /** \brief Point Cloud (input). */
  pcl::PCLPointCloud2 g_pcl_pc;
  
  /** \brief Voxel Grid filter. */
  pcl::VoxelGrid<PointT> g_vx;
  
  /** \brief Pass Through filter. */
  pcl::PassThrough<PointT> g_pt;
  
  /** \brief Pass Through min and max threshold sizes. */
  double g_pt_thrs_min, g_pt_thrs_max;
  
  /** \brief KDTree for nearest neighborhood search. */
  pcl::search::KdTree<PointT>::Ptr g_tree_ptr;
  
  /** \brief Normal estimation. */
  pcl::NormalEstimation<PointT, pcl::Normal> g_ne;
  
  /** \brief Cloud of normals. */
  pcl::PointCloud<pcl::Normal>::Ptr g_cloud_normals, g_cloud_normals2;
  
  /** \brief Nearest neighborhooh size for normal estimation. */
  double g_k_nn;
  
  /** \brief SAC segmentation. */
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> g_seg; 
  
  /** \brief Extract point cloud indices. */
  pcl::ExtractIndices<PointT> g_extract_pc;

  /** \brief Extract point cloud normal indices. */
  pcl::ExtractIndices<pcl::Normal> g_extract_normals;
  
  /** \brief Point indices for plane. */
  pcl::PointIndices::Ptr g_inliers_plane;
    
  /** \brief Point indices for cylinder. */
  pcl::PointIndices::Ptr g_inliers_cylinder;
  
  /** \brief Model coefficients for the plane segmentation. */
  pcl::ModelCoefficients::Ptr g_coeff_plane;
  
  /** \brief Model coefficients for the culinder segmentation. */
  pcl::ModelCoefficients::Ptr g_coeff_cylinder;

    /** \brief Point cloud to hold plane and cylinder points. */
  PointCPtr g_cloud_plane, g_cloud_cylinder;

  pcl::PassThrough<PointT> pass;

  pcl::PointCloud<PointT>::Ptr cloud_filtered;

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
