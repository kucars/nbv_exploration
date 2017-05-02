#ifndef SENSING_AND_MAPPING_H
#define SENSING_AND_MAPPING_H

#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>


#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap_msgs/conversions.h>

//PCL
//#include <pcl/filters.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "nbv_exploration/common.h"
#include "nbv_exploration/symmetry_detector.h"

class MappingModule
{
public:
  // =========
  // Methods
  // =========
  MappingModule();

  bool commandGetCameraData();
  bool commandFinalMapLoad();
  bool commandFinalMapSave();
  bool commandProfilingStart();
  bool commandProfilingStop();
  bool commandProfileLoad();
  bool commandProfileSave();
  bool commandScanningStart();
  bool commandScanningStop();

  octomap::OcTree*   getOctomap();
  octomap::OcTree*   getOctomapPredicted();
  PointCloudXYZ::Ptr getPointCloud();
  void run();

private:
  // =========
  // Methods
  // =========
  void addPointCloudToPointCloud(const PointCloudXYZ::Ptr& cloud_in, PointCloudXYZ::Ptr& cloud_out, double filter_res);
  void addPointCloudToPointCloud(const PointCloudXYZ::Ptr& cloud_in, PointCloudXYZ::Ptr& cloud_out);
  void addPointCloudToTree(octomap::OcTree* octree_in, PointCloudXYZ cloud_in);
  void addPointCloudToTree(octomap::OcTree* octree_in, PointCloudXYZ cloud_in, octomap::point3d sensor_origin, octomap::point3d sensor_dir, double range, bool isPlanar=false);


  void callbackDepth(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
  void callbackScan(const sensor_msgs::LaserScan& laser_msg);

  void computeTreeUpdatePlanar(octomap::OcTree* octree_in, const octomap::Pointcloud& scan,
                        const octomap::point3d& origin, octomap::point3d& sensor_dir,
                        octomap::KeySet& free_cells, octomap::KeySet& occupied_cells,
                        double maxrange);

  void initializeParameters();
  void initializeTopicHandlers();
  void processScans();
  void updatePrediction(octomap::OcTree* octree_in, PointCloudXYZ cloud_in, octomap::point3d sensor_origin, octomap::point3d sensor_dir, double range, bool isPlanar);

  // =========
  // Variables
  // =========
  // == Config
  double max_rgbd_range_;
  double sensor_data_min_height_;

  //Voxel grid resolutions
  double profile_grid_res_;
  double depth_grid_res_;
  double octree_res_;

  octomap::point3d bound_min_, bound_max_;
  int camera_height_px_;
  int camera_width_px_;

  int ray_skipping_horizontal_;
  int ray_skipping_vertical_;

  // == Booleans
  bool is_debugging_; //Set to true to see debug text
  bool is_filling_octomap_;
  bool is_filling_octomap_continuously_;
  bool is_get_camera_data_;
  bool is_scanning_;
  bool is_checking_symmetry_;

  // == Profiling
  std::vector<octomap::point3d> pose_vec_, dir_vec_;
  std::vector< PointCloudXYZ > scan_vec_;
  double laser_range_;


  // == Point clouds and octrees
  PointCloudXYZ::Ptr cloud_ptr_rgbd_;
  PointCloudXYZ::Ptr cloud_ptr_profile_;
  PointCloudXYZ::Ptr cloud_ptr_profile_symmetry_;
  octomap::OcTree* octree_;
  octomap::OcTree* octree_prediction_;

  // == Strings
  std::string filename_octree_;
  std::string filename_octree_final_;
  std::string filename_pcl_;
  std::string filename_pcl_final_;
  std::string filename_pcl_symmetry_;

  std::string topic_depth_;
  std::string topic_map_;
  std::string topic_scan_in_;
  std::string topic_scan_out_;
  std::string topic_rgbd_out_;
  std::string topic_tree_;
  std::string topic_tree_predicted_;

  // == Publishers
  ros::NodeHandle ros_node_;

  ros::Publisher pub_global_cloud_;
  ros::Publisher pub_scan_cloud_;
  ros::Publisher pub_rgbd_cloud_;
  ros::Publisher pub_tree_;
  ros::Publisher pub_tree_prediction_;

  // == Subscriptions
  ros::Subscriber sub_rgbd_;
  ros::Subscriber sub_scan_;
  ros::Subscriber sub_scan_command_;

  tf::TransformListener *tf_listener_;
};

#endif // SENSING_AND_MAPPING_H
