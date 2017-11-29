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

struct OctomapKeyCompare {
  bool operator() (const octomap::OcTreeKey& lhs, const octomap::OcTreeKey& rhs) const
  {
    size_t h1 = size_t(lhs.k[0]) + 1447*size_t(lhs.k[1]) + 345637*size_t(lhs.k[2]);
    size_t h2 = size_t(rhs.k[0]) + 1447*size_t(rhs.k[1]) + 345637*size_t(rhs.k[2]);
    return h1< h2;
  }
};

struct NormalHistogram{
  int size;
  int histogram[6];

  NormalHistogram()
  {
    size = 0;
    for (int i=0; i<6; i++)
    {
      histogram[i] = 0;
    }
  }

  struct NormalHistogram& operator+=(const NormalHistogram& rhs)
  {
    size += rhs.size;
    for (int i=0; i<6; i++)
    {
      histogram[i] += rhs.histogram[i];
    }

    return *this;
  }
};


class MappingModule
{
public:
  struct VoxelDensity{
    int count;
    double total;
    double density;
  };

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

  double getAveragePointDensity();
  int getDensityAtOcTreeKey(octomap::OcTreeKey key);
  NormalHistogram getNormalHistogramAtOcTreeKey(octomap::OcTreeKey key);
  octomap::OcTree*   getOctomap();
  octomap::OcTree*   getOctomapPredicted();
  PointCloudXYZ::Ptr getProfilePointCloud();
  PointCloudXYZ::Ptr getPointCloud();

  bool isNodeFree(octomap::OcTreeNode node);
  bool isNodeOccupied(octomap::OcTreeNode node);
  bool isNodeUnknown(octomap::OcTreeNode node);

  void run();

  void updateVoxelDensities();
  void updateVoxelDensities(const PointCloudXYZ::Ptr& cloud);
  void updateVoxelNormals();

private:
  // =========
  // Methods
  // =========
  void addPointCloudToPointCloud(const PointCloudXYZ::Ptr& cloud_in, PointCloudXYZ::Ptr& cloud_out, double filter_res);
  void addPointCloudToPointCloud(const PointCloudXYZ::Ptr& cloud_in, PointCloudXYZ::Ptr& cloud_out);
  void addPredictedPointCloudToTree(octomap::OcTree* octree_in, PointCloudXYZ cloud_in);
  void addPointCloudToTree(octomap::OcTree* octree_in, PointCloudXYZ cloud_in, octomap::point3d sensor_origin, octomap::point3d sensor_dir, double range, bool isPlanar=false);

  void callbackScan(const sensor_msgs::LaserScan& laser_msg);
  void callbackDepth(const sensor_msgs::PointCloud2& cloud_msg);
  void callbackDepth2(const sensor_msgs::PointCloud2& cloud_msg);
  void callbackDepthSync(const sensor_msgs::PointCloud2ConstPtr& cloud_msg1, const sensor_msgs::PointCloud2ConstPtr& cloud_msg2);
  void createMaxRangeCloud();
  PointCloudXYZ::Ptr correctDepth(const sensor_msgs::PointCloud2& input_msg);
  void processDepth(const sensor_msgs::PointCloud2& cloud_msg);

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
  int counter_;
  // == Config
  double max_rgbd_range_;
  double sensor_data_min_height_;
  int save_iterations_;

  //Voxel grid resolutions
  double profile_grid_res_;
  double depth_grid_res_;
  double octree_res_;
  double octree_thresh_; //Threshold for occupancy
  double predicted_occupancy_value_;

  // Depth cloud correction
  PointXYZ* cloud_max_range_;
  int camera_width_px, camera_height_px;
  double camera_fov_vertical, camera_fov_horizontal;
  double camera_range_max, camera_range_min;
  double camera_range_upper_adjustment;

  octomap::point3d bound_min_, bound_max_;
  int camera_height_px_;
  int camera_width_px_;

  int ray_skipping_horizontal_;
  int ray_skipping_vertical_;

  // == Booleans
  bool is_debugging_; //Set to true to see debug text
  bool is_debug_load_state_;
  bool is_debug_save_state_;

  bool is_filling_octomap_;
  bool is_filling_octomap_continuously_;
  bool getCameraData;
  uint camera_done_flags_;
  uint all_done_flags_;
  bool is_scanning_;
  bool is_checking_symmetry_;
  bool is_integrating_prediction_;
  bool skip_load_map_;

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
  std::map<octomap::OcTreeKey, VoxelDensity, OctomapKeyCompare> voxel_densities_;
  std::map<octomap::OcTreeKey, NormalHistogram, OctomapKeyCompare> voxel_normals_;

  // == Strings
  std::string filename_octree_;
  std::string filename_octree_final_;
  std::string filename_pcl_;
  std::string filename_pcl_save_state_;
  std::string filename_pcl_final_;
  std::string filename_pcl_symmetry_;

  std::string topic_depth_;
  std::string topic_depth2_;
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
  ros::Subscriber sub_rgbd2_;
  ros::Subscriber sub_scan_;
  ros::Subscriber sub_scan_command_;

  tf::TransformListener *tf_listener_;

private:
  friend class boost::serialization::access;

  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & counter_;
    // == Config
    ar & max_rgbd_range_;
    ar & sensor_data_min_height_;
    ar & save_iterations_;

    //Voxel grid resolutions
    ar & profile_grid_res_;
    ar & depth_grid_res_;
    ar & octree_res_;
    ar & octree_thresh_; //Threshold for occupancy
    ar & predicted_occupancy_value_;

    //octomap::point3d bound_min_, bound_max_;
    ar & camera_height_px_;
    ar & camera_width_px_;

    ar & ray_skipping_horizontal_;
    ar & ray_skipping_vertical_;

    // == Booleans
    ar & is_debugging_; //Set to true to see debug text
    ar & is_filling_octomap_;
    ar & is_filling_octomap_continuously_;
    ar & getCameraData;
    ar & is_scanning_;
    ar & is_checking_symmetry_;
    ar & is_integrating_prediction_;
    ar & skip_load_map_;

    // == Profiling
    //std::vector<octomap::point3d> pose_vec_, dir_vec_;
    //std::vector< PointCloudXYZ > scan_vec_;
    ar &  laser_range_;


    // == Point clouds and octrees
    ar & cloud_ptr_rgbd_;
    //PointCloudXYZ::Ptr cloud_ptr_profile_;
    //PointCloudXYZ::Ptr cloud_ptr_profile_symmetry_;
    //octomap::OcTree* octree_;
    //octomap::OcTree* octree_prediction_;
    //std::map<octomap::OcTreeKey, int, OctomapKeyCompare> voxel_densities_;

    // == Strings
    ar & filename_octree_;
    ar & filename_octree_final_;
    ar & filename_pcl_;
    ar & filename_pcl_final_;
    ar & filename_pcl_symmetry_;

    ar & topic_depth_;
    ar & topic_depth2_;
    ar & topic_map_;
    ar & topic_scan_in_;
    ar & topic_scan_out_;
    ar & topic_rgbd_out_;
    ar & topic_tree_;
    ar & topic_tree_predicted_;

    /*
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
    */
  }
};

namespace boost {
  namespace serialization {

    template<class Archive>
    void serialize(Archive & ar, PointCloudXYZ::Ptr& cloud, const unsigned int version) {
        ar & cloud->points;
    }

    template<class Archive>
    void serialize(Archive & ar, PointXYZ& p, const unsigned int version) {
        ar & p.x;
        ar & p.y;
        ar & p.z;
    }

  } // namespace serialization
} // namespace boost

#endif // SENSING_AND_MAPPING_H
