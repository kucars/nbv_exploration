/* Author: Abdullah Abduldayem
 * Derived from coverage_quantification.cpp
 */

// catkin_make -DCATKIN_WHITELIST_PACKAGES="aircraft_inspection" && rosrun aircraft_inspection wall_follower

#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

#include <nbv_exploration/MappingSrv.h>
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

#include <nbv_exploration/pose_conversion.h>

//PCL
//#include <pcl/filters.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "utilities/console_utility.h"
ConsoleUtility cc;


// ===================
// === Variables  ====
// ===================
bool isDebug = !true; //Set to true to see debug text
bool isDebugContinuousStates = !true;
bool is_get_camera_data = false;
bool is_batch_profiling = true;

// Config
double max_range;
double sensor_data_min_height;
double profile_grid_res, depth_grid_res; //Voxel grid resolution
double octree_res;
octomap::point3d bound_min, bound_max;
double noise_coefficient;
int camera_width_px, camera_height_px, ray_skipping_vertical, ray_skipping_horizontal;

// Profiling --------
bool isScanning = false;
std::vector<octomap::point3d> pose_vec, dir_vec;
std::vector< pcl::PointCloud<pcl::PointXYZRGB> > scan_vec;
double laser_range = -1;

// == Consts
std::string filename_pcl = "profile_cloud.pcd";
std::string filename_octree = "profile_octree.ot";

std::string topic_depth        = "/nbv_exploration/depth"; //"/iris/xtion_sensor/iris/xtion_sensor_camera/depth/points";
std::string topic_position     = "/iris/ground_truth/pose";
std::string topic_scan_in      = "/nbv_exploration/scan"; //"scan_in";
std::string topic_scan_command = "/nbv_exploration/scan_command";

std::string topic_map          = "/nbv_exploration/global_map_cloud";
std::string topic_scan_out     = "/nbv_exploration/scan_cloud";
std::string topic_rgbd_out     = "/nbv_exploration/rgbd_cloud";
std::string topic_tree         = "/nbv_exploration/output_tree";

// == Navigation variables
geometry_msgs::Pose mobile_base_pose;

// == Publishers
ros::Publisher pub_global_cloud;
ros::Publisher pub_scan_cloud;
ros::Publisher pub_rgbd_cloud;
ros::Publisher pub_tree;

// == Subsctiptions
ros::Subscriber sub_kinect;
ros::Subscriber sub_pose;
ros::Subscriber sub_scan;
ros::Subscriber sub_scan_command;

tf::TransformListener *tf_listener;


// == Point clouds and octrees
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sensed(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbd_cloud_ptr;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr profile_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

octomap::OcTree* tree;

// ======================
// Function prototypes (@todo: move to header file)
// ======================

void callbackDepth(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
void callbackPosition(const geometry_msgs::PoseStamped& pose_msg);
void callbackScan(const sensor_msgs::LaserScan& laser_msg);
bool callbackCommand(nbv_exploration::MappingSrv::Request  &req, nbv_exploration::MappingSrv::Response &res);

void processScans();
void addPointCloudToTree(pcl::PointCloud<pcl::PointXYZRGB> cloud_in, octomap::point3d sensor_origin, octomap::point3d sensor_dir, double range, bool isPlanar=false);

void addToGlobalCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out, bool should_filter = true);

void sigIntHandler(int sig)
{
  std::cout << cc.yellow << "Handling SIGINT exception\n" << cc.reset;
  
  // Forces ros::Rate::sleep() to end if it's stuck (for example, Gazebo isn't running)
  ros::shutdown();
}

int main(int argc, char **argv)
{
    // >>>>>>>>>>>>>>>>>
    // Initialize ROS
    // >>>>>>>>>>>>>>>>>
    std::cout << cc.red << "Begin Sensing\n" << cc.reset;

    ros::init(argc, argv, "sensing_and_mapping", ros::init_options::NoSigintHandler);
    ros::NodeHandle ros_node;

    // >>>>>>>>>>>>>>>>>
    // Parameters
    // >>>>>>>>>>>>>>>>>
    ros::param::param("~depth_range_max", max_range, 5.0);
    ros::param::param("~octree_resolution", octree_res, 0.2);
    ros::param::param("~sensor_data_min_height", sensor_data_min_height, 0.5);
    ros::param::param("~voxel_grid_resolution_profile", profile_grid_res, 0.1); 
    ros::param::param("~voxel_grid_resolution_depth_sensor", depth_grid_res, 0.1); 
    
    double obj_x_min, obj_x_max, obj_y_min, obj_y_max, obj_z_min, obj_z_max;
    ros::param::param("~object_bounds_x_min", obj_x_min,-1.0);
    ros::param::param("~object_bounds_x_max", obj_x_max, 1.0);
    ros::param::param("~object_bounds_y_min", obj_y_min,-1.0);
    ros::param::param("~object_bounds_y_max", obj_y_max, 1.0);
    ros::param::param("~object_bounds_z_min", obj_z_min, 0.0);
    ros::param::param("~object_bounds_z_max", obj_z_max, 1.0);
    
    bound_min = octomap::point3d(obj_x_min, obj_y_min, obj_z_min);
    bound_max = octomap::point3d(obj_x_max, obj_y_max, obj_z_max);
			  
    
    ros::param::param("~noise_coefficient", noise_coefficient, 0.02);
    
    ros::param::param("~width_px", camera_width_px, 640);
    ros::param::param("~height_px", camera_height_px, 480);
    ros::param::param("~ray_skipping_vertical", ray_skipping_vertical, 1);
    ros::param::param("~ray_skipping_horizontal", ray_skipping_horizontal, 1);

    // >>>>>>>>>>>>>>>>>
    // Subscribers / Servers
    // >>>>>>>>>>>>>>>>>
    
    // Sensor data
    sub_kinect = ros_node.subscribe(topic_depth, 1, callbackDepth);
    sub_pose   = ros_node.subscribe(topic_position, 1, callbackPosition);
    sub_scan   = ros_node.subscribe(topic_scan_in, 1, callbackScan);
    
    ros::ServiceServer service = ros_node.advertiseService("/nbv_exploration/mapping_command", callbackCommand);
    
    tf_listener = new tf::TransformListener();
    
    // >>>>>>>>>>>>>>>>>
    // Publishers
    // >>>>>>>>>>>>>>>>>
    pub_global_cloud  = ros_node.advertise<sensor_msgs::PointCloud2>(topic_map, 10);
    pub_scan_cloud    = ros_node.advertise<sensor_msgs::PointCloud2>(topic_scan_out, 10);
    pub_rgbd_cloud    = ros_node.advertise<sensor_msgs::PointCloud2>(topic_rgbd_out, 10);
    pub_tree          = ros_node.advertise<octomap_msgs::Octomap>(topic_tree, 10);
    
    
    
    // >>>>>>>>>>>>>>>>>
    // Main function
    // >>>>>>>>>>>>>>>>>
    std::cout << cc.magenta << "\nStarted\n" << cc.reset;
    std::cout << "Listening for the following topics: \n";
    std::cout << "\t" << topic_depth << "\n";
    std::cout << "\t" << topic_position << "\n";
    std::cout << "\t" << topic_scan_in << "\n";
    std::cout << "\n";
    
    int i=0;
    ros::Rate rate(30);
    while(ros::ok())
    {
      //Publish once a second, but update 20 times a second
      
      if (i>1)
      {
        i--;
      }
      else
      {
        i=30;
        // == Publish
        // Profile Cloud
        if (profile_cloud_ptr)
        {
          sensor_msgs::PointCloud2 cloud_msg;
          pcl::toROSMsg(*profile_cloud_ptr, cloud_msg); 	//cloud of original (white) using original cloud
          cloud_msg.header.frame_id = "world";
          cloud_msg.header.stamp = ros::Time::now();
          pub_scan_cloud.publish(cloud_msg);
        }
        
        // RGB-D Cloud
        if (rgbd_cloud_ptr)
        {
          sensor_msgs::PointCloud2 cloud_msg;
          pcl::toROSMsg(*rgbd_cloud_ptr, cloud_msg); 	//cloud of original (white) using original cloud
          cloud_msg.header.frame_id = "world";
          cloud_msg.header.stamp = ros::Time::now();
          pub_rgbd_cloud.publish(cloud_msg);
        }
      
        // Octomap
        if (tree)
        {
          octomap_msgs::Octomap msg;
          octomap_msgs::fullMapToMsg (*tree, msg);
          
          msg.header.frame_id = "world";
          msg.header.stamp = ros::Time::now();
          pub_tree.publish(msg);
        }
      }
    
      // Sleep
      ros::spinOnce();
      rate.sleep();
    }
    
    //ros::spin();
    return 0;
}


bool callbackCommand(nbv_exploration::MappingSrv::Request  &request,
                     nbv_exploration::MappingSrv::Response &response)
{
  bool error = false;
  
  ros::Rate sleep_rate(10);
  
  switch(request.data)
  {
    case nbv_exploration::MappingSrv::Request::START_SCANNING:
      isScanning = true;
      std::cout << cc.green << "Started scanning\n" << cc.reset;
      
      response.data = response.ACK;
      break;
    
    
    case nbv_exploration::MappingSrv::Request::STOP_SCANNING:
      isScanning = false;
      std::cout << cc.green << "Processing " << scan_vec.size() << " scans...\n" << cc.reset;
      
      if (is_batch_profiling)
        processScans();
      
      response.data = response.DONE;
      break;
       
      
    case nbv_exploration::MappingSrv::Request::START_PROFILING:
      std::cout << cc.green << "Started profiling\n" << cc.reset;
    
      if (!tree)
      {
        tree = new octomap::OcTree (octree_res);
        tree->setBBXMin( bound_min );	       
        tree->setBBXMax( bound_max );
      }
        
      
      response.data = response.ACK;
      break;
      
      
    case nbv_exploration::MappingSrv::Request::STOP_PROFILING:
      isScanning = false;
      std::cout << cc.green << "Done profiling\n" << cc.reset;
      
      response.data = response.ACK;
      break;
      
      
    case nbv_exploration::MappingSrv::Request::SAVE_MAP:
      // Point cloud
      if (!profile_cloud_ptr)
      {
        response.data  = response.ERROR;
        response.error = "No point cloud data available";
        error = true;
        break;
      }
      pcl::io::savePCDFileASCII (filename_pcl, *profile_cloud_ptr);
      
      // Octree
      if (!tree)
      {
        response.data  = response.ERROR;
        response.error = "No octomap data available";
        error = true;
        break;
      }
      if (!tree->write(filename_octree))
      {
        response.data  = response.ERROR;
        response.error = "Failed to save octomap data to " + filename_octree;
        error = true;
        break;
      }
      
      std::cout << cc.green << "Successfully saved map\n" << cc.reset;
      
    case nbv_exploration::MappingSrv::Request::GET_CAMERA_DATA:
      is_get_camera_data = true;
      std::cout << cc.magenta << "Waiting for camera data\n" << cc.reset;
      
      for (int i=0; i<10 && ros::ok() && is_get_camera_data; i++)
      {
        ros::spinOnce();
        sleep_rate.sleep();
      }
      
      if (is_get_camera_data)
      {
        response.data = response.ERROR;
        response.error = "Could not get camera data";
      }
      else
        response.data = response.DONE;
      
      break;
      
      
    case nbv_exploration::MappingSrv::Request::LOAD_MAP:
      // Point cloud
      std::cout << "Reading " << filename_pcl << "\n";
      if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename_pcl, *profile_cloud_ptr) == -1) //* load the file
      {
        response.data  = response.ERROR;
        response.error = "Failed to load point cloud: Could not read file " + filename_pcl;
        error = true;
        break;
      }
      
      // Octree
      std::cout << "Reading " << filename_octree << "\n";
      
      octomap::AbstractOcTree* temp_tree = octomap::AbstractOcTree::read(filename_octree);
      if(temp_tree)
      { // read error returns NULL
        tree = dynamic_cast<octomap::OcTree*>(temp_tree);
        
        if (tree)
        {
          /*
          // Scale octree probabilities (bring them closer to 50%)
          for (octomap::OcTree::iterator it = tree->begin(), end = tree->end(); it != end; ++it)
          {
            tree->updateNode(it.getKey(), -0.5f*it->getLogOdds(), true);
          }
          tree->updateInnerOccupancy();
          
          std::cout << cc.green << "Successfully scaled probabilities\n" << cc.reset;
          response.data  = response.DONE;
          break;
          */
        } 
        else
        {
          response.data  = response.ERROR;
          response.error = "Failed to load octomap: Type cast failed";
          error = true;
          break;
        }
      }
      else
      {
        response.data  = response.ERROR;
        response.error = "Failed to load octomap: Could not read file " + filename_octree;
        error = true;
        break;
      }
      
      std::cout << cc.green << "Successfully loaded maps\n" << cc.reset;
      response.data  = response.DONE;
      break;
  }
  
  if (error)
    ros::shutdown();
  
  return true;
}

// Update global position of UGV
void callbackPosition(const geometry_msgs::PoseStamped& pose_msg)
{
    if (isDebug && isDebugContinuousStates){
        std::cout << cc.magenta << "Grabbing location\n" << cc.reset;
    }
    
    // Save UGV pose
    mobile_base_pose = pose_msg.pose;
}

void callbackScan(const sensor_msgs::LaserScan& laser_msg){
  if (isDebug && isDebugContinuousStates){
    std::cout << cc.magenta << "Scan readings\n" << cc.reset;
  }
  
  if (!isScanning)
    return;
  
  // == Get transform
  tf::StampedTransform transform;
  Eigen::Matrix4d tf_eigen;
  
  try{
    tf_listener->lookupTransform("world", "/iris/hokuyo_laser_link", ros::Time(0), transform);
    tf_eigen = pose_conversion::convertStampedTransform2Matrix4d(transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    //ros::Duration(1.0).sleep();
    return;
  }
  
  // == Add scan to point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_far_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  int steps = (laser_msg.angle_max - laser_msg.angle_min)/laser_msg.angle_increment;
  float step_size = (laser_msg.angle_max - laser_msg.angle_min)/steps;
  
  laser_range = laser_msg.range_max;
  
  // == Discard points outside range
  for (int i=0; i<steps; i++)
  {
    float r = laser_msg.ranges[i];
    float angle = laser_msg.angle_min + step_size*i;
    
    if (r < laser_msg.range_min)
    {
      // Discard points that are too close
      continue;
    }
    
    // Add scan point to temporary point cloud
    pcl::PointXYZRGB p;
    
    p.x = r*cos(angle);
    p.y = r*sin(angle);
    p.z = 0;
    
    if (r > laser_msg.range_max)
      scan_far_ptr->push_back(p);
    else
      scan_ptr->push_back(p);
  }
  
  // == Transform scan point cloud
  pcl::transformPointCloud(*scan_ptr, *scan_ptr, tf_eigen);
  pcl::transformPointCloud(*scan_far_ptr, *scan_far_ptr, tf_eigen);
  
  // == Discard points close to the ground
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_ptr_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  for (int i=0; i<scan_ptr->points.size(); i++)
  {
    if (scan_ptr->points[i].z > sensor_data_min_height)
    {
      scan_ptr_filtered->push_back(scan_ptr->points[i]);
    }
  }
  
  // == Add to global map
  addToGlobalCloud(scan_ptr_filtered, profile_cloud_ptr, true);
  
  // == Add to vectors for later octomap processing
  octomap::point3d sensor_origin (transform.getOrigin().x(),
                                  transform.getOrigin().y(),
                                  transform.getOrigin().z());
                                  
  octomap::point3d sensor_dir =  pose_conversion::getOctomapDirectionVectorFromTransform(transform);
  
  if (is_batch_profiling)
  {
    pose_vec.push_back(sensor_origin);
    dir_vec.push_back(sensor_dir);
    scan_vec.push_back(*scan_ptr + *scan_far_ptr); 
  }
  else
  {
    addPointCloudToTree(*scan_ptr + *scan_far_ptr, sensor_origin, sensor_dir, laser_range);
  }
}


void callbackDepth(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    if (isDebug && isDebugContinuousStates){
        std::cout << cc.green << "Depth sensing\n" << cc.reset;
    }
    
    if (!is_get_camera_data)
    {
      //std::cout << "Not allowed to process depth data yet\n";
      return;
    }
    
    // == Convert to pcl pointcloud
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_raw_ptr;
    
    pcl::fromROSMsg (*cloud_msg, cloud);
    cloud_raw_ptr = cloud.makeShared();
    
    // == Create cloud without points that are too far
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_distance_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    for (int i=0; i<cloud.points.size(); i++)
      if (cloud.points[i].z <= max_range)
        cloud_distance_ptr->push_back(cloud.points[i]);
    
    // == Transform
    tf::StampedTransform transform;
    try{
        // Listen for transform
        tf_listener->lookupTransform("world", "iris/xtion_sensor/camera_depth_optical_frame", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    
    // == Convert tf:Transform to Eigen::Matrix4d
    Eigen::Matrix4d tf_eigen = pose_conversion::convertStampedTransform2Matrix4d(transform);
    
    // == Transform point cloud to global frame
    pcl::transformPointCloud(*cloud_raw_ptr, *cloud_raw_ptr, tf_eigen);
    pcl::transformPointCloud(*cloud_distance_ptr, *cloud_distance_ptr, tf_eigen);
    
    // == Add filtered to final cloud
    addToGlobalCloud(cloud_distance_ptr, rgbd_cloud_ptr, true);
    
    // == Update octomap
    octomap::point3d origin (transform.getOrigin().x(),
                             transform.getOrigin().y(),
                             transform.getOrigin().z());
    octomap::point3d sensor_dir = pose_conversion::getOctomapDirectionVectorFromTransform(transform);
    
    addPointCloudToTree(*cloud_raw_ptr, origin, sensor_dir, max_range, true);
    
    // == Done updating
    is_get_camera_data = false;
}

void computeTreeUpdatePlanar(const octomap::Pointcloud& scan, const octomap::point3d& origin, octomap::point3d& sensor_dir,
                      octomap::KeySet& free_cells, octomap::KeySet& occupied_cells,
                      double maxrange)
{
  /*
   * Based on the implimentation of computeUpdate in http://octomap.github.io/octomap/doc/OccupancyOcTreeBase_8hxx_source.html
   * 
   * This method does not use "max_range" as a radial/Eucledian distance (as in the original implimentation)
   * Instead, it is used to evaluate the perpendicular distance to the far plane of the camera
   * @todo: bbx limit
   */
  
  sensor_dir.normalize();
  
  bool lazy_eval = false;
  bool use_ray_skipping = false;
  if (camera_width_px*camera_height_px == scan.size() &&
      (ray_skipping_horizontal != 1 || ray_skipping_horizontal != 1))
  {
    use_ray_skipping = true;
    std::cout << "Ray skipping\n";
  }
  
  // create as many KeyRays as there are OMP_THREADS defined,
  // one buffer for each thread
  std::vector<octomap::KeyRay> keyrays;
  #ifdef _OPENMP
    #pragma omp parallel
    #pragma omp critical
    {
      if (omp_get_thread_num() == 0){
        keyrays.resize(omp_get_num_threads());
      }

    } // end critical
  #else
    keyrays.resize(1);
  #endif
  

#ifdef _OPENMP
  omp_set_num_threads(keyrays.size());
  #pragma omp parallel for schedule(guided)
#endif
  for (int i = 0; i < (int)scan.size(); ++i)
  {
    if (use_ray_skipping)
    {
      int ix = i%camera_width_px;
      int iy = i/camera_width_px;
      
      if (ix % ray_skipping_horizontal != 0 || iy % ray_skipping_horizontal != 0)
        continue;
    }
    
    const octomap::point3d& p = scan[i];
    unsigned threadIdx = 0;
    #ifdef _OPENMP
    threadIdx = omp_get_thread_num();
    #endif
    
    octomap::KeyRay* keyray = &(keyrays.at(threadIdx));
    
    // Gets the perpendicular distance from the UAV's direction vector
    double perp_dist = sensor_dir.dot(p - origin);

    if (maxrange < 0.0 || perp_dist <= maxrange) 
    { // is not maxrange meas.
      // free cells
      if (tree->computeRayKeys(origin, p, *keyray))
      {
        #ifdef _OPENMP
        #pragma omp critical (free_insert)
        #endif
        {
          free_cells.insert(keyray->begin(), keyray->end());
        }
      }
      // occupied endpoint
      octomap::OcTreeKey key;
      if (tree->coordToKeyChecked(p, key)){
        #ifdef _OPENMP
        #pragma omp critical (occupied_insert)
        #endif
        {
          occupied_cells.insert(key);
        }
      }
    }
    else
    { // user set a maxrange and length is above
      octomap::point3d direction = (p - origin).normalized ();
      double max_range_to_point = maxrange/sensor_dir.dot(direction); 
      
      octomap::point3d new_end = origin + direction * (float) max_range_to_point;
      if (tree->computeRayKeys(origin, new_end, *keyray)){
        #ifdef _OPENMP
        #pragma omp critical (free_insert)
        #endif
        {
          free_cells.insert(keyray->begin(), keyray->end());
        }
      }
    } // end if maxrange
  } // end for all points, end of parallel OMP loop

  // prefer occupied cells over free ones (and make sets disjunct)
  for(octomap::KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; )
  {
    if (occupied_cells.find(*it) != occupied_cells.end())
      it = free_cells.erase(it);
    
    else 
      ++it;
  }
}

void addPointCloudToTree(pcl::PointCloud<pcl::PointXYZRGB> cloud_in, octomap::point3d sensor_origin, octomap::point3d sensor_dir, double range, bool isPlanar)
{
  // Note that "range" is the perpendicular distance to the end of the camera plane
  
  octomap::Pointcloud ocCloud;
  for (int j=0; j<cloud_in.points.size(); j++)
  {
    ocCloud.push_back(cloud_in.points[j].x,
                      cloud_in.points[j].y,
                      cloud_in.points[j].z);
  }
  
  // == Insert point cloud based on planar (camera) or spherical (laser) scan data
  octomap::KeySet free_cells, occupied_cells;
  
  if (isPlanar)
    computeTreeUpdatePlanar(ocCloud, sensor_origin, sensor_dir, free_cells, occupied_cells, range);
  else
    tree->computeUpdate(ocCloud, sensor_origin, free_cells, occupied_cells, range);
  
  // insert data into tree using continuous probabilities -----------------------
  for (octomap::KeySet::iterator it = free_cells.begin(); it != free_cells.end(); ++it)
  {
    tree->updateNode(*it, false);
  }
  
  for (octomap::KeySet::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it)
  {
    octomap::point3d p = tree->keyToCoord(*it);
    if (p.z() <= sensor_data_min_height)
    {
      continue;
    }
    else
    {
      tree->updateNode(*it, true);
    }
  }
  
  /*
  // insert data into tree using binary probabilities -----------------------
  for (octomap::KeySet::iterator it = free_cells.begin(); it != free_cells.end(); ++it)
  {
    tree->updateNode(*it, false);
  }
  
  for (octomap::KeySet::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it)
  {
    octomap::point3d p = tree->keyToCoord(*it);
    if (p.z() <= sensor_data_min_height)
      tree->updateNode(*it, false);
    else
      tree->updateNode(*it, true);
  }
  */
}

void processScans()
{
  double t_start, t_end;
  t_start = ros::Time::now().toSec();
  
  int count =0;
  for (int i=scan_vec.size()-1; i>=0; i--)
  {
    octomap::point3d sensor_origin = pose_vec[i];
    octomap::point3d sensor_dir = dir_vec[i];
    pcl::PointCloud<pcl::PointXYZRGB> scan = scan_vec[i];
    
    addPointCloudToTree(scan_vec[i], sensor_origin, sensor_dir, laser_range);
    
    // == Pop
    pose_vec.pop_back();
    dir_vec.pop_back();
    scan_vec.pop_back();
    count++;
  }
  
  // == Timing
  t_end = ros::Time::now().toSec();
  std::cout << cc.green << "Done processing.\n" << cc.reset;
  std::cout << "   Total time: " << t_end-t_start << " sec\tTotal scan: " << count << "\t(" << (t_end-t_start)/count << " sec/scan)\n";
}




void addToGlobalCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out, bool should_filter) {
    if (isDebug && isDebugContinuousStates){
        std::cout << cc.green << "MAPPING\n" << cc.reset;
    }
    
    // Initialize global cloud if not already done so
    if (!cloud_out){
        cloud_out = cloud_in;
        return;
    }

    // Only add points that meet the min height requirement
    for (int i=0; i<cloud_in->points.size(); i++)
      if (cloud_in->points[i].z >= sensor_data_min_height)
        cloud_out->push_back(cloud_in->points[i]);
    
    // Perform voxelgrid filtering
    if (should_filter)
    {
      // == Voxel grid filter
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

      pcl::VoxelGrid<pcl::PointXYZRGB> vox_sor;
      vox_sor.setInputCloud (cloud_out);
      vox_sor.setLeafSize (profile_grid_res, profile_grid_res, profile_grid_res);
      vox_sor.filter (*voxel_filtered);
      
      cloud_out = voxel_filtered;


      // == Statistical outlier removal
      /*
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr stat_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
      
      pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> stat_sor;
      stat_sor.setInputCloud (voxel_filtered);
      stat_sor.setMeanK (10);
      stat_sor.setStddevMulThresh (0.5);
      stat_sor.filter (*stat_filtered);

      cloud_out = stat_filtered;
      */
    }
    
    if (isDebug && isDebugContinuousStates){
        std::cout << cc.blue << "Number of points in global map: " << cloud_out->points.size() << "\n" << cc.reset;
    }
}
