/* Author: Abdullah Abduldayem
 * Derived from coverage_quantification.cpp
 */

// catkin_make -DCATKIN_WHITELIST_PACKAGES="aircraft_inspection" && rosrun aircraft_inspection wall_follower

#include "nbv_exploration/mapping_module.h"
#include <mutex>

std::mutex mutex_profile;
std::mutex mutex_octo;
std::mutex mutex_depth_callback;

MappingModule::MappingModule(const ros::NodeHandle& nh_, const ros::NodeHandle& nh_private_)
  : cloud_ptr_rgbd_ (new PointCloudXYZ),
    cloud_ptr_profile_ (new PointCloudXYZ),
    cloud_ptr_profile_symmetry_ (new PointCloudXYZ),
    octree_(NULL),
    counter_(0),
    nh(nh_),
    nh_private(nh_private_),
    depth1_sub(NULL),
    depth2_sub(NULL),
    sync(NULL)
{
  // >>>>>>>>>>>>>>>>>
  // Initialization
  // >>>>>>>>>>>>>>>>>
  initializeParameters();
  initializeTopicHandlers();

  // Depth cloud correction
  ros::param::param("~camera_range_max", camera_range_max, 8.0);
  ros::param::param("~camera_range_min", camera_range_min, 0.5);
  ros::param::param("~camera_width_px", camera_width_px, 640);
  ros::param::param("~camera_height_px", camera_height_px, 480);
  ros::param::param("~camera_fov_vertical", camera_fov_vertical, 45.0);
  ros::param::param("~camera_fov_horizontal", camera_fov_horizontal, 60.0);

  ros::param::param("~camera_range_upper_adjustment", camera_range_upper_adjustment, 0.1);
  createMaxRangeCloud();

  // >>>>>>>>>>>>>>>>>
  // Main function
  // >>>>>>>>>>>>>>>>>
  std::cout << "[Mapping] " << cc.magenta << "Begin Sensing\n" << cc.reset;
  std::cout << "Listening for the following topics: \n";
  std::cout << "\t" << topic_depth_ << "\n";
  std::cout << "\t" << topic_depth2_ << "\n";
  std::cout << "\t" << topic_scan_in_ << "\n";
  std::cout << "\n";
}

MappingModule::~MappingModule()
{
  if(depth1_sub)
    delete depth1_sub;
  if(depth2_sub)
    delete depth2_sub;
  if(sync)
    delete sync;
}

void MappingModule::run()
{
  // >>>>>>>>>>>>>>>>>
  // Main loop
  // >>>>>>>>>>>>>>>>>
  int i=0;
  ros::Rate rate(30);
  while(ros::ok())
  {
    //Publish once a second, but update 30 times a second
    if (i-- <= 0)
    {
      i=30;

      // Publish profile Cloud
      if (cloud_ptr_profile_ && pub_scan_cloud_.getNumSubscribers() > 0)
      {
        sensor_msgs::PointCloud2 cloud_msg;
        mutex_profile.lock();
        pcl::toROSMsg(*cloud_ptr_profile_, cloud_msg);
        mutex_profile.unlock();
        cloud_msg.header.frame_id = "world";
        cloud_msg.header.stamp = ros::Time::now();
        pub_scan_cloud_.publish(cloud_msg);
      }

      // Publish RGB-D Cloud
      if (cloud_ptr_rgbd_ && pub_rgbd_cloud_.getNumSubscribers() > 0)
      {
        sensor_msgs::PointCloud2 cloud_msg;
        mutex_profile.lock();
        pcl::toROSMsg(*cloud_ptr_rgbd_, cloud_msg);
        mutex_profile.unlock();
        cloud_msg.header.frame_id = "world";
        cloud_msg.header.stamp = ros::Time::now();
        pub_rgbd_cloud_.publish(cloud_msg);
      }

      // Publish octomap
      if (octree_ && pub_tree_.getNumSubscribers() > 0)
      {
        octomap_msgs::Octomap msg;
        octomap_msgs::fullMapToMsg (*octree_, msg);

        msg.header.frame_id = "world";
        msg.header.stamp = ros::Time::now();
        pub_tree_.publish(msg);
      }

      if (octree_prediction_ && pub_tree_prediction_.getNumSubscribers() > 0)
      {
        octomap_msgs::Octomap msg;
        octomap_msgs::fullMapToMsg (*octree_prediction_, msg);

        msg.header.frame_id = "world";
        msg.header.stamp = ros::Time::now();
        pub_tree_prediction_.publish(msg);
      }
    }

    // Sleep
    ros::spinOnce();
    rate.sleep();
  }
}

void MappingModule::addPredictedPointCloudToTree(octomap::OcTree* octree_in, PointCloudXYZ cloud_in)
{
  /*
   * Add prediction to final map
   */

  for (int j=0; j<cloud_in.points.size(); j++)
  {
    // Convert from PCL point to Octomap point
    octomap::point3d p (
          cloud_in.points[j].x,
          cloud_in.points[j].y,
          cloud_in.points[j].z);

    // Find key corresponding to this position
    octomap::OcTreeKey key;
    if ( octree_in->coordToKeyChecked(p, key) )
    {
      octomap::OcTreeNode* node = octree_in->search(key);

      if (node == NULL)
      {
        // Node doesn't exist, create it
        node = octree_in->updateNode(key, false);
      }
      else
      {
        // Check if the current cell is occupied or free
        double p = node->getLogOdds();

        if (p >= predicted_occupancy_value_ || p <= -predicted_occupancy_value_)
          continue; // Cell is occupied/free, skip it


      }

      // Cell is neither occupied nor free, update it with the predicted value
      if (is_integrating_prediction_)
        node->setValue(predicted_occupancy_value_); //Whatever value the user set in the settings
      else
        node->setValue(5.0f); //Fully occupied
    }
  }
}

void MappingModule::addPointCloudToTree(octomap::OcTree* octree_in, PointCloudXYZ cloud_in, octomap::point3d sensor_origin, octomap::point3d sensor_dir, double range, bool isPlanar)
{
  // Lock the octree
  mutex_octo.lock();

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
    computeTreeUpdatePlanar(octree_in, ocCloud, sensor_origin, sensor_dir, free_cells, occupied_cells, range);
  else
    octree_in->computeUpdate(ocCloud, sensor_origin, free_cells, occupied_cells, range);

  // insert data into tree using continuous probabilities -----------------------
  for (octomap::KeySet::iterator it = free_cells.begin(); it != free_cells.end(); ++it)
  {
    octree_in->updateNode(*it, false);
  }

  for (octomap::KeySet::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it)
  {
    octomap::point3d p = octree_->keyToCoord(*it);
    if (p.z() <= sensor_data_min_height_)
    {
      continue;
    }
    else
    {
      octree_in->updateNode(*it, true);
    }
  }

  mutex_octo.unlock();
}

void MappingModule::addPointCloudToPointCloud(const PointCloudXYZ::Ptr& cloud_in, PointCloudXYZ::Ptr& cloud_out) {
  if (is_debugging_)
  {
      std::cout << "[Mapping] " << cc.green << "MAPPING\n" << cc.reset;
  }

  // Initialize cloud if not already done so
  if (!cloud_out)
  {
      cloud_out = cloud_in;
      return;
  }

  // Only add points that meet the min height requirement
  for (int i=0; i<cloud_in->points.size(); i++)
    if (cloud_in->points[i].z >= sensor_data_min_height_)
      cloud_out->push_back(cloud_in->points[i]);
}

void MappingModule::addPointCloudToPointCloud(const PointCloudXYZ::Ptr& cloud_in, PointCloudXYZ::Ptr& cloud_out, double filter_res) {
  // Since this function typically operates on the profile, we will lock the profile
  // This prevents race condition if two callbacks occur simultaneously
  mutex_profile.lock();

  // Add two point clouds
  addPointCloudToPointCloud(cloud_in, cloud_out);

  // Perform voxelgrid filtering
  PointCloudXYZ::Ptr voxel_filtered(new PointCloudXYZ);

  pcl::VoxelGrid<PointXYZ> vox_sor;
  vox_sor.setInputCloud (cloud_out);
  vox_sor.setLeafSize (filter_res, filter_res, filter_res);
  vox_sor.filter (*voxel_filtered);

  cloud_out = voxel_filtered;

  if (is_debugging_)
  {
    std::cout << "[Mapping] " << cc.blue << "Number of points in filtered map: " << cloud_out->points.size() << "\n" << cc.reset;
  }

  mutex_profile.unlock();
}


void MappingModule::callbackScan(const sensor_msgs::LaserScan& laser_msg){
  if (is_debugging_)
  {
    std::cout << "[Mapping] " << cc.magenta << "Scan readings\n" << cc.reset;
  }

  if (!is_scanning_)
    return;

  // == Get transform
  tf::StampedTransform transform;
  Eigen::Matrix4d tf_eigen;

  try{
    ros::Time tf_time = laser_msg.header.stamp;

    tf_listener_->lookupTransform("world", laser_msg.header.frame_id, tf_time, transform);
    tf_eigen = pose_conversion::convertStampedTransform2Matrix4d(transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
    return;
  }

  // == Add scan to point cloud
  PointCloudXYZ::Ptr scan_ptr(new PointCloudXYZ);
  PointCloudXYZ::Ptr scan_far_ptr(new PointCloudXYZ);

  int steps = (laser_msg.angle_max - laser_msg.angle_min)/laser_msg.angle_increment;
  float step_size = (laser_msg.angle_max - laser_msg.angle_min)/steps;

  laser_range_ = laser_msg.range_max;

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
    PointXYZ p;

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
  PointCloudXYZ::Ptr scan_ptr_filtered(new PointCloudXYZ);

  for (int i=0; i<scan_ptr->points.size(); i++)
  {
    if (scan_ptr->points[i].z > sensor_data_min_height_)
    {
      scan_ptr_filtered->push_back(scan_ptr->points[i]);
    }
  }

  // == Add to global map
  addPointCloudToPointCloud(scan_ptr_filtered, cloud_ptr_profile_, profile_grid_res_);

  // == Add to vectors for later octomap processing
  octomap::point3d sensor_origin (transform.getOrigin().x(),
                                  transform.getOrigin().y(),
                                  transform.getOrigin().z());

  octomap::point3d sensor_dir =  pose_conversion::getOctomapDirectionVectorFromTransform(transform);

  if (is_filling_octomap_)
  {
    if (is_filling_octomap_continuously_)
    {
      addPointCloudToTree(octree_, *scan_ptr + *scan_far_ptr, sensor_origin, sensor_dir, laser_range_);
    }
    else
    {
      pose_vec_.push_back(sensor_origin);
      dir_vec_.push_back(sensor_dir);
      scan_vec_.push_back(*scan_ptr + *scan_far_ptr);
    }
  }
}

void MappingModule::createMaxRangeCloud()
{
  cloud_max_range_ = new PointXYZ[camera_width_px*camera_height_px];

  double r = camera_range_max + camera_range_upper_adjustment;

  double x_step = tan(camera_fov_horizontal/2*M_PI/180)/(camera_width_px/2);
  double y_step = tan(camera_fov_vertical/2*M_PI/180)/(camera_height_px/2);

  for (int j=0; j<camera_height_px; j++)
  {
    for (int i=0; i<camera_width_px; i++)
    {
      PointXYZ p;

      p.x = (i-camera_width_px/2+0.5)*x_step*r;
      p.y = (j-camera_height_px/2+0.5)*y_step*r;
      p.z = r;

      cloud_max_range_[j*camera_width_px + i] = p;
    }
  }
}

PointCloudXYZ::Ptr MappingModule::correctDepth(const sensor_msgs::PointCloud2& input_msg)
{
  /*
   * Points near the max and min range are pushed outside the range
   * This way, they can be ignored
   *
   * IMPORTANT: NANs are considered to be beyond the max range, and NOT before the earlier range.
   * Violating this condition will result in occupied areas considered as "free"
   */

  // == Convert to pcl pointcloud
  PointCloudXYZ cloud;
  PointCloudXYZ::Ptr cloud_ptr;

  pcl::fromROSMsg (input_msg, cloud);
  cloud_ptr = cloud.makeShared();

  // == Check the number of points is correct
  if (cloud.points.size() != camera_width_px*camera_height_px)
  {
    ROS_ERROR("Number of points in cloud (%d) do not match supplied inputs (%dx%d px)", (int) cloud.points.size(), camera_width_px, camera_height_px);
    return NULL;
  }

  // == Create points slighly out of range if needed
  for (int i=0; i<cloud_ptr->points.size(); i++)
  {
    if (std::isfinite(cloud_ptr->points[i].x) &&
        std::isfinite(cloud_ptr->points[i].y) &&
        std::isfinite(cloud_ptr->points[i].z) )
    {
      // Point is valid, continue
      continue;
    }

    int x_px = i%camera_width_px;
    int y_px = i/camera_width_px;

    cloud_ptr->points[i] = cloud_max_range_[y_px*camera_width_px + x_px];
  }

  return cloud_ptr;
}


void MappingModule::processDepth(const sensor_msgs::PointCloud2& cloud_msg)
{
  std::cout << "[Mapping] " << cc.green << "Processing Depth\n" << cc.reset;
  timer.start("[MappingModule]callbackDepth-conversion");
  if(cloud_msg.data.size() == 0)
  {
    std::cout << "[Mapping] " << cc.red << "Cloud Empty, Skipping\n" << cc.reset;
    return;
  }
  else
    std::cout << "[Mapping] " << cc.green << "Point Size:"<<cloud_msg.data.size()<<"\n" << cc.reset;

  PointCloudXYZ::Ptr cloud_raw_ptr;
  cloud_raw_ptr = correctDepth(cloud_msg);

  if (cloud_raw_ptr == NULL)
    return;

  // == Create cloud without points that are too far
  PointCloudXYZ::Ptr cloud_distance_ptr(new PointCloudXYZ);

  for (int i=0; i<cloud_raw_ptr->points.size(); i++)
  {
    if (cloud_raw_ptr->points[i].z <= max_rgbd_range_)
      cloud_distance_ptr->push_back(cloud_raw_ptr->points[i]);
  }

  // == Transform
  tf::StampedTransform transform;
  while (true)
  {
    try{
      // Listen for transform
      //tf_listener_->lookupTransform("world", cloud_msg->header.frame_id, cloud_msg->header.stamp, transform);

      // Wait for the absolute latest position, to ensure we have the correct position
      // (Latest time needed for teleporting sensor)
      //ros::Time now = ros::Time::now();
      tf_listener_->waitForTransform("world", cloud_msg.header.frame_id, cloud_msg.header.stamp, ros::Duration(1.0), ros::Duration(0.01));
      tf_listener_->lookupTransform ("world", cloud_msg.header.frame_id, cloud_msg.header.stamp, transform);
      break;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(0.1).sleep();
      return;
    }
  }

  // std::cout << "Depth Topic: " << cloud_msg.header.frame_id << "\n";
  // std::cout << "Depth stamp: " << cloud_msg.header.stamp << "\n";
  // std::cout << "Transform stamp: " << transform.stamp_ << "\n";
  // std::cout << "MAP: Y: " << transform.getOrigin().getY() << "\n";

  // == Convert tf:Transform to Eigen::Matrix4d
  Eigen::Matrix4d tf_eigen = pose_conversion::convertStampedTransform2Matrix4d(transform);

  // == Transform point cloud to global frame
  pcl::transformPointCloud(*cloud_raw_ptr, *cloud_raw_ptr, tf_eigen);
  pcl::transformPointCloud(*cloud_distance_ptr, *cloud_distance_ptr, tf_eigen);

  timer.stop("[MappingModule]callbackDepth-conversion");

  // == Add filtered to final cloud
  addPointCloudToPointCloud(cloud_distance_ptr, cloud_ptr_rgbd_, depth_grid_res_);

  // == Update octomap
  timer.start("[MappingModule]callbackDepth-updateOcto");
  octomap::point3d origin (transform.getOrigin().x(),
                           transform.getOrigin().y(),
                           transform.getOrigin().z());
  octomap::point3d sensor_dir = pose_conversion::getOctomapDirectionVectorFromTransform(transform);

  addPointCloudToTree(octree_, *cloud_raw_ptr, origin, sensor_dir, max_rgbd_range_, true);
  timer.stop("[MappingModule]callbackDepth-updateOcto");

  // == Update prediction, if necessary
  // If prediction is integrated into map, don't do anything
  if (is_checking_symmetry_ && !is_integrating_prediction_)
  {
    timer.start("[MappingModule]callbackDepth-updatePrediction");
    std::cout << cc.yellow << "[Mapping] " <<" HERE --->>>\n";fflush(stdout);
    updatePrediction(octree_prediction_, *cloud_raw_ptr, origin, sensor_dir, max_rgbd_range_, true);
    std::cout << cc.yellow << "[Mapping] " << " HERE <<<---\n";fflush(stdout);
    timer.stop("[MappingModule]callbackDepth-updatePrediction");
  }

  // == Update density map
  updateVoxelDensities(cloud_distance_ptr);
  std::cout << "[Mapping] " << cc.green << "Done Processing Depth\n" << cc.reset;
}

void MappingModule::callbackDepthSync(const sensor_msgs::PointCloud2ConstPtr& cloud_msg1, const sensor_msgs::PointCloud2ConstPtr& cloud_msg2)
{
    timer.start("[MappingModule]callbackDepthSync");

    if (is_debugging_)
    {
      std::cout << "[Mapping] " << cc.green << "Depth sensing\n" << cc.reset;
    }

    if (!getCameraData)
    {
      if (is_debugging_)
        std::cout << "[Mapping]" << cc.yellow << "Not allowed to process depth data yet\n";

      return;
    }

    mutex_depth_callback.lock();
    processDepth(*cloud_msg1);
    processDepth(*cloud_msg2);

    // == Done updating
    camera_done_flags_ |= 0x03;
    mutex_depth_callback.unlock();

    timer.stop("[MappingModule]callbackDepthSync");
}

void MappingModule::callbackDepth(const sensor_msgs::PointCloud2& cloud_msg)
{
    timer.start("[MappingModule]callbackDepth");

    if (is_debugging_)
    {
      std::cout << "[Mapping] " << cc.green << "Depth sensing\n" << cc.reset;
    }

    if (!getCameraData || (camera_done_flags_ & 0x01))
    {
      if (is_debugging_)
        std::cout << "[Mapping]" << cc.yellow << "Not allowed to process depth data yet\n";

      return;
    }

    mutex_depth_callback.lock();
    processDepth(cloud_msg);

    // == Done updating
    camera_done_flags_ |= 0x01;
    mutex_depth_callback.unlock();

    timer.stop("[MappingModule]callbackDepth");
}

void MappingModule::callbackDepth2(const sensor_msgs::PointCloud2& cloud_msg)
{
    timer.start("[MappingModule]callbackDepth2");

    if (is_debugging_)
    {
      std::cout << "[Mapping] " << cc.green << "Depth sensing\n" << cc.reset;
    }

    if (!getCameraData || (camera_done_flags_ & 0x02))
    {
      if (is_debugging_)
        std::cout << "[Mapping]" << cc.yellow << "Not allowed to process depth data yet\n";

      return;
    }

    mutex_depth_callback.lock();
    processDepth(cloud_msg);

    // == Done updating
    camera_done_flags_ |= 0x02;
    mutex_depth_callback.unlock();

    timer.stop("[MappingModule]callbackDepth2");
}

bool MappingModule::commandGetCameraData()
{
  getCameraData      = true;
  camera_done_flags_ = 0;

  timer.start("[MappingModule]commandGetCameraData-waiting");
  while(camera_done_flags_ != all_done_flags_ )
  {
    std::cout << "[Mapping] " << cc.magenta << "Waiting for camera data\n" << cc.reset;
    ros::spinOnce();
    ros::Rate(1000).sleep();
  }
  getCameraData = false;

  timer.stop("[MappingModule]commandGetCameraData-waiting");

  if (getCameraData)
  {
    std::cout << "[Mapping] " << cc.magenta << "Could not get camera data\n" << cc.reset;
  }

  // Save final map ever 'x' iterations
  counter_++;
  counter_ %= save_iterations_;
  if (counter_ == 0)
    commandFinalMapSave();

  return true;
}

bool MappingModule::commandFinalMapLoad()
{
  // Point cloud
  std::cout << "[Mapping] " << "Reading " << filename_pcl_save_state_ << "\n";
  if (pcl::io::loadPCDFile<PointXYZ> (filename_pcl_save_state_, *cloud_ptr_rgbd_) == -1) //* load the file
  {
    std::cout << "[Mapping] " << cc.red << "ERROR: Failed to load point cloud: Could not read file " << filename_pcl_ << ".Exiting node.\n" << cc.reset;
    return false;
  }

  return true;
}

bool MappingModule::commandFinalMapSave()
{
  timer.start("[MappingModule]commandFinalMapSave");

  std::cout << "[Mapping] " << cc.green << "Saving final maps\n" << cc.reset;

  // Point cloud
  if (!cloud_ptr_rgbd_)
  {
    std::cout << "[Mapping] " << cc.red << "ERROR: No point cloud data available. Exiting node.\n" << cc.reset;
    return false;
  }
  pcl::io::savePCDFileASCII (filename_pcl_final_, *cloud_ptr_rgbd_);

  if (is_debug_save_state_)
    pcl::io::savePCDFileASCII (filename_pcl_save_state_, *cloud_ptr_rgbd_);

  // Octree
  if (is_filling_octomap_)
  {
    if (!octree_)
    {
      std::cout << "[Mapping] " << cc.red << "ERROR: No octomap data available. Exiting node.\n" << cc.reset;
      return false;
    }
    if (!octree_->write(filename_octree_final_))
    {
      std::cout << "[Mapping] " << cc.red << "ERROR: Failed to save octomap data to " << filename_octree_ << ". Exiting node.\n" << cc.reset;
      return false;
    }
  }

  std::cout << "[Mapping] " << cc.blue << "Successfully saved map\n" << cc.reset;
  timer.stop("[MappingModule]commandFinalMapSave");
  return true;
}

bool MappingModule::commandProfileLoad()
{
  // Point cloud
  std::cout << "[Mapping] " << "Reading " << filename_pcl_ << "\n";
  if (pcl::io::loadPCDFile<PointXYZ> (filename_pcl_, *cloud_ptr_profile_) == -1) //* load the file
  {
    std::cout << "[Mapping] " << cc.red << "ERROR: Failed to load point cloud: Could not read file " << filename_pcl_ << ".Exiting node.\n" << cc.reset;
    return false;
  }

  // Initialize cloud_ptr_rgbd_ with profile data
  if (is_debug_load_state_)
    commandFinalMapLoad();
  else
    copyPointCloud(*cloud_ptr_profile_, *cloud_ptr_rgbd_);

  // Octree
  if (is_debug_load_state_)
  {
    std::cout << "[Mapping] " << "Reading " << filename_octree_final_ << "\n";

    octomap::AbstractOcTree* temp_tree = octomap::AbstractOcTree::read(filename_octree_final_);
    if(temp_tree)
    { // read error returns NULL
      octree_ = dynamic_cast<octomap::OcTree*>(temp_tree);
      octree_->setOccupancyThres( octree_thresh_ );

      if (!octree_)
      {
        std::cout << "[Mapping] " << cc.red << "ERROR: Failed to load octomap: Type cast failed .Exiting node.\n" << cc.reset;
        return false;
      }
    }
    else
    {
      std::cout << "[Mapping] " << cc.red << "ERROR: Failed to load octomap: Could not read file " << filename_octree_ << ". Exiting node.\n" << cc.reset;
      return false;
    }
  }
  else if (is_filling_octomap_)
  {
    std::cout << "[Mapping] " << "Reading " << filename_octree_ << "\n";

    octomap::AbstractOcTree* temp_tree = octomap::AbstractOcTree::read(filename_octree_);
    if(temp_tree)
    { // read error returns NULL
      octree_ = dynamic_cast<octomap::OcTree*>(temp_tree);
      octree_->setOccupancyThres( octree_thresh_ );

      if (!octree_)
      {
        std::cout << "[Mapping] " << cc.red << "ERROR: Failed to load octomap: Type cast failed .Exiting node.\n" << cc.reset;
        return false;
      }
    }
    else
    {
      std::cout << "[Mapping] " << cc.red << "ERROR: Failed to load octomap: Could not read file " << filename_octree_ << ". Exiting node.\n" << cc.reset;
      return false;
    }
  }

  // Symmetry
  if (is_checking_symmetry_)
  {
    std::cout << "[Mapping] " << "Reading " << filename_pcl_symmetry_ << "\n";

    if (pcl::io::loadPCDFile<PointXYZ> (filename_pcl_symmetry_, *cloud_ptr_profile_symmetry_) == -1) //* load the file
    {
      std::cout << "[Mapping] " << cc.red << "ERROR: Failed to load point cloud: Could not read file " << filename_pcl_symmetry_ << ".Exiting node.\n" << cc.reset;
      return false;
    }

    // Populate octree with symmetry data
    octree_prediction_ = new octomap::OcTree (octree_res_);
    octree_prediction_->setBBXMin( bound_min_ );
    octree_prediction_->setBBXMax( bound_max_ );

    if (is_integrating_prediction_)
      addPredictedPointCloudToTree(octree_, *cloud_ptr_profile_symmetry_);
    else
      addPredictedPointCloudToTree(octree_prediction_, *cloud_ptr_profile_symmetry_);
  }

  std::cout << "[Mapping] " << cc.green << "Successfully loaded maps\n" << cc.reset;
  return true;
}

bool MappingModule::commandProfileSave()
{
  std::cout << "[Mapping] " << cc.green << "Saving maps\n" << cc.reset;

  // Point cloud
  if (!cloud_ptr_profile_)
  {
    std::cout << "[Mapping] " << cc.red << "ERROR: No point cloud data available. Exiting node.\n" << cc.reset;
    return false;
  }
  pcl::io::savePCDFileASCII (filename_pcl_, *cloud_ptr_profile_);


  if (is_checking_symmetry_)
  {
    if (!cloud_ptr_profile_symmetry_)
    {
      std::cout << "[Mapping] " << cc.red << "ERROR: No symmetry data available. Exiting node.\n" << cc.reset;
      return false;
    }

    pcl::io::savePCDFileASCII (filename_pcl_symmetry_, *cloud_ptr_profile_symmetry_);
  }

  // Octree
  if (is_filling_octomap_)
  {
    if (!octree_)
    {
      std::cout << "[Mapping] " << cc.red << "ERROR: No octomap data available. Exiting node.\n" << cc.reset;
      return false;
    }
    if (!octree_->write(filename_octree_))
    {
      std::cout << "[Mapping] " << cc.red << "ERROR: Failed to save octomap data to " << filename_octree_ << ". Exiting node.\n" << cc.reset;
      return false;
    }
  }

  std::cout << "[Mapping] " << cc.green << "Successfully saved map\n" << cc.reset;
  return true;
}

bool MappingModule::commandProfilingStart()
{
  std::cout << "[Mapping] " << cc.green << "Started profiling\n" << cc.reset;

  sub_scan_ = nh.subscribe(topic_scan_in_, 10, &MappingModule::callbackScan, this);

  if (!octree_)
  {
    octree_ = new octomap::OcTree (octree_res_);
    octree_->setBBXMin( bound_min_ );
    octree_->setBBXMax( bound_max_ );
    octree_->setOccupancyThres( octree_thresh_ );
  }

  if (!octree_prediction_)
  {
    octree_prediction_ = new octomap::OcTree (octree_res_);
    octree_prediction_->setBBXMin( bound_min_ );
    octree_prediction_->setBBXMax( bound_max_ );
  }

  return true;
}

bool MappingModule::commandProfilingStop()
{
  sub_scan_.shutdown();

  std::cout << "[Mapping] " << cc.green << "Done profiling\n" << cc.reset;
  is_scanning_ = false;

  // Initialize cloud_ptr_rgbd_ with profile data
  copyPointCloud(*cloud_ptr_profile_, *cloud_ptr_rgbd_);

  if (skip_load_map_)
    return true;

  if (is_checking_symmetry_)
  {
    SymmetryDetector* sym_det = new SymmetryDetector();
    sym_det->setInputCloud(cloud_ptr_profile_);
    sym_det->run();
    sym_det->getOutputCloud(cloud_ptr_profile_symmetry_);

    // Populate octree with symmetry data
    addPredictedPointCloudToTree(octree_prediction_, *cloud_ptr_profile_symmetry_);
  }

  return true;
}

bool MappingModule::commandScanningStart()
{
  std::cout << "[Mapping] " << cc.green << "Started scanning\n" << cc.reset;
  is_scanning_ = true;
  return true;
}

bool MappingModule::commandScanningStop()
{
  std::cout << "[Mapping] " << cc.green << "Stop scanning\n" << cc.reset;
  is_scanning_ = false;

  if (is_filling_octomap_ && !is_filling_octomap_continuously_)
  {
    std::cout << "[Mapping] " << cc.green << "Processing " << scan_vec_.size() << " scans...\n" << cc.reset;
    processScans();
  }

  return true;
}

void MappingModule::computeTreeUpdatePlanar(octomap::OcTree* octree_in, const octomap::Pointcloud& scan, const octomap::point3d& origin, octomap::point3d& sensor_dir,
                      octomap::KeySet& free_cells, octomap::KeySet& occupied_cells,
                      double maxrange)
{
  /*
   * Based on the implimentation of computeUpdate in http://octomap.github.io/octomap/doc/OccupancyOcTreeBase_8hxx_source.html
   *
   * This method does not use "max_rgbd_range_" as a radial/Eucledian distance (as in the original implimentation)
   * Instead, it is used to evaluate the perpendicular distance to the far plane of the camera
   * @todo: bbx limit
   */

  sensor_dir.normalize();
  bool lazy_eval = false;
  bool use_ray_skipping = false;
  if (camera_width_px_*camera_height_px_ == scan.size() &&
      (ray_skipping_vertical_ != 1 || ray_skipping_horizontal_ != 1))
  {
    use_ray_skipping = true;
    std::cout << "[Mapping] " << "Ray skipping\n";
  }
  std::cout <<cc.red << "[Mapping] computeTreeUpdatePlanar 1\n";fflush(stdout);
  // create as many KeyRays as there are OMP_THREADS defined,
  // one buffer for each thread
  std::vector<octomap::KeyRay> keyrays;
  /*
  std::cout << "[Mapping] computeTreeUpdatePlanar 3 KeyRays:"<<keyrays.size()<< cc.red<<"\n";fflush(stdout);
  #ifdef _OPENMP
    std::cout << "[Mapping] USING OPENMP\n"<<cc.red;fflush(stdout);
    #pragma omp parallel
    #pragma omp critical
    std::cout << "[Mapping] USING OPENMP 2\n"<<cc.red;fflush(stdout);
    {
      if (omp_get_thread_num() == 0){
        std::cout << "[Mapping] USING OPENMP 3\n"<<cc.red;fflush(stdout);
        keyrays.resize(omp_get_num_threads());
        std::cout << "[Mapping] USING OPENMP 4\n"<<cc.red;fflush(stdout);
      }

    } // end critical
  #else
    std::cout << "[Mapping] NOT USING OPENMP\n"<<cc.red;fflush(stdout);
    keyrays.resize(1);
  #endif
  */
  keyrays.resize(omp_get_max_threads());
  std::cout <<cc.red << "[Mapping] computeTreeUpdatePlanar 2c\n";fflush(stdout);

  //std::cout << "[Mapping] computeTreeUpdatePlanar Pre 4, scan size:"<<cc.red<< scan.size() <<" camera width px:"<<camera_width_px_<<" octree size:"<<octree_in->size()<<" keyRays:"<<keyrays.size()<<"\n";fflush(stdout);
  int num_threads = keyrays.size();
  std::vector<octomap::KeySet> free_cells_threaded(num_threads);
  std::vector<octomap::KeySet> occupied_cells_threaded(num_threads);
  std::cout <<cc.red << "[Mapping] computeTreeUpdatePlanar 3 Max Number of threads:"<<omp_get_max_threads()<<"\n";fflush(stdout);

  #ifdef _OPENMP
  omp_set_num_threads(num_threads);
  #pragma omp parallel for schedule(guided)
  #endif
  for (int i = 0; i < (int)scan.size(); ++i)
  {
    if (use_ray_skipping)
    {
      int ix = i%camera_width_px_;
      int iy = i/camera_width_px_;

      if (ix % ray_skipping_vertical_ != 0 || iy % ray_skipping_horizontal_ != 0)
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
      if (octree_in->computeRayKeys(origin, p, *keyray))
      {
        free_cells_threaded[threadIdx].insert(keyray->begin(), keyray->end());
      }
      // occupied endpoint
      octomap::OcTreeKey key;
      if (octree_in->coordToKeyChecked(p, key)){
        occupied_cells_threaded[threadIdx].insert(key);
      }
    }
    else
    { // user set a maxrange and length is above
      octomap::point3d direction = (p - origin).normalized ();
      double max_rgbd_range__to_point = maxrange/sensor_dir.dot(direction);

      octomap::point3d new_end = origin + direction * (float) max_rgbd_range__to_point;
      if (octree_in->computeRayKeys(origin, new_end, *keyray)){
        free_cells_threaded[threadIdx].insert(keyray->begin(), keyray->end());
      }
    } // end if maxrange
  } // end for all points, end of parallel OMP loop

  // Merge threaded vectors
  for (int i=0; i<num_threads; i++)
  {
    free_cells.insert(free_cells_threaded[i].begin(), free_cells_threaded[i].end());
    occupied_cells.insert(occupied_cells_threaded[i].begin(), occupied_cells_threaded[i].end());
  }

  // prefer occupied cells over free ones (and make sets disjunct)
  for(octomap::KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; )
  {
    if (occupied_cells.find(*it) != occupied_cells.end())
      it = free_cells.erase(it);

    else
      ++it;
  }
}

double MappingModule::getAveragePointDensity()
{
  // Get total number of occupied voxels
  int num_occ = 0;
  int treeDepth = 16;
  for (octomap::OcTree::iterator it = octree_->begin(treeDepth), end = octree_->end(); it != end; ++it)
  {
    if ( isNodeOccupied(*it) )
      num_occ++;
  }

  return double(cloud_ptr_rgbd_->points.size())/num_occ;
}

bool MappingModule::isNodeFree(octomap::OcTreeNode node)
{
  if (node.getOccupancy() <= 1-octree_->getOccupancyThres())
    return true;

  return false;
}

bool MappingModule::isNodeOccupied(octomap::OcTreeNode node)
{
  if (node.getOccupancy() >= octree_->getOccupancyThres())
    return true;

  return false;
}

bool MappingModule::isNodeUnknown(octomap::OcTreeNode node)
{
  return !isNodeFree(node) && !isNodeOccupied(node);
}

octomap::OcTree* MappingModule::getOctomap()
{
  return octree_;
}

octomap::OcTree* MappingModule::getOctomapPredicted()
{
  return octree_prediction_;
}

PointCloudXYZ::Ptr MappingModule::getProfilePointCloud()
{
  return cloud_ptr_profile_;
}

PointCloudXYZ::Ptr MappingModule::getPointCloud()
{
  return cloud_ptr_rgbd_;
}

void MappingModule::initializeParameters()
{
  getCameraData = false;
  is_scanning_ = false;

  filename_pcl_           = "/tmp/profile_cloud.pcd";
  filename_pcl_save_state_= "/tmp/rgbd_cloud_save_state.pcd";
  filename_pcl_final_     = "/tmp/final_cloud.pcd";
  filename_pcl_symmetry_  = "/tmp/profile_cloud_symmetry.pcd";
  filename_octree_        = "/tmp/profile_octree.ot";
  filename_octree_final_  = "/tmp/final_octree.ot";

  topic_depth_        = "/nbv_exploration/depth";
  topic_depth2_       = "/nbv_exploration/depth2";
  topic_map_          = "/nbv_exploration/global_map_cloud";
  topic_scan_in_      = "/nbv_exploration/scan";
  topic_scan_out_     = "/nbv_exploration/scan_cloud";
  topic_rgbd_out_     = "/nbv_exploration/rgbd_cloud";
  topic_tree_         = "/nbv_exploration/output_tree";
  topic_tree_predicted_="/nbv_exploration/output_tree_predicted";

  ros::param::param("~debug_mapping", is_debugging_, false);
  ros::param::param("~profiling_check_symmetry", is_checking_symmetry_, true);
  ros::param::param("~profiling_skip_load_map", skip_load_map_, false);
  ros::param::param("~profiling_fill_octomap", is_filling_octomap_, true);
  ros::param::param("~profiling_fill_octomap_continuously", is_filling_octomap_continuously_, true);
  ros::param::param("~debug_load_state", is_debug_load_state_, false);
  ros::param::param("~debug_save_state", is_debug_save_state_, false);

  ros::param::param("~depth_range_max", max_rgbd_range_, 5.0);
  ros::param::param("~mapping_octree_resolution", octree_res_, 0.2);
  ros::param::param("~mapping_occupancy_threshold", octree_thresh_, 0.5);
  ros::param::param("~mapping_sensor_data_min_height_", sensor_data_min_height_, 0.5);
  ros::param::param("~mapping_voxel_grid_res_profile", profile_grid_res_, 0.1);
  ros::param::param("~mapping_voxel_grid_res_rgbd", depth_grid_res_, 0.1);
  ros::param::param("~mapping_integrate_prediction", is_integrating_prediction_, false);
  ros::param::param("~mapping_integrate_occupancy", predicted_occupancy_value_, 0.7);

  ros::param::param("~mapping_save_map_iterations", save_iterations_, 10);

  // Convert to logodds
  predicted_occupancy_value_ = octomap::logodds(predicted_occupancy_value_);

  double obj_x_min, obj_x_max, obj_y_min, obj_y_max, obj_z_min, obj_z_max;
  ros::param::param("~object_bounds_x_min", obj_x_min,-1.0);
  ros::param::param("~object_bounds_x_max", obj_x_max, 1.0);
  ros::param::param("~object_bounds_y_min", obj_y_min,-1.0);
  ros::param::param("~object_bounds_y_max", obj_y_max, 1.0);
  ros::param::param("~object_bounds_z_min", obj_z_min, 0.0);
  ros::param::param("~object_bounds_z_max", obj_z_max, 1.0);

  bound_min_ = octomap::point3d(obj_x_min, obj_y_min, obj_z_min);
  bound_max_ = octomap::point3d(obj_x_max, obj_y_max, obj_z_max);

  ros::param::param("~width_px", camera_width_px_, 640);
  ros::param::param("~height_px", camera_height_px_, 480);
  ros::param::param("~ray_skipping_vertical", ray_skipping_vertical_, 1);
  ros::param::param("~ray_skipping_horizontal", ray_skipping_horizontal_, 1);


  int camera_count;
  ros::param::param("~camera_count", camera_count, 1);

  // The following is equivalent to (2^n)-1
  // This sets the last n bits to '1'
  all_done_flags_ = (1 << camera_count) - 1;

}

void MappingModule::initializeTopicHandlers()
{
  // >>>>>>>>>>>>>>>>>
  // Subscribers / Servers
  // >>>>>>>>>>>>>>>>>

  // Sensor data

  depth1_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, topic_depth_, 1);
  depth2_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, topic_depth2_, 1);
  sync = new message_filters::Synchronizer<sync_policy>(sync_policy(10), *depth1_sub, *depth2_sub);

  //message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> sync(depth1_sub, depth2_sub, 10);
  sync->registerCallback(boost::bind(&MappingModule::callbackDepthSync, this, _1, _2));

  tf_listener_ = new tf::TransformListener();

  // >>>>>>>>>>>>>>>>>
  // Publishers
  // >>>>>>>>>>>>>>>>>
  pub_global_cloud_  = nh.advertise<sensor_msgs::PointCloud2>(topic_map_, 10);
  pub_scan_cloud_    = nh.advertise<sensor_msgs::PointCloud2>(topic_scan_out_, 10);
  pub_rgbd_cloud_    = nh.advertise<sensor_msgs::PointCloud2>(topic_rgbd_out_, 10);
  pub_tree_          = nh.advertise<octomap_msgs::Octomap>(topic_tree_, 10);
  pub_tree_prediction_ = nh.advertise<octomap_msgs::Octomap>(topic_tree_predicted_, 10);
}

void MappingModule::processScans()
{
  double t_start, t_end;
  t_start = ros::Time::now().toSec();

  int count =0;
  for (int i=scan_vec_.size()-1; i>=0; i--)
  {
    octomap::point3d sensor_origin = pose_vec_[i];
    octomap::point3d sensor_dir = dir_vec_[i];
    PointCloudXYZ scan = scan_vec_[i];

    addPointCloudToTree(octree_, scan_vec_[i], sensor_origin, sensor_dir, laser_range_);

    // == Pop
    pose_vec_.pop_back();
    dir_vec_.pop_back();
    scan_vec_.pop_back();
    count++;
  }

  // == Timing
  t_end = ros::Time::now().toSec();
  std::cout << "[Mapping] " << cc.green << "Done processing.\n" << cc.reset;
  std::cout << "   Total time: " << t_end-t_start << " sec\tTotal scan: " << count << "\t(" << (t_end-t_start)/count << " sec/scan)\n";
}

void MappingModule::updatePrediction(octomap::OcTree* octree_in, PointCloudXYZ cloud_in, octomap::point3d sensor_origin, octomap::point3d sensor_dir, double range, bool isPlanar)
{
  // Clear predictions along rays
  // Modified version of from MappingModule::addPointCloudToTree()

  if (!is_checking_symmetry_)
    return;

  octomap::Pointcloud ocCloud;
  for (int j=0; j<cloud_in.points.size(); j++)
  {
    ocCloud.push_back(cloud_in.points[j].x,
                      cloud_in.points[j].y,
                      cloud_in.points[j].z);
  }

  // == Insert point cloud based on planar (camera) or spherical (laser) scan data
  octomap::KeySet free_cells, occupied_cells;
  std::cout << "[Mapping] " << cc.yellow<<" --- D ---\n";fflush(stdout);
  if (isPlanar)
  {
   try
    {
      computeTreeUpdatePlanar(octree_in, ocCloud, sensor_origin, sensor_dir, free_cells, occupied_cells, range);
    }
    catch(...)
    {
      std::cout << boost::current_exception_diagnostic_information() << std::endl;
    }
  }
  else
  {
    octree_in->computeUpdate(ocCloud, sensor_origin, free_cells, occupied_cells, range);
  }
  std::cout << "[Mapping] " << cc.yellow<<" --- E ---\n";fflush(stdout);
  // Clear all predicted values
  for (octomap::KeySet::iterator it = free_cells.begin(); it != free_cells.end(); ++it)
    octree_in->updateNode(*it, -5.0f);

  for (octomap::KeySet::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it)
    octree_in->updateNode(*it, -5.0f);
}

int MappingModule::getDensityAtOcTreeKey(octomap::OcTreeKey key)
{
  std::map<octomap::OcTreeKey, VoxelDensity>::iterator it;
  it = voxel_densities_.find(key);

  if (it == voxel_densities_.end())
  {
    // Key not found, display error
    //printf("[ViewSelecterBase]: Invalid key, no point count retrieved\n");
    return -1;
  }

  return it->second.density;
}

void MappingModule::updateVoxelDensities()
{
  //=======
  // Fill a map with point count at each octreekey
  //=======
  voxel_densities_.clear(); // Clear old densities

  // ============
  // Create KD tree to find nearest neighbors
  // ============
  if(cloud_ptr_rgbd_->points.size()<=0)
    return;
  pcl::KdTreeFLANN<PointXYZ> kdtree;
  kdtree.setInputCloud (cloud_ptr_rgbd_);
  double search_radius = octree_->getResolution()/sqrt(2); //Encapsulates a voxel

  std::map<octomap::OcTreeKey, VoxelDensity>::iterator it;

  for (int i=0; i<cloud_ptr_rgbd_->points.size(); i++)
  {
    PointXYZ p = cloud_ptr_rgbd_->points[i];

    octomap::OcTreeKey key;
    if( !octree_->coordToKeyChecked(p.x, p.y, p.z, key) )
      continue;

    std::vector<int> pointIndicesOut;
    std::vector<float> pointRadiusSquaredDistance;
    kdtree.radiusSearch(p, search_radius, pointIndicesOut, pointRadiusSquaredDistance);

    // Get iterator for desired key
    it = voxel_densities_.find(key);
    if (it != voxel_densities_.end())
    {
      // Key found, increment it it
      it->second.count++;
      it->second.total += pointIndicesOut.size();
      it->second.density = double(it->second.total)/it->second.count;
    }
    else
    {
      // Key not found, initialize it
      VoxelDensity v;
      v.count = 1;
      v.total = pointIndicesOut.size();
      v.density = double(v.total)/v.count;

      voxel_densities_.insert(std::make_pair(key, v));
    }
  }
}


void MappingModule::updateVoxelDensities(const PointCloudXYZ::Ptr& cloud)
{
  std::map<octomap::OcTreeKey, VoxelDensity>::iterator it;
  octomap::OcTreeKey key;
  if(cloud_ptr_rgbd_->empty())
    return;
  //=======
  // Clear old values around newly collected points
  //=======
  for (int i=0; i<cloud->points.size(); i++)
  {
    PointXYZ p = cloud->points[i];
    if( !octree_->coordToKeyChecked(p.x, p.y, p.z, key) )
      continue;

    it = voxel_densities_.find(key);
    if (it != voxel_densities_.end())
    {
      // Key found, clear it
      it->second.count = 0;
      it->second.total = 0;
      it->second.density = -1;
    }
  }

  // ============
  // Create KD tree to find nearest neighbors
  // ============
  pcl::KdTreeFLANN<PointXYZ> kdtree;
  kdtree.setInputCloud (cloud_ptr_rgbd_);
  double search_radius = octree_->getResolution()/sqrt(2); //Encapsulates a voxel

  // ============
  // Update voxel densities obtained from camera reading
  // ============
  for (int i=0; i<cloud->points.size(); i++)
  {
    PointXYZ p = cloud->points[i];

    if( !octree_->coordToKeyChecked(p.x, p.y, p.z, key) )
      continue;

    std::vector<int> pointIndicesOut;
    std::vector<float> pointRadiusSquaredDistance;
    kdtree.radiusSearch(p, search_radius, pointIndicesOut, pointRadiusSquaredDistance);

    // Get iterator for desired key
    it = voxel_densities_.find(key);
    if (it != voxel_densities_.end())
    {
      // Key found, increment it it
      it->second.count++;
      it->second.total += pointIndicesOut.size();
      it->second.density = double(it->second.total)/it->second.count;
    }
    else
    {
      // Key not found, initialize it
      VoxelDensity v;
      v.count = 1;
      v.total = pointIndicesOut.size();
      v.density = double(v.total)/v.count;

      voxel_densities_.insert(std::make_pair(key, v));
    }
  }
}

void MappingModule::updateVoxelNormals()
{
  //=======
  // Fill a map with point count at each octreekey
  //=======
  voxel_normals_.clear(); // Clear old densities

  if(cloud_ptr_rgbd_->points.size()<=0)
    return;

  // ============
  // Compute normals
  // ============
  PointCloudN::Ptr cloud_normals (new PointCloudN);
  pcl::search::KdTree<PointXYZ>::Ptr tree (new pcl::search::KdTree<PointXYZ> ());
  pcl::NormalEstimation<PointXYZ, PointN> norm_est;

  norm_est.setSearchMethod (tree);
  norm_est.setInputCloud (cloud_ptr_rgbd_);
  norm_est.setKSearch (20);
  norm_est.compute (*cloud_normals);

  // ===========
  // Generate histogram
  // ===========
  for (int i=0; i<cloud_normals->points.size(); i++)
  {
    PointN p = cloud_normals->points[i];
    PointXYZ p2 = cloud_ptr_rgbd_->points[i];

    octomap::OcTreeKey key;
    if( !octree_->coordToKeyChecked(p2.x, p2.y, p2.z, key) )
      continue;

    // Compute histogram
    /*
     * histo[0] = x positive
     * histo[1] = x negative
     * histo[2] = y positive
     * histo[3] = y negative
     * histo[4] = z positive
     * histo[5] = z negative
     */
    NormalHistogram h;
    h.size = 1;

    float hor_length = sqrt(p.normal[0]*p.normal[0] + p.normal[1]*p.normal[1]);
    float angle_hor = atan2(p.normal[1],p.normal[0]);
    float angle_ver = atan2(p.normal[2],hor_length);

    if (angle_ver > M_PI_4)
      h.histogram[4] = 1;
    else if (angle_ver < -M_PI_4)
      h.histogram[5] = 1;
    else if (-M_PI_4 < angle_hor && angle_hor <= M_PI_4)
      h.histogram[0] = 1;
    else if (M_PI_4 < angle_hor && angle_hor <= 3*M_PI_4)
      h.histogram[2] = 1;
    else if (-3*M_PI_4 < angle_hor && angle_hor <= -M_PI_4)
      h.histogram[3] = 1;
    else
      h.histogram[1] = 1;

    // Get iterator for desired key
    std::map<octomap::OcTreeKey, NormalHistogram>::iterator it;
    it = voxel_normals_.find(key);
    if (it != voxel_normals_.end())
    {
      // Key found, increment it
      it->second += h;
    }
    else
    {
      // Key not found, insert it
      voxel_normals_.insert(std::make_pair(key, h));
    }
  }
}

NormalHistogram MappingModule::getNormalHistogramAtOcTreeKey(octomap::OcTreeKey key)
{
  std::map<octomap::OcTreeKey, NormalHistogram>::iterator it;
  it = voxel_normals_.find(key);

  if (it == voxel_normals_.end())
  {
    // Key not found, display error
    //printf("[ViewSelecterBase]: Invalid key, no point count retrieved\n");
    NormalHistogram invalid_histo;
    invalid_histo.histogram[0] = -1;
    return invalid_histo;
  }

  return it->second;
}
