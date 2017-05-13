/* Author: Abdullah Abduldayem
 * Derived from coverage_quantification.cpp
 */

// catkin_make -DCATKIN_WHITELIST_PACKAGES="aircraft_inspection" && rosrun aircraft_inspection wall_follower

#include "nbv_exploration/mapping_module.h"

MappingModule::MappingModule()
  : cloud_ptr_rgbd_ (new PointCloudXYZ),
    cloud_ptr_profile_ (new PointCloudXYZ),
    cloud_ptr_profile_symmetry_ (new PointCloudXYZ),
    octree_(NULL)
{
  // >>>>>>>>>>>>>>>>>
  // Initialization
  // >>>>>>>>>>>>>>>>>
  initializeParameters();
  initializeTopicHandlers();

  // >>>>>>>>>>>>>>>>>
  // Main function
  // >>>>>>>>>>>>>>>>>
  std::cout << "[Mapping] " << cc.magenta << "Begin Sensing\n" << cc.reset;
  std::cout << "Listening for the following topics: \n";
  std::cout << "\t" << topic_depth_ << "\n";
  std::cout << "\t" << topic_scan_in_ << "\n";
  std::cout << "\n";
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
        pcl::toROSMsg(*cloud_ptr_profile_, cloud_msg);
        cloud_msg.header.frame_id = "world";
        cloud_msg.header.stamp = ros::Time::now();
        pub_scan_cloud_.publish(cloud_msg);
      }

      // Publish RGB-D Cloud
      if (cloud_ptr_rgbd_ && pub_rgbd_cloud_.getNumSubscribers() > 0)
      {
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud_ptr_rgbd_, cloud_msg);
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

void MappingModule::addPointCloudToTree(octomap::OcTree* octree_in, PointCloudXYZ cloud_in)
{


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
      node->setValue(predicted_occupancy_value_);
    }
  }
}

void MappingModule::addPointCloudToTree(octomap::OcTree* octree_in, PointCloudXYZ cloud_in, octomap::point3d sensor_origin, octomap::point3d sensor_dir, double range, bool isPlanar)
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


void MappingModule::callbackDepth(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    if (is_debugging_)
    {
      std::cout << "[Mapping] " << cc.green << "Depth sensing\n" << cc.reset;
    }

    if (!is_get_camera_data_)
    {
      if (is_debugging_)
        std::cout << "[Mapping]" << cc.yellow << "Not allowed to process depth data yet\n";

      return;
    }

    // == Convert to pcl pointcloud
    PointCloudXYZ cloud;
    PointCloudXYZ::Ptr cloud_raw_ptr;

    pcl::fromROSMsg (*cloud_msg, cloud);
    cloud_raw_ptr = cloud.makeShared();

    // == Create cloud without points that are too far
    PointCloudXYZ::Ptr cloud_distance_ptr(new PointCloudXYZ);

    for (int i=0; i<cloud.points.size(); i++)
      if (cloud.points[i].z <= max_rgbd_range_)
        cloud_distance_ptr->push_back(cloud.points[i]);

    // == Transform
    tf::StampedTransform transform;
    try{
      // Listen for transform
      tf_listener_->lookupTransform("world", cloud_msg->header.frame_id, cloud_msg->header.stamp, transform);
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
    addPointCloudToPointCloud(cloud_distance_ptr, cloud_ptr_rgbd_, depth_grid_res_);

    // == Update octomap
    octomap::point3d origin (transform.getOrigin().x(),
                             transform.getOrigin().y(),
                             transform.getOrigin().z());
    octomap::point3d sensor_dir = pose_conversion::getOctomapDirectionVectorFromTransform(transform);

    addPointCloudToTree(octree_, *cloud_raw_ptr, origin, sensor_dir, max_rgbd_range_, true);

    // == Update prediction, if necessary
    if (is_checking_symmetry_)
    {
      // If prediction is integrated into map, don't do anything
      if (!is_integrating_prediction_)
        updatePrediction(octree_prediction_, *cloud_raw_ptr, origin, sensor_dir, max_rgbd_range_, true);
    }

    // == Done updating
    is_get_camera_data_ = false;
}

bool MappingModule::commandGetCameraData()
{
  is_get_camera_data_ = true;
  std::cout << "[Mapping] " << cc.magenta << "Waiting for camera data\n" << cc.reset;

  for (int i=0; i<10 && ros::ok() && is_get_camera_data_; i++)
  {
    ros::spinOnce();
    ros::Rate(10).sleep();
  }

  if (is_get_camera_data_)
  {
    std::cout << "[Mapping] " << cc.magenta << "Could not get camera data\n" << cc.reset;
  }

  return true;
}

bool MappingModule::commandFinalMapLoad()
{
  std::cout << "[Mapping] " << cc.yellow << "commandFinalMapLoad() not implimented\n" << cc.reset;
  return true;
}

bool MappingModule::commandFinalMapSave()
{
  std::cout << "[Mapping] " << cc.green << "Saving final maps\n" << cc.reset;

  // Point cloud
  if (!cloud_ptr_rgbd_)
  {
    std::cout << "[Mapping] " << cc.red << "ERROR: No point cloud data available. Exiting node.\n" << cc.reset;
    return false;
  }
  pcl::io::savePCDFileASCII (filename_pcl_final_, *cloud_ptr_rgbd_);

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

  std::cout << "[Mapping] " << cc.green << "Successfully saved map\n" << cc.reset;
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
  copyPointCloud(*cloud_ptr_profile_, *cloud_ptr_rgbd_);

  // Octree
  if (is_filling_octomap_)
  {
    std::cout << "[Mapping] " << "Reading " << filename_octree_ << "\n";

    octomap::AbstractOcTree* temp_tree = octomap::AbstractOcTree::read(filename_octree_);
    if(temp_tree)
    { // read error returns NULL
      octree_ = dynamic_cast<octomap::OcTree*>(temp_tree);

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
      addPointCloudToTree(octree_, *cloud_ptr_profile_symmetry_);
    else
      addPointCloudToTree(octree_prediction_, *cloud_ptr_profile_symmetry_);
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

  if (!octree_)
  {
    octree_ = new octomap::OcTree (octree_res_);
    octree_->setBBXMin( bound_min_ );
    octree_->setBBXMax( bound_max_ );
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
  std::cout << "[Mapping] " << cc.green << "Done profiling\n" << cc.reset;
  is_scanning_ = false;

  // Initialize cloud_ptr_rgbd_ with profile data
  copyPointCloud(*cloud_ptr_profile_, *cloud_ptr_rgbd_);

  if (is_checking_symmetry_)
  {
    SymmetryDetector* sym_det = new SymmetryDetector();
    sym_det->setInputCloud(cloud_ptr_profile_);
    sym_det->run();
    sym_det->getOutputCloud(cloud_ptr_profile_symmetry_);

    // Populate octree with symmetry data
    addPointCloudToTree(octree_prediction_, *cloud_ptr_profile_symmetry_);
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
        #ifdef _OPENMP
        #pragma omp critical (free_insert)
        #endif
        {
          free_cells.insert(keyray->begin(), keyray->end());
        }
      }
      // occupied endpoint
      octomap::OcTreeKey key;
      if (octree_in->coordToKeyChecked(p, key)){
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
      double max_rgbd_range__to_point = maxrange/sensor_dir.dot(direction);

      octomap::point3d new_end = origin + direction * (float) max_rgbd_range__to_point;
      if (octree_in->computeRayKeys(origin, new_end, *keyray)){
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

octomap::OcTree* MappingModule::getOctomap()
{
  return octree_;
}

octomap::OcTree* MappingModule::getOctomapPredicted()
{
  return octree_prediction_;
}

PointCloudXYZ::Ptr MappingModule::getPointCloud()
{
  return cloud_ptr_rgbd_;
  //return cloud_ptr_profile_;
}

void MappingModule::initializeParameters()
{
  is_get_camera_data_ = false;
  is_scanning_ = false;

  filename_pcl_          = "profile_cloud.pcd";
  filename_pcl_final_    = "final_cloud.pcd";
  filename_pcl_symmetry_ = "profile_cloud_symmetry.pcd";
  filename_octree_       = "profile_octree.ot";
  filename_octree_final_ = "final_octree.ot";

  topic_depth_        = "/nbv_exploration/depth";
  topic_map_          = "/nbv_exploration/global_map_cloud";
  topic_scan_in_      = "/nbv_exploration/scan";
  topic_scan_out_     = "/nbv_exploration/scan_cloud";
  topic_rgbd_out_     = "/nbv_exploration/rgbd_cloud";
  topic_tree_         = "/nbv_exploration/output_tree";
  topic_tree_predicted_="/nbv_exploration/output_tree_predicted";

  ros::param::param("~debug_mapping", is_debugging_, false);
  ros::param::param("~profiling_check_symmetry", is_checking_symmetry_, true);
  ros::param::param("~profiling_fill_octomap", is_filling_octomap_, true);
  ros::param::param("~profiling_fill_octomap_continuously", is_filling_octomap_continuously_, true);

  ros::param::param("~depth_range_max", max_rgbd_range_, 5.0);
  ros::param::param("~mapping_octree_resolution", octree_res_, 0.2);
  ros::param::param("~mapping_sensor_data_min_height_", sensor_data_min_height_, 0.5);
  ros::param::param("~mapping_voxel_grid_res_profile", profile_grid_res_, 0.1);
  ros::param::param("~mapping_voxel_grid_res_rgbd", depth_grid_res_, 0.1);
  ros::param::param("~mapping_integrate_prediction", is_integrating_prediction_, false);
  ros::param::param("~mapping_integrate_occupancy", predicted_occupancy_value_, 0.7);

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
}

void MappingModule::initializeTopicHandlers()
{
  // >>>>>>>>>>>>>>>>>
  // Subscribers / Servers
  // >>>>>>>>>>>>>>>>>

  // Sensor data
  sub_rgbd_ = ros_node_.subscribe(topic_depth_, 10, &MappingModule::callbackDepth, this);
  sub_scan_ = ros_node_.subscribe(topic_scan_in_, 10, &MappingModule::callbackScan, this);

  //ros::ServiceServer service = ros_node_.advertiseService("/nbv_exploration/mapping_command", &MappingModule::callbackCommand, this);

  tf_listener_ = new tf::TransformListener();

  // >>>>>>>>>>>>>>>>>
  // Publishers
  // >>>>>>>>>>>>>>>>>
  pub_global_cloud_  = ros_node_.advertise<sensor_msgs::PointCloud2>(topic_map_, 10);
  pub_scan_cloud_    = ros_node_.advertise<sensor_msgs::PointCloud2>(topic_scan_out_, 10);
  pub_rgbd_cloud_    = ros_node_.advertise<sensor_msgs::PointCloud2>(topic_rgbd_out_, 10);
  pub_tree_          = ros_node_.advertise<octomap_msgs::Octomap>(topic_tree_, 10);
  pub_tree_prediction_ = ros_node_.advertise<octomap_msgs::Octomap>(topic_tree_predicted_, 10);
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

  if (isPlanar)
    computeTreeUpdatePlanar(octree_in, ocCloud, sensor_origin, sensor_dir, free_cells, occupied_cells, range);
  else
    octree_in->computeUpdate(ocCloud, sensor_origin, free_cells, occupied_cells, range);

  // Clear all predicted values
  for (octomap::KeySet::iterator it = free_cells.begin(); it != free_cells.end(); ++it)
    octree_in->updateNode(*it, -5.0f);

  for (octomap::KeySet::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it)
    octree_in->updateNode(*it, -5.0f);
}
