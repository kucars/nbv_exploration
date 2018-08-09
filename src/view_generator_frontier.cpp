#include <iostream>
#include <queue>          // std::queue
#include <stdlib.h>
#include <ros/ros.h>

#include <Eigen/Geometry>
#include <Eigen/Eigen>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "nbv_exploration/view_generator_frontier.h"
#include "nbv_exploration/common.h"
#include <sensor_msgs/PointCloud2.h>

ViewGeneratorFrontier::ViewGeneratorFrontier():
    ViewGeneratorBase() //Call base class constructor
{

    ros::param::param<int>("~view_generator_frontier_minimum_size", minimum_frontier_size_, 10);
//    ros::param::param<int>("~view_generator_frontier_nearest_count", nearest_frontiers_count_, 3);
    ros::param::param<double>("~view_generator_frontier_cylinder_radius", cylinder_radius_, 2.5);
    ros::param::param<double>("~view_generator_frontier_cylinder_height", cylinder_height_, 1);
    ros::param::param<double>("~view_generator_frontier_density_threshold", density_threshold_, 10);

//    ros::NodeHandle ros_node;
    pub_vis_frontier_points_ = ros_node.advertise<sensor_msgs::PointCloud2>("nbv_exploration/generation/frontier_points", 10);
    pub_vis_points_          = ros_node.advertise<sensor_msgs::PointCloud2>("nbv_exploration/generation/visible_points", 10);
    pub_vis_centroid_points_ = ros_node.advertise<sensor_msgs::PointCloud2>("nbv_exploration/generation/centroid_points", 10);
//    pub_marker_normals_      = ros_node.advertise<visualization_msgs::Marker>("nbv_exploration/generation/normals", 10);
    pub_marker_normals_      = ros_node.advertise<geometry_msgs::PoseArray>("nbv_exploration/generation/normals", 10);
    pub_marker_planes_       = ros_node.advertise<visualization_msgs::Marker>("nbv_exploration/generation/planes", 10);
//    pub_marker_lines_        = ros_node.advertise<visualization_msgs::Marker>("nbv_exploration/generation/lines", 10);
    viewBase = new ViewSelecterBase();     // to get the rotation matrix
    std::cout<<"cameras number : "<<viewBase->camera_rotation_mtx_.size()<<std::endl;
    //@todo: later take the pitch angle from the rotation matrix from the tf and add the arrays according to the number of cameras
    setCameraParams({-20,20},{viewBase->fov_horizontal_,viewBase->fov_horizontal_},{viewBase->fov_vertical_,viewBase->fov_vertical_},viewBase->range_max_);
    std::cout<<cc.green<<"camera parameters set : ["<<viewBase->fov_horizontal_<<", "<<viewBase->fov_vertical_<<"] , max range:  "<<viewBase->range_max_ <<cc.reset <<std::endl;

}

std::vector<std::vector<octomap::OcTreeKey> >
ViewGeneratorFrontier::findFrontierAdjacencies(std::vector<octomap::OcTreeKey>& cells)
{
    std::vector<std::vector<octomap::OcTreeKey> > list;
    int cell_count = cells.size();

    std::vector<bool> was_cell_checked;
    was_cell_checked.assign(cell_count, false); //fill the vector with "false"

    for (int i_cell=0; i_cell<cell_count && ros::ok(); i_cell++)
    {
        // Cell was already checked, move along
        if (was_cell_checked[i_cell])
            continue;

        // key_list contains the final linked keys
        // key_queue is pushed and popped recursively until we find all linked cells
        std::vector<octomap::OcTreeKey> key_list;
        std::queue<KeyIndex> key_queue;

        KeyIndex ki;
        ki.key = cells[i_cell];
        ki.index = i_cell;

        key_queue.push(ki);

        // @todo: requires HEAVY optimization
        // Looks around the current cell for linked cells
        while (key_queue.size() > 0)
        {
            KeyIndex ki_dequeue = key_queue.front();
            key_queue.pop();

            if (was_cell_checked[ki_dequeue.index])
                continue;
            was_cell_checked[ki_dequeue.index] = true;
            key_list.push_back(ki_dequeue.key);

            // Check if any remaining cells are neighbors
            for (int i_check=0; i_check<cell_count; i_check++)
            {
                if (was_cell_checked[i_check])
                    continue;

                if ( isNear(cells[ki_dequeue.index], cells[i_check]) )
                {
                    KeyIndex ki_new;
                    ki_new.key = cells[i_check];
                    ki_new.index = i_check;

                    key_queue.push(ki_new);
                }
            } //end checking for neighbors
        } //end queue

        // Not enough frontier cells, do not count this as a frontier
        if (key_list.size() < minimum_frontier_size_)
            continue;

        list.push_back(key_list);
    } //end for

    return list;
}

std::vector<octomap::OcTreeKey> ViewGeneratorFrontier::findFrontierCells()
{
    int treeDepth = 16;
    std::vector<octomap::OcTreeKey> frontier_keys;


    for (octomap::OcTree::iterator it = tree_->begin(treeDepth), end = tree_->end(); it != end; ++it)
    {
        if (!isNodeUnknown(*it))
            continue;

        // check if cell is frontier (has 1 free and 1 occupied near it)
        bool found_free = false;
        bool found_occ = false;

        octomap::OcTreeKey key;
        octomap::OcTreeKey nKey = it.getKey();

        for (int k=-1; k <= 1; ++k)
        {
            key[2] = nKey[2] + k;

            for (int j=-1; j <= 1; ++j)
            {
                key[1] = nKey[1] + j;

                for (int i=-1; i <= 1; ++i)
                {
                    key[0] = nKey[0] + i;

                    if (key == nKey)
                        continue;

                    octomap::OcTreeNode* node = tree_->search(key);

                    if (!node)
                        continue;
                    else if ( isNodeFree(*node) )
                        found_free = true;
                    else if ( isNodeOccupied(*node) )
                        found_occ = true;
                } //end i: key[0]
            } //end j: key[1]
        } //end k: key[2]
        // end proximity check

        if (found_free && found_occ)
            frontier_keys.push_back(nKey);
    } //end for

    return frontier_keys;
}



std::vector<std::vector<octomap::OcTreeKey> > ViewGeneratorFrontier::findFrontiers()
{
    /*
  std::vector<octomap::OcTreeKey> frontier_cells = findFrontierCells();
  std::vector<std::vector<octomap::OcTreeKey> > frontier_list;
  frontier_list = findFrontierAdjacencies(frontier_cells);
  */

    std::vector<octomap::OcTreeKey> low_density_cells = findLowDensityCells(); //frontiers
    std::vector<std::vector<octomap::OcTreeKey> > density_list; //clusters of frontiers

    density_list  = findFrontierAdjacencies(low_density_cells);//clustering method

    //printf("[ViewGeneratorFrontier] Frontiers -- Entropy: %lu\tDensity: %lu\n", frontier_list.size(), density_list.size() );
    printf("[ViewGeneratorFrontier] Density Frontiers -- Cells %lu\tClusters: %lu\n", low_density_cells.size(), density_list.size() );

    std::vector<std::vector<octomap::OcTreeKey> > final_list;
    //final_list.insert(final_list.end(), frontier_list.begin(), frontier_list.end());
    final_list.insert(final_list.end(), density_list.begin(), density_list.end());

    return final_list;
}

void ViewGeneratorFrontier::findFrontiersPCL(std::vector<pcl::PointCloud<pcl::PointXYZ> >& clusters_frontiers_vec, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr >& clusters_frontiers_ptr_vec, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{

    pcl::search::KdTree<pcl::PointXYZ>::Ptr Kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
    Kdtree->setInputCloud (point_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.2); // 30cm
    ec.setMinClusterSize (5);
    ec.setMaxClusterSize (1000);//10000
    ec.setSearchMethod (Kdtree);
    ec.setInputCloud (point_cloud);
    ec.extract (cluster_indices);
    std::cout<<"number of clusters: "<<cluster_indices.size()<<std::endl;

    //////////converting the clusters to pointcloud/////////////////
    int j = 0;
    int r =0,g=0,b=0;
    //    std::vector<pcl::PointCloud<pcl::PointXYZ> > clusters_pointcloud; //vector to store clusters
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredClusters (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_color (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (point_cloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters_frontiers_vec.push_back(*cloud_cluster);
        r+=10; b+=30; g+=20; //coloring
        for(int i=0; i<cloud_cluster->points.size(); i++)
        {
            pcl::PointXYZRGB point = pcl::PointXYZRGB(r,g,b);
            point.x = cloud_cluster->points[i].x;
            point.y = cloud_cluster->points[i].y;
            point.z = cloud_cluster->points[i].z;
            coloredClusters->points.push_back(point);
            cloud_cluster_color->points.push_back(point);
        }
        clusters_frontiers_ptr_vec.push_back(cloud_cluster_color);
        //      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        j++;
    }

    printf("[ViewGeneratorFrontier2] Density Frontiers and PCL clusters -- Cells %lu\tClusters: %lu\n", point_cloud->size(), clusters_frontiers_vec.size() );
}

std::vector<octomap::OcTreeKey> ViewGeneratorFrontier::findLowDensityCells()
{
    int treeDepth = 16;
    std::vector<octomap::OcTreeKey> density_keys;

    for (octomap::OcTree::iterator it = tree_->begin(treeDepth), end = tree_->end(); it != end; ++it)
    {
        octomap::OcTreeKey key = it.getKey();
        // Ignore high density cells and invalid keys

        int density = mapping_module_->getDensityAtOcTreeKey(key);
//        std::cout<<cc.blue<<"density: "<<density_threshold_<<cc.reset<<std::endl;
        if (density <= 0 || density >= density_threshold_)
            continue;

        //Store low density keys
        density_keys.push_back( key );
    } //end for

    return density_keys;
}

//cameras bounds (from rrt explorrer)
void ViewGeneratorFrontier::setCameraParams(std::vector<double> cameraPitch,
                                    std::vector<double> cameraHorizontalFoV,
                                    std::vector<double> cameraVerticalFoV, double maxDist)
{

  // Precompute the normals of the separating hyperplanes that constrain the field of view.
  cameraPitch_ = cameraPitch;
  cameraHorizontalFoV_ = cameraHorizontalFoV;
  cameraVerticalFoV_ = cameraVerticalFoV;
  maxDist_ = maxDist;
  camBoundNormals_.clear();
  for (int i = 0; i < cameraPitch_.size(); i++) {
    double pitch = M_PI * cameraPitch_[i] / 180.0;
    double camTop = (pitch - M_PI * cameraVerticalFoV_[i] / 360.0) + M_PI / 2.0;
    double camBottom = (pitch + M_PI * cameraVerticalFoV_[i] / 360.0) - M_PI / 2.0;
    double side = M_PI * (cameraHorizontalFoV_[i]) / 360.0 - M_PI / 2.0;
    Eigen::Vector3d bottom(cos(camBottom), 0.0, -sin(camBottom));
    Eigen::Vector3d top(cos(camTop), 0.0, -sin(camTop));
    Eigen::Vector3d right(cos(side), sin(side), 0.0);
    Eigen::Vector3d left(cos(side), -sin(side), 0.0);
    Eigen::AngleAxisd m = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
    Eigen::Vector3d rightR = m * right;
    Eigen::Vector3d leftR = m * left;
    rightR.normalize();
    leftR.normalize();
    std::vector<Eigen::Vector3d> camBoundNormals;
    camBoundNormals.push_back(Eigen::Vector3d(bottom.x(), bottom.y(), bottom.z()));
    camBoundNormals.push_back(Eigen::Vector3d(top.x(), top.y(), top.z()));
    camBoundNormals.push_back(Eigen::Vector3d(rightR.x(), rightR.y(), rightR.z()));
    camBoundNormals.push_back(Eigen::Vector3d(leftR.x(), leftR.y(), leftR.z()));
    camBoundNormals_.push_back(camBoundNormals);
  }
}

//from rrt explorer
bool ViewGeneratorFrontier::pointInFOV(Eigen::Vector4d state, pcl::PointXYZ pt)
{

    visualization_msgs::Marker p;
    const double disc = 1;
    Eigen::Vector3d origin(state[0], state[1], state[2]);
    Eigen::Vector3d vec(pt.x,pt.y,pt.z);

    Eigen::Vector3d dir = vec - origin;
    bool insideAFieldOfView = false;
    // Check that voxel center is inside one of the fields of view.
    for (typename std::vector<std::vector<Eigen::Vector3d>>::iterator itCBN = camBoundNormals_.begin(); itCBN != camBoundNormals_.end(); itCBN++) {
        bool inThisFieldOfView = true;
        for (typename std::vector<Eigen::Vector3d>::iterator itSingleCBN = itCBN->begin();
             itSingleCBN != itCBN->end(); itSingleCBN++) {
            //angle in state[3] in radians
            Eigen::Vector3d normal = Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ())
                    * (*itSingleCBN);
            double val = dir.dot(normal.normalized());
            if (val < SQRT2 * disc) {
                inThisFieldOfView = false;
                break;
            }
        }
        if (inThisFieldOfView) {
            insideAFieldOfView = true;
            return true;
        }
    }
    return false;

}


//FOV and orientation evaluation
void ViewGeneratorFrontier::generateViews()
{
    generated_poses.clear();
    std::vector<geometry_msgs::Pose> rejected_poses;

    // ==========
    // Find frontiers
    // ==========
    timer.start("[ViewGeneratorFrontier]findFrontiers");
    std::vector<octomap::OcTreeKey> low_density_cells = findLowDensityCells();//finding the frontiers

    if (low_density_cells.size() == 0)
        return;
    timer.stop("[ViewGeneratorFrontier]findFrontiers");


    // ===========
    // Cluster and get Centroids of frontiers
    // ===========
    timer.start("[ViewGeneratorFrontier]getCentroids");
    PointCloudXYZ::Ptr centroid_cloud (new PointCloudXYZ);
    PointCloudXYZ::Ptr visualization_cloud (new PointCloudXYZ);

    //  PointCloudXYZ::Ptr clustering_cloud (new PointCloudXYZ);

    //convert the frontier octree keys to point cloud
    for (int i_f=0; i_f<low_density_cells.size(); i_f++)
    {
        octomap::point3d pt = tree_->keyToCoord(low_density_cells[i_f]);
        visualization_cloud->points.push_back( PointXYZ(pt.x(), pt.y(), pt.z()) );
    }

    //clustering the frontiers using PCL
    std::vector<pcl::PointCloud<pcl::PointXYZ> > clusters_pointcloud_vec ;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > clusters_pointcloud_ptr_vec ;
    findFrontiersPCL(clusters_pointcloud_vec,clusters_pointcloud_ptr_vec, visualization_cloud);


    //getting the centroids using PCL
    for (int i_c=0; i_c<clusters_pointcloud_vec.size(); i_c++)
    {
        pcl::CentroidPoint<pcl::PointXYZ> centroid;
        for(int i_p=0; i_p<clusters_pointcloud_vec[i_c].points.size(); i_p++)
            centroid.add(clusters_pointcloud_vec[i_c].points[i_p]);
        pcl::PointXYZ computed_centroid;
//        Eigen::Vector4f centroid;
        centroid.get(computed_centroid);
//        pcl::compute3DCentroid (clusters_pointcloud_vec[i_c], centroid);
//        computed_centroid.x = centroid[0];computed_centroid.y = centroid[1]; computed_centroid.z = centroid[2];
        centroid_cloud->points.push_back(computed_centroid);
    }

    timer.stop("[ViewGeneratorFrontier]getCentroids");


    // Visualize
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*visualization_cloud, cloud_msg);
    cloud_msg.header.frame_id = "world";
    cloud_msg.header.stamp = ros::Time::now();
    pub_vis_frontier_points_.publish(cloud_msg);

    sensor_msgs::PointCloud2 cloud_msg2;
    pcl::toROSMsg(*centroid_cloud, cloud_msg2);
    cloud_msg2.header.frame_id = "world";
    cloud_msg2.header.stamp = ros::Time::now();
    pub_vis_centroid_points_.publish(cloud_msg2);

    // ============
    // Get nearest frontiers
    // ============
    timer.start("[ViewGeneratorFrontier]getNearestFrontier");
    pcl::KdTreeFLANN<PointXYZ> kdtree;
    kdtree.setInputCloud (centroid_cloud);
    std::vector<int> pointIdxNKNSearch(nearest_frontiers_count_);
    std::vector<float> pointNKNSquaredDistance(nearest_frontiers_count_);

    PointXYZ current_pt (
                current_pose_.position.x,
                current_pose_.position.y,
                current_pose_.position.z
                );

    if ( kdtree.nearestKSearch (current_pt, nearest_frontiers_count_, pointIdxNKNSearch, pointNKNSquaredDistance) == 0 )
        return;

    std::cout<<cc.yellow<<"number of near clusters "<<pointIdxNKNSearch.size()<<cc.reset<<std::endl;
    PointCloudXYZ::Ptr global_ptr (new PointCloudXYZ);
    pcl::PointCloud<pcl::PointXYZ> global;
    OcclusionCulling* occ = new OcclusionCulling(ros_node,centroid_cloud);
    std::vector<geometry_msgs::Pose> generated_poses_copy;

    for (int i=0; i<pointIdxNKNSearch.size(); i++)
    {
        int idx = pointIdxNKNSearch[i];
//        std::cout<<cc.yellow<<"cluster processing start"<<idx<<cc.reset<<std::endl;
        PointXYZ centroid = centroid_cloud->points[idx];

        // Generate poses around the centroid
        PointCloudXYZ::Ptr frontier_cluster (new PointCloudXYZ);
        PointCloudXYZ::Ptr mapped_cloud (new PointCloudXYZ);

        //method 1: problem is that it only considers the cloud, it doesn't consider the covered surroundings
        //        frontier_cluster->points = clusters_pointcloud_vec[idx].points;

        //method 2: consider the surroundings using Kdtree
        pcl::KdTreeFLANN<PointXYZ> kdtree_frontier;
        mapped_cloud = mapping_module_->getPointCloud();
        kdtree_frontier.setInputCloud (mapped_cloud);
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        if ( kdtree_frontier.radiusSearch (centroid, (cylinder_radius_-1), pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0 )
            return;
        else
        {
            for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
                     frontier_cluster->points.push_back(mapped_cloud->points[ pointIdxRadiusSearch[i] ]);
        }

        double coverage_percent =0;
        for (double theta=0; theta<=2*M_PI && coverage_percent<80; theta+=M_PI_4)
        {
            //decrement to up
            int max=0,index=0;
            occ->initConfig(frontier_cluster);
            PointCloudXYZ::Ptr visible_ptr (new PointCloudXYZ);

//            std::vector<geometry_msgs::Pose> waypoints;
            for (int z_inc=-1; z_inc<=1; z_inc+=1)
            {
                geometry_msgs::Pose pose;
                pose.position.x = centroid.x + cylinder_radius_*cos(theta);
                pose.position.y = centroid.y + cylinder_radius_*sin(theta);
                pose.position.z = centroid.z + z_inc*cylinder_height_;
                pose.orientation = pose_conversion::getQuaternionFromYaw(M_PI+theta); // Point to center of cylinder
                Eigen::Vector3d T(pose.position.x,pose.position.y,pose.position.z);

                //get the position
                Eigen::Matrix3d r_pose=pose_conversion::getRotationMatrix( pose );
                Eigen::Matrix4d robotPoseMat;
                robotPoseMat.setZero ();
                robotPoseMat.block (0, 0, 3, 3) = r_pose;
                robotPoseMat.block (0, 3, 3, 1) = T;
                robotPoseMat (3, 3) = 1;

                //check if the centroid inside the FOVs of this pose (according to the rrt implementation without taking into consideration of the camera translation)
                Eigen::Vector4d point(pose.position.x,pose.position.y,pose.position.z,M_PI+theta);

                if(pointInFOV(point,centroid) && isValidViewpoint(pose))
                {
//                    std::cout<<cc.red<<"is valid and inside FOV : "<<isValidViewpoint(pose)<<" "<<pointInFOV(point,centroid) <<cc.reset<<std::endl;
                    if(collision_check_mesh_)
                    {
                        if(generated_poses_copy.size()>0)
                        {
                            if( !(isConnectionConditionSatisfied(generated_poses_copy[generated_poses_copy.size()-1],pose)) )
                            {
                               rejected_poses.push_back(pose);
                               continue;
                            }
                        }else
                        {
                            if( !(isConnectionConditionSatisfied(pose)) )
                            {
                                rejected_poses.push_back(pose);
                                continue;
                            }
                        }
                    }

                    pcl::PointCloud<pcl::PointXYZ> visible;
                    for (int c=0; c<viewBase->camera_count_; c++)
                    {
                        Eigen::Matrix4d robot2sensorMat, sensorPoseMat,sensor2sensorMat;
                        robot2sensorMat.setZero ();
                        robot2sensorMat.block (0, 0, 3, 3) = viewBase->camera_rotation_mtx_[c];
                        robot2sensorMat.block (0, 3, 3, 1) = viewBase->camera_translation_[c];
                        robot2sensorMat (3, 3) = 1;
                        sensorPoseMat = robotPoseMat * robot2sensorMat;


                        //extract the visible points
                        //the frustum culling sensor needs this
                        //the transofrmation is rotation by +90 around x axis of the sensor
                        sensor2sensorMat << 1, 0, 0, 0,
                                            0, 0,-1, 0,
                                            0, 1, 0, 0,
                                            0, 0, 0, 1;
                        Eigen::Matrix4d newSensorPoseMat = sensorPoseMat * sensor2sensorMat;

                        Eigen::Vector3d T_Eigen;Eigen::Matrix3d R_Eigen; tf::Matrix3x3 R_TF;tf::Quaternion qt;
                        R_Eigen = newSensorPoseMat.block (0, 0, 3, 3);
                        tf::matrixEigenToTF(R_Eigen,R_TF);
                        R_TF.getRotation(qt);
                        T_Eigen = newSensorPoseMat.block (0, 3, 3, 1);
                        geometry_msgs::Pose p;
                        p.position.x=T_Eigen[0];p.position.y=T_Eigen[1];p.position.z=T_Eigen[2];
                        p.orientation.x = qt.getX(); p.orientation.y = qt.getY();p.orientation.z = qt.getZ();p.orientation.w = qt.getW();
                        visible += occ->extractVisibleSurface(p);
                        global += visible;
                        occ->visualizeFOV(p);

//                        std::cout<<cc.blue<<"frustum cluster size: "<<frontier_cluster->points.size()<<" visible: "<<visible.points.size()<<cc.reset<<std::endl;
                    }//end of loop through the sensors
                    if(max==0)
                    {
                        max = visible.points.size();
                        generated_poses_copy.push_back(pose);
                        visible_ptr->points = visible.points;
                    }else if (visible.points.size()>max)
                    {
                        max = visible.points.size();
                        generated_poses_copy.pop_back();
                        generated_poses_copy.push_back(pose);
                        visible_ptr->points = visible.points;

                    }


                }else//end of if inside FOVs and if the waypoint is  valid
                {
//                    std::cout<<cc.blue<<"rejected"<<cc.reset<<std::endl;
                    rejected_poses.push_back(pose);
                }


            }//end of loop through the viewpoint z discretization

            if(visible_ptr->points.size()!=0)
            {
                coverage_percent = occ->calcCoveragePercent(visible_ptr);
//                std::cout<<cc.red<<"coverage : "<<coverage_percent<<cc.reset<<std::endl;
            }else {
                coverage_percent=0;
            }
//            std::cout<<cc.blue<<"frustum cluster size before: "<<frontier_cluster->points.size()<<cc.reset<<std::endl;
            frontier_cluster->points = (occ->pointsDifference(*visible_ptr)).points;
//            std::cout<<cc.blue<<"frustum cluster size after: "<<frontier_cluster->points.size()<<cc.reset<<std::endl;


        }// end of loop through the different orientations

//        std::cout<<cc.yellow<<"cluster processing ends "<<idx<<cc.reset<<std::endl;

    }// end of loop through the frontier clusters


    for(int i=generated_poses_copy.size()-1; i>=0; i--)
    {
        generated_poses.push_back(generated_poses_copy[i]);
    }

    // Visualize
    global_ptr->points = global.points;
    sensor_msgs::PointCloud2 cloud_msg3;
    pcl::toROSMsg(*global_ptr, cloud_msg3);
    cloud_msg3.header.frame_id = "world";
    cloud_msg3.header.stamp = ros::Time::now();
    pub_vis_points_.publish(cloud_msg3);
    timer.stop("[ViewGeneratorFrontier]getNearestFrontier");


    // Visualize
    std::cout << "[ViewGeneratorFrontier] Generated " << generated_poses.size() << " poses (" << rejected_poses.size() << " rejected)" << std::endl;
    ViewGeneratorBase::visualize(generated_poses, rejected_poses);
}

std::string ViewGeneratorFrontier::getMethodName()
{
    return "Frontier";
}

bool ViewGeneratorFrontier::isNear(octomap::OcTreeKey k1, octomap::OcTreeKey k2)
{
    if (abs(k1[0]-k2[0]) <=2 &&
            abs(k1[1]-k2[1]) <=2 &&
            abs(k1[2]-k2[2]) <=2)
        return true;

    return false;
}

bool ViewGeneratorFrontier::isNodeFree(octomap::OcTreeNode node)
{
    //if (node.getOccupancy() <= 1-tree_->getOccupancyThres())
    if (node.getOccupancy() <= 0.4)
        return true;

    return false;
}

bool ViewGeneratorFrontier::isNodeOccupied(octomap::OcTreeNode node)
{
    //if (node.getOccupancy() >= tree_->getOccupancyThres())
    if (node.getOccupancy() >= 0.6)
        return true;

    return false;
}

bool ViewGeneratorFrontier::isNodeUnknown(octomap::OcTreeNode node)
{
    return !isNodeFree(node) && !isNodeOccupied(node);
}

//new
void ViewGeneratorFrontier::visualizeNormals(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, geometry_msgs::PoseArray& normal_poses, bool pcl_visualize)
{
    if(!pcl_visualize)
    {
        for(int i = 0; i<cloud_normals->size(); i++)
        {
            Eigen::Vector3d axis_vector;
            geometry_msgs::Pose output_vector;
            Eigen::Quaterniond q;
            axis_vector[0] = cloud_normals->points[i].normal_x;
            axis_vector[1] = cloud_normals->points[i].normal_y;
            axis_vector[2] = cloud_normals->points[i].normal_z;
            //    std::cout<<"axis vector pose>> "<<axis_vector[0]<<" "<<axis_vector[1]<<" "<<axis_vector[2]<<std::endl;
            axis_vector.normalize();
            Eigen::Vector3d up_vector(0.0, 0.0, 1.0);//changes the orientation of the normal
            Eigen::Vector3d right_axis_vector = axis_vector.cross(up_vector);
            //    std::cout<<"right_axis_ vector pose>> "<<right_axis_vector[0]<<" "<<right_axis_vector[1]<<" "<<right_axis_vector[2]<<std::endl;

            right_axis_vector.normalized();
            double theta = axis_vector.dot(up_vector);
            double angle_rotation = -1.0*acos(theta);
            //    std::cout<<"theta and angle orientaiton>> "<<theta<<" "<<angle_rotation<<std::endl;

            tf::Vector3 tf_right_axis_vector;
            tf::vectorEigenToTF(right_axis_vector, tf_right_axis_vector);
            //    std::cout<<"tf right axis orientaiton>> "<<tf_right_axis_vector[0]<<" "<<tf_right_axis_vector[1]<<" "<<tf_right_axis_vector[2]<<std::endl;
            tf::Quaternion tf_q(tf_right_axis_vector, angle_rotation);
            //    std::cout<<"tf angle orientaiton>> "<<tf_q.x()<<" "<<tf_q.y()<<" "<<tf_q.z()<<" "<<tf_q.w()<<std::endl;
            tf::quaternionTFToEigen(tf_q, q);
            Eigen::Affine3d pose;
            q.normalize();
            pose = q * Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitY());
            Eigen::Vector3d a;
            a[0]= cloud->points[i].x;
            a[1]= cloud->points[i].y;
            a[2]= cloud->points[i].z;
            //    std::cout<<"a >> "<<a[0]<<" "<<a[1]<<" "<<a[2]<<std::endl;
            pose.translation() = a;
            tf::poseEigenToMsg(pose, output_vector);
            //    std::cout<<"output vector pose>> "<<output_vector.position.x<<" "<<output_vector.position.y<<" "<<output_vector.position.z<<std::endl;
            //    std::cout<<"output vector orientation>> "<<output_vector.orientation.x<<" "<<output_vector.orientation.y<<" "<<output_vector.orientation.z<<" "<<output_vector.orientation.w <<std::endl;
            normal_poses.poses.push_back(output_vector);
        }
        normal_poses.header.frame_id = "world";
        normal_poses.header.stamp = ros::Time::now();
        pub_marker_normals_.publish(normal_poses);
    }
    else
    {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0, 0, 0);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb,"sample cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
        viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, cloud_normals, 5, 0.1, "normals");
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();
        //visualization

        while (!viewer->wasStopped ())
        {
            viewer->spinOnce ();
        }
    }

}


