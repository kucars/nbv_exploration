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
    ros::param::param<int>("~view_generator_frontier_nearest_count", nearest_frontiers_count_, 10);
    ros::param::param<double>("~view_generator_frontier_cylinder_radius", cylinder_radius_, 3.0);
    ros::param::param<double>("~view_generator_frontier_cylinder_height", cylinder_height_, 1.0);
    ros::param::param<double>("~view_generator_frontier_density_threshold", density_threshold_, 10);

    ros::NodeHandle ros_node;
    pub_vis_frontier_points_ = ros_node.advertise<sensor_msgs::PointCloud2>("nbv_exploration/generation/frontier_points", 10);
    pub_vis_centroid_points_ = ros_node.advertise<sensor_msgs::PointCloud2>("nbv_exploration/generation/centroid_points", 10);
//    pub_marker_normals_      = ros_node.advertise<visualization_msgs::Marker>("nbv_exploration/generation/normals", 10);
    pub_marker_normals_      = ros_node.advertise<geometry_msgs::PoseArray>("nbv_exploration/generation/normals", 10);

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
    ec.setClusterTolerance (0.8); // 30cm
    ec.setMinClusterSize (5);
    ec.setMaxClusterSize (50000);//10000
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
        if (density <= 0 || density >= density_threshold_)
            continue;

        //Store low density keys
        density_keys.push_back( key );
    } //end for

    return density_keys;
}

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
        //      centroid.add( PointXYZ(clustersPointCloudVec[i_c].points[i_p].data[0],clustersPointCloudVec[i_c].points[i_p].data[1], clustersPointCloudVec[i_c].points[i_p].data[2]) );

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
    // Pose Generation
    // ============
    //going through frontier clusters
    timer.start("[ViewGeneratorFrontier]viewGeneration");

    for(int j =0 ; j<clusters_pointcloud_vec.size(); j++)
    {
        //normals
//        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
//        ne.setInputCloud (clusters_pointcloud_ptr_vec[j]);
//        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGB> ());
//        ne.setSearchMethod (tree2);
//        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);// Output datasets
//        // Use all neighbors in a sphere of radius 3cm
//        ne.setRadiusSearch (0.5); // Compute the features
//        ne.compute (*cloud_normals);

//        std::cout<<cc.cyan<<"[ViewGeneratorFrontier] Normals size : " <<cloud_normals->points.size()<<std::endl<<cc.reset;
//        geometry_msgs::PoseArray normal_poses ;
//        visualizeNormals(cloud_normals,clusters_pointcloud_ptr_vec[j], normal_poses, false);

        Eigen::Vector4f plane_params;
        float curvature;
        pcl::computePointNormal(clusters_pointcloud_vec[j],plane_params, curvature);
        geometry_msgs::Vector3 grid_size;
        geometry_msgs::Pose grid_start;
        findClusterBB(clusters_pointcloud_vec[j], grid_size, grid_start);
        double  grid_res = 1.0;
        std::cout<<cc.white<<"grid size: "<< grid_size.x<<" "<<grid_size.y<< " " <<grid_size.z << cc.reset <<std::endl;
        std::cout<<cc.white<<"grid_start: "<< grid_start.position.x<<" "<<grid_start.position.y<< " " <<grid_start.position.z << cc.reset <<std::endl;

        double orientation_res=45;
        //intersection cannot be performed using point (used segment)
        Segment centroid_seg(Point(centroid_cloud->points[j].x,centroid_cloud->points[j].y,centroid_cloud->points[j].z),Point(centroid_cloud->points[j].x,centroid_cloud->points[j].y,centroid_cloud->points[j].z));
        //or use the cluster estimated plane normal for intersection
        Plane3 cluster_plane(plane_params[0],plane_params[1],plane_params[2],plane_params[3]);

        //        Direction d(centroid_seg);
        //        std::cout<<"direction of centroid segment: "<<d.dx()<<" "<<d.dy()<<" "<<d.dz()<<std::endl;
        for(double z = (grid_start.position.z ); z<=(grid_start.position.z + grid_size.z); z+=grid_res)
        {
            for(double x = (grid_start.position.x ); x<=(grid_start.position.x + grid_size.x); x+=grid_res)
            {
                for(double y = (grid_start.position.y ); y<=(grid_start.position.y + grid_size.y); y+=grid_res)
                {
                    geometry_msgs::Pose pose;
                    pose.position.x = x;
                    pose.position.y = y;
                    pose.position.z = z;

                    int orientationsNum= 360.0f/orientation_res;
                    // in radians
                    double yaw=0.0;
                    tf::Quaternion tf ;
                    for(int i=0; i<orientationsNum;i++)
                    {
                        tf = tf::createQuaternionFromYaw(yaw);
                        pose.orientation.x = tf.getX();
                        pose.orientation.y = tf.getY();
                        pose.orientation.z = tf.getZ();
                        pose.orientation.w = tf.getW();

                        if ( isValidViewpoint(pose) ) // using the isValid of viewgenerator base
                        {
                            //check intersection with the centroid small segment (need to check if I need to generate the sensor poses with respect to the robot or not needed in this case)
                            Ray r(Point(x,y,z),Direction(round(cos(yaw)),round(sin(yaw)),0));
//                            Direction d_line(r);
//                            std::cout<<"cos: "<<cos(yaw)<<" "<<(int)cos(yaw)<<" "<<round(cos(yaw))<<" yaw "<<yaw<<std::endl;
//                            std::cout<<"direction of line: "<<d_ray.dx()<<" "<<d_ray.dy()<<" "<<d_ray.dz()<<std::endl;
                            if(CGAL::do_intersect(cluster_plane,r))
                            {
                                generated_poses.push_back(pose);

                            }
                            else
                            {
                              rejected_poses.push_back(pose);
                            }

                        }

                        else
                        {
                           rejected_poses.push_back(pose);
                        }
                        yaw+=(orientation_res*M_PI/180.0f);


                    }//orientation discretization

                }//position discretization
            }
        }


    }//clusters loop
    timer.stop("[ViewGeneratorFrontier]viewGeneration");

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

//taken from the ASSPP (coverage heuristic of sspp package)
void ViewGeneratorFrontier::findClusterBB(pcl::PointCloud<pcl::PointXYZ> clusterPoints, geometry_msgs::Vector3& gridSize, geometry_msgs::Pose& gridStart)
{
    //finding bounding box of cluster
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusterPointsPtr (new pcl::PointCloud<pcl::PointXYZ>);
    clusterPointsPtr->points = clusterPoints.points;
    pcl::PointCloud<pcl::PointXYZ> tempClusterPoints;
    pcl::VoxelGridOcclusionEstimation<pcl::PointXYZ> grid;
    grid.setInputCloud (clusterPointsPtr);
    grid.setLeafSize (0.5, 0.5, 0.5);
    grid.initializeVoxelGrid();
    grid.filter(tempClusterPoints);

    //getting grid size and start
    Eigen::Vector4f min_b = grid.getCentroidCoordinate (grid.getMinBoxCoordinates());
    Eigen::Vector4f max_b = grid.getCentroidCoordinate (grid.getMaxBoxCoordinates ());

    // 3 and 5 is used to making the BB bigger not exactly on the boundry of the cluster
    // (sometimes it is very small set of samples and the descritization sample will not fit)
    double maximizeSizeXY = 3;
    double maximizeSizeZ = 1;
    gridSize.x = std::abs(max_b[0]-min_b[0]) + maximizeSizeXY;//5
    gridSize.y = std::abs(max_b[1]-min_b[1]) + maximizeSizeXY;//5
    gridSize.z = std::abs(max_b[2]-min_b[2]) + maximizeSizeZ;//3

    gridStart.position.x = min_b[0] - double(maximizeSizeXY/2);//5
    gridStart.position.y = min_b[1] - double(maximizeSizeXY/2);//5
    gridStart.position.z = min_b[2];//to avoid going under 0, UAVs can't fly under 0

}
