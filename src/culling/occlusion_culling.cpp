/***************************************************************************
 *   Copyright (C) 2015 - 2018 by                                          *
 *      Tarek Taha, KURI  <tataha@tarektaha.com>                           *
 *      Randa Almadhoun   <randa.almadhoun@kustar.ac.ae>                   *
 *      Abdullah Abdeldayem <abdullah.dayem@kustar.ac.ae>                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/
//modified version from component_test package

#include "culling/occlusion_culling.h"
#include <ros/ros.h>

OcclusionCulling::OcclusionCulling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_xyzrgb){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::copyPointCloud(*cloud_xyzrgb, *cloud_xyz);

    initConfig(cloud_xyz);
}


OcclusionCulling::OcclusionCulling(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudPtr){
    initConfig(cloudPtr);
}

OcclusionCulling::OcclusionCulling(ros::NodeHandle & n, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudPtr):
    nh(n),
    fc(true)
{
    occupancyGrid = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud <pcl::PointXYZRGB>);
    fov_pub = nh.advertise<visualization_msgs::MarkerArray>("fov", 100);
    occupancy_pub = nh.advertise<sensor_msgs::PointCloud2> ("occupancy_cloud",100);
    ray_pub = nh.advertise<visualization_msgs::Marker>("lines", 100);

    initConfig(cloudPtr, n);

}

void OcclusionCulling::initConfig(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudPtr){
    ros::NodeHandle n = ros::NodeHandle("~") ;
    initConfig(cloudPtr, n);
}

void OcclusionCulling::initConfig(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudPtr, ros::NodeHandle& nodeHandle) {
    // Get config parameters
    nodeHandle.param<float>("voxelRes", voxelRes, 0.1f );
    nodeHandle.param<float>("sensor_vert_fov", vert_fov, 45);
    nodeHandle.param<float>("sensor_hor_fov", hor_fov, 60);
    nodeHandle.param<float>("sensor_near_plane_distance", near_dist, 0.7);
    nodeHandle.param<float>("sensor_far_plane_distance", far_dist, 8.0);
    nodeHandle.param<std::string>("frame_id", frame_id, "world");

    cloud = cloudPtr;
    filtered_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
    FrustumCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
    occlusionFreeCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);

    OriginalVoxelsSize=0.0;
    id=0.0;
    voxelFilterOriginal.setInputCloud (cloud);
    voxelFilterOriginal.setLeafSize (voxelRes, voxelRes, voxelRes);
    voxelFilterOriginal.initializeVoxelGrid();
    min_b1 = voxelFilterOriginal.getMinBoxCoordinates ();
    max_b1 = voxelFilterOriginal.getMaxBoxCoordinates ();
    for (int kk = min_b1.z (); kk <= max_b1.z (); ++kk)
    {
        for (int jj = min_b1.y (); jj <= max_b1.y (); ++jj)
        {
            for (int ii = min_b1.x (); ii <= max_b1.x (); ++ii)
            {
                Eigen::Vector3i ijk1 (ii, jj, kk);
                int index1 = voxelFilterOriginal.getCentroidIndexAt (ijk1);
                if(index1!=-1)
                {
                    OriginalVoxelsSize++;
                }

            }
        }
    }
    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
    voxelgrid.setInputCloud (cloud);
    voxelgrid.setLeafSize (voxelRes, voxelRes, voxelRes);
    voxelgrid.filter (*filtered_cloud);

    fc.setInputCloud (cloud);
    fc.setVerticalFOV (vert_fov);
    fc.setHorizontalFOV (hor_fov);
    fc.setNearPlaneDistance (near_dist);
    fc.setFarPlaneDistance (far_dist);

}

pcl::PointCloud<pcl::PointXYZ> OcclusionCulling::extractVisibleSurface(geometry_msgs::Pose location)
{
    // >>>>>>>>>>>>>>>>>>>>
    // 1. Frustum Culling
    // >>>>>>>>>>>>>>>>>>>>
    pcl::PointCloud <pcl::PointXYZ>::Ptr output (new pcl::PointCloud <pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr occlusionFreeCloud_local(new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Matrix4f camera_pose;
    Eigen::Matrix3d Rd;
    Eigen::Matrix3f Rf;

    camera_pose.setZero ();


    // Convert quaterion orientation to XYZ angles (?)
    tf::Quaternion qt;
    qt.setX(location.orientation.x);
    qt.setY(location.orientation.y);
    qt.setZ(location.orientation.z);
    qt.setW(location.orientation.w);
    tf::Matrix3x3 R_tf(qt);
    tf::matrixTFToEigen(R_tf,Rd);
    Rf = Rd.cast<float>();

    camera_pose.block (0, 0, 3, 3) = Rf;

    // Set position
    Eigen::Vector3f T;
    T (0) = location.position.x;
    T (1) = location.position.y;
    T (2) = location.position.z;
    camera_pose.block (0, 3, 3, 1) = T;

    // Set pose
    camera_pose (3, 3) = 1;
    fc.setCameraPose (camera_pose);

    // Perform culling
    ros::Time tic = ros::Time::now();
    fc.filter (*output);
    ros::Time toc = ros::Time::now();

    //std::cout<<"\nFrustum Filter took:"<< toc.toSec() - tic.toSec();
    FrustumCloud->points= output->points;

    // >>>>>>>>>>>>>>>>>>>>
    // 2. Voxel grid occlusion estimation
    // >>>>>>>>>>>>>>>>>>>>
    Eigen::Quaternionf quat(qt.w(),qt.x(),qt.y(),qt.z());
    output->sensor_origin_  = Eigen::Vector4f(T[0],T[1],T[2],0);
    output->sensor_orientation_= quat;
    pcl::VoxelGridOcclusionEstimationT voxelFilter;
    voxelFilter.setInputCloud (output);
    voxelFilter.setLeafSize (voxelRes, voxelRes, voxelRes);
    voxelFilter.initializeVoxelGrid();

    int state,ret;

    pcl::PointXYZ pt,p1,p2;
    pcl::PointXYZRGB point;
    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > out_ray;
    //std::vector<geometry_msgs::Point> lineSegments;
    geometry_msgs::Point linePoint;

    // iterate over the entire frustum points
    for ( int i = 0; i < (int)output->points.size(); i ++ )
    {
        // Get voxel centroid corresponding to selected point
        pcl::PointXYZ ptest = output->points[i];
        Eigen::Vector3i ijk = voxelFilter.getGridCoordinates( ptest.x, ptest.y, ptest.z);

        if(voxelFilter.getCentroidIndexAt(ijk) == -1 ) {
            // Voxel is out of bounds
            continue;
        }

        Eigen::Vector4f centroid = voxelFilter.getCentroidCoordinate (ijk);
        point = pcl::PointXYZRGB(0,244,0);
        point.x = centroid[0];
        point.y = centroid[1];
        point.z = centroid[2];

        // >>>>>>>>>>>>>>>>>>>>
        // 2.1 Perform occlusion estimation
        // >>>>>>>>>>>>>>>>>>>>

        /*
         * The way this works is it traces a line to the point and finds
         * the distance to the first occupied voxel in its path (t_min).
         * It then determines if the distance between the target point and
         * the collided voxel are close (within voxelRes).
         *
         * If they are, the point is visible. Otherwise, the point is behind
         * an occluding point
         *
         *
         * This is the expected function of the following command:
         *   ret = voxelFilter.occlusionEstimation( state,out_ray, ijk);
         *
         * However, it sometimes shows occluded points on the edge of the cloud
         */

        // Direction to target voxel
        Eigen::Vector4f direction = centroid - output->sensor_origin_;
        direction.normalize ();

        // Estimate entry point into the voxel grid
        float tmin = voxelFilter.rayBoxIntersection (output->sensor_origin_, direction,p1,p2); //where did this 4-input syntax come from?

        if(tmin == -1){
            // ray does not intersect with the bounding box
            continue;
        }

        // Calculate coordinate of the boundary of the voxel grid
        Eigen::Vector4f start = output->sensor_origin_ + tmin * direction;

        // Determine distance between boundary and target voxel centroid
        Eigen::Vector4f dist_vector = centroid-start;
        float distance = (dist_vector).dot(dist_vector);

        if (distance > voxelRes*1.414){ // voxelRes/sqrt(2)
            // ray does not correspond to this point
            continue;
        }

        // Save point
        occlusionFreeCloud_local->points.push_back(ptest);
        occlusionFreeCloud->points.push_back(ptest);

        // >>>>>>>>>>>>>>>>>>>>
        // 2.2 Save line segment for visualization
        // >>>>>>>>>>>>>>>>>>>>

        linePoint.x = output->sensor_origin_[0];
        linePoint.y = output->sensor_origin_[1];
        linePoint.z = output->sensor_origin_[2];
        lineSegments.push_back(linePoint);

        linePoint.x = start[0];
        linePoint.y = start[1];
        linePoint.z = start[2];
        lineSegments.push_back(linePoint);

        occupancyGrid->points.push_back(point);
    }

    FreeCloud.points = occlusionFreeCloud_local->points;

    return FreeCloud;
}



float OcclusionCulling::calcCoveragePercent(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{

    // *******************original cloud Grid***************************
    //used VoxelGridOcclusionEstimationT since the voxelGrid does not include getcentroid function
    //        pcl::VoxelGridOcclusionEstimationT voxelFilterOriginal;
    //        voxelFilterOriginal.setInputCloud (cloud);
    //        voxelFilterOriginal.setLeafSize (voxelRes, voxelRes, voxelRes);
    //        voxelFilterOriginal.initializeVoxelGrid();

    //*******************Occupied Cloud Grid***************************
    ros::Time covpercent_begin = ros::Time::now();
    pcl::VoxelGridOcclusionEstimationT voxelFilterOccupied;
    //        voxelFilterOccupied.setInputCloud (occlusionFreeCloud);
    voxelFilterOccupied.setInputCloud (cloud_filtered);
    voxelFilterOccupied.setLeafSize (voxelRes, voxelRes, voxelRes);
    voxelFilterOccupied.initializeVoxelGrid();



    //*****************************************************************
    Eigen::Vector3i  min_b = voxelFilterOccupied.getMinBoxCoordinates ();
    Eigen::Vector3i  max_b = voxelFilterOccupied.getMaxBoxCoordinates ();
    //        Eigen::Vector3i  min_b1 = voxelFilterOriginal.getMinBoxCoordinates ();
    //        Eigen::Vector3i  max_b1 = voxelFilterOriginal.getMaxBoxCoordinates ();

    float MatchedVoxels=0 ;//OriginalVoxelsSize=0, ;

    // iterate over the entire original voxel grid to get the size of the grid
    //        for (int kk = min_b1.z (); kk <= max_b1.z (); ++kk)
    //        {
    //            for (int jj = min_b1.y (); jj <= max_b1.y (); ++jj)
    //            {
    //                for (int ii = min_b1.x (); ii <= max_b1.x (); ++ii)
    //                {
    //                    Eigen::Vector3i ijk1 (ii, jj, kk);
    //                    int index1 = voxelFilterOriginal.getCentroidIndexAt (ijk1);
    //                    if(index1!=-1)
    //                    {
    //                        OriginalVoxelsSize++;
    //                    }

    //                }
    //            }
    //        }

    //iterate through the entire coverage grid to check the number of matched voxel between the original and the covered ones
    for (int kk = min_b.z (); kk <= max_b.z (); ++kk)
    {
        for (int jj = min_b.y (); jj <= max_b.y (); ++jj)
        {
            for (int ii = min_b.x (); ii <= max_b.x (); ++ii)
            {

                Eigen::Vector3i ijk (ii, jj, kk);
                int index1 = voxelFilterOccupied.getCentroidIndexAt (ijk);
                if(index1!=-1)
                {
                    Eigen::Vector4f centroid = voxelFilterOccupied.getCentroidCoordinate (ijk);
                    Eigen::Vector3i ijk_in_Original= voxelFilterOriginal.getGridCoordinates(centroid[0],centroid[1],centroid[2]) ;

                    int index = voxelFilterOriginal.getCentroidIndexAt (ijk_in_Original);

                    if(index!=-1)
                    {
                        MatchedVoxels++;
                    }
                }

            }
        }
    }

    //calculating the coverage percentage
    float coverage_ratio= MatchedVoxels/OriginalVoxelsSize;
    float coverage_percentage= coverage_ratio*100;

    //    std::cout<<" the coverage ratio is = "<<coverage_ratio<<"\n";
    //    std::cout<<" the number of covered voxels = "<<MatchedVoxels<<" voxel is covered"<<"\n";
    //    std::cout<<" the number of original voxels = "<<OriginalVoxelsSize<<" voxel"<<"\n\n\n";
    //    std::cout<<" the coverage percentage is = "<<coverage_percentage<<" %"<<"\n";

    ros::Time covpercent_end = ros::Time::now();
    double elapsed =  covpercent_end.toSec() - covpercent_begin.toSec();
    //    std::cout<<"Coverage Percentage Calculation duration (s) = "<<elapsed<<"\n";

    return coverage_percentage;
}

//taken from the original aircraft_inspection package
bool OcclusionCulling::contains(pcl::PointCloud<pcl::PointXYZ> c, pcl::PointXYZ p) {
    pcl::PointCloud<pcl::PointXYZ>::iterator it = c.begin();
    for (; it != c.end(); ++it) {
        if (it->x == p.x && it->y == p.y && it->z == p.z)
            return true;
    }
    return false;
}
//taken from the original aircraft_inspection package
//c2 the cloud that you want to compare with the original cloud in the occlusion culling
pcl::PointCloud<pcl::PointXYZ> OcclusionCulling::pointsDifference(pcl::PointCloud<pcl::PointXYZ> c2) {
    pcl::PointCloud<pcl::PointXYZ> inter;
    pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin();
    for (; it != cloud->end(); ++it) {
        if (!contains(c2, *it))
            inter.push_back(*it);
    }

    return inter;
}

void OcclusionCulling::visualizeFOV(geometry_msgs::Pose location)
{
    //*** visualization the FOV *****
    std::vector<geometry_msgs::Point> fov_points;
    int c_color[3];
    geometry_msgs::Point point1;
    point1.x=fc.fp_bl[0];
    point1.y=fc.fp_bl[1];
    point1.z=fc.fp_bl[2];
    fov_points.push_back(point1);//0
    point1.x=fc.fp_br[0];
    point1.y=fc.fp_br[1];
    point1.z=fc.fp_br[2];
    fov_points.push_back(point1);//1
    point1.x=fc.fp_tr[0];
    point1.y=fc.fp_tr[1];
    point1.z=fc.fp_tr[2];
    fov_points.push_back(point1);//2
    point1.x=fc.fp_tl[0];
    point1.y=fc.fp_tl[1];
    point1.z=fc.fp_tl[2];
    fov_points.push_back(point1);//3
    point1.x=fc.np_bl[0];
    point1.y=fc.np_bl[1];
    point1.z=fc.np_bl[2];
    fov_points.push_back(point1);//4
    point1.x=fc.np_br[0];
    point1.y=fc.np_br[1];
    point1.z=fc.np_br[2];
    fov_points.push_back(point1);//5
    point1.x=fc.np_tr[0];
    point1.y=fc.np_tr[1];
    point1.z=fc.np_tr[2];
    fov_points.push_back(point1);//6
    point1.x=fc.np_tl[0];
    point1.y=fc.np_tl[1];
    point1.z=fc.np_tl[2];
    fov_points.push_back(point1);//7

    std::vector<geometry_msgs::Point> fov_linesNear;
    fov_linesNear.push_back(fov_points[4]);
    fov_linesNear.push_back(fov_points[5]);
    fov_linesNear.push_back(fov_points[5]);
    fov_linesNear.push_back(fov_points[6]);
    fov_linesNear.push_back(fov_points[6]);
    fov_linesNear.push_back(fov_points[7]);
    fov_linesNear.push_back(fov_points[7]);
    fov_linesNear.push_back(fov_points[4]);
    c_color[0]=1;
    c_color[1]=0;
    c_color[2]=1;
    linesList1 = drawLines(fov_linesNear,id++,c_color);//purple

    std::vector<geometry_msgs::Point> fov_linesFar;
    fov_linesFar.push_back(fov_points[0]);
    fov_linesFar.push_back(fov_points[1]);
    fov_linesFar.push_back(fov_points[1]);
    fov_linesFar.push_back(fov_points[2]);
    fov_linesFar.push_back(fov_points[2]);
    fov_linesFar.push_back(fov_points[3]);
    fov_linesFar.push_back(fov_points[3]);
    fov_linesFar.push_back(fov_points[0]);
    c_color[0]=1;
    c_color[1]=1;
    c_color[2]=0;
    linesList2 = drawLines(fov_linesFar,id++,c_color);//yellow


    std::vector<geometry_msgs::Point> fov_linestop;
    fov_linestop.push_back(fov_points[7]);
    fov_linestop.push_back(fov_points[3]);//top
    fov_linestop.push_back(fov_points[6]);
    fov_linestop.push_back(fov_points[2]);//top
    c_color[0]=0;
    c_color[1]=1;
    c_color[2]=0;
    linesList3 = drawLines(fov_linestop,id++,c_color);//green

    std::vector<geometry_msgs::Point> fov_linesbottom;
    fov_linesbottom.push_back(fov_points[5]);
    fov_linesbottom.push_back(fov_points[1]);//bottom
    fov_linesbottom.push_back(fov_points[4]);
    fov_linesbottom.push_back(fov_points[0]);//bottom
    c_color[0]=0;
    c_color[1]=0;
    c_color[2]=1;
    linesList4 = drawLines(fov_linesbottom,id++,c_color);//blue

    marker_array.markers.push_back(linesList1);
    marker_array.markers.push_back(linesList2);
    marker_array.markers.push_back(linesList3);
    marker_array.markers.push_back(linesList4);
    fov_pub.publish(marker_array);

    visualizeRaycast(location); // For debugging
}

void OcclusionCulling::visualizeRaycast(geometry_msgs::Pose location) {
    int c_color[3];
    c_color[0]=0;
    c_color[1]=1;
    c_color[2]=1;
    visualization_msgs::Marker linesList = drawLines(lineSegments, 0, c_color, 0.02);

    sensor_msgs::PointCloud2 cloud3;
    pcl::toROSMsg(*occupancyGrid, cloud3);

    cloud3.header.frame_id = frame_id;
    cloud3.header.stamp = ros::Time::now();

    occupancy_pub.publish(cloud3);
    ray_pub.publish(linesList);
}

visualization_msgs::Marker OcclusionCulling::drawLines(std::vector<geometry_msgs::Point> links, int id, int c_color[])
{
    return drawLines(links, id, c_color, 0.08);
}

visualization_msgs::Marker OcclusionCulling::drawLines(std::vector<geometry_msgs::Point> links, int id, int c_color[], float scale = 0.08)
{
    visualization_msgs::Marker linksMarkerMsg;
    linksMarkerMsg.header.frame_id= frame_id;
    linksMarkerMsg.header.stamp=ros::Time::now();
    linksMarkerMsg.ns="link_marker";
    linksMarkerMsg.id = id;
    linksMarkerMsg.type = visualization_msgs::Marker::LINE_LIST;
    linksMarkerMsg.scale.x = scale;
    linksMarkerMsg.action  = visualization_msgs::Marker::ADD;
    linksMarkerMsg.lifetime  = ros::Duration(1000);

    std_msgs::ColorRGBA color;
    color.r=(float)c_color[0];
    color.g=(float)c_color[1];
    color.b=(float)c_color[2];
    color.a=1.0f;

    std::vector<geometry_msgs::Point>::iterator linksIterator;
    for(linksIterator = links.begin(); linksIterator != links.end(); linksIterator++)
    {
        linksMarkerMsg.points.push_back(*linksIterator);
        linksMarkerMsg.colors.push_back(color);
    }
    return linksMarkerMsg;
}
