#include <iostream>
#include <class_ViewGenerator.hpp>

#include <tf_conversions/tf_eigen.h>


double getYawFromQuaternion(geometry_msgs::Quaternion gm_q){
    double roll, pitch, yaw;

    tf::Quaternion quat (gm_q.x, gm_q.y, gm_q.z, gm_q.w);
      
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    return yaw;
}

geometry_msgs::Quaternion getQuaternionFromYaw(double yaw){
    geometry_msgs::Quaternion quat;
    
    tf::Quaternion tf_q;
    tf_q = tf::createQuaternionFromYaw(yaw); //Rotate 22.5 deg
    
    quat.x = tf_q.getX();
    quat.y = tf_q.getY();
    quat.z = tf_q.getZ();
    quat.w = tf_q.getW();
    
    return quat;
}


// ==============
// Nearest Neighbor
// ==============
void ViewGenerator_NN::generate(){
    generated_poses.clear();
    
    double currX = currentPose.position.x;
    double currY = currentPose.position.y;
    double currZ = currentPose.position.z;
    double currYaw = getYawFromQuaternion(currentPose.orientation);
    
    if (cloudPtr->points.size() < 0){
        std::cout << "[ViewpointGen::NN] No points in map. Rotating" << std::endl;
        
        Pose p;
        p.position.x = std::numeric_limits<double>::quiet_NaN(); //Set to NaN
        p.position.y = std::numeric_limits<double>::quiet_NaN();;
        p.position.z = std::numeric_limits<double>::quiet_NaN();;
        p.orientation = getQuaternionFromYaw(res_yaw); //Rotate 22.5 deg
        
        generated_poses.push_back(p);
    }
    else{
        std::cout << "[ViewpointGen::NN] Generating 4-D state lattice" << std::endl;
        
        for (int i=-1; i<=1; i+=2){
            Pose p;
            p.position.x = currX + res_x*i;
            p.position.y = currY;
            p.position.z = currZ;
            p.orientation = currentPose.orientation;
            generated_poses.push_back(p);
        }
        
        for (int i=-1; i<=1; i+=2){
            Pose p;
            p.position.x = currX;
            p.position.y = currY + res_y*i;
            p.position.z = currZ;
            p.orientation = currentPose.orientation;
            generated_poses.push_back(p);
        }
        
        for (int i=-1; i<=1; i+=2){
            Pose p;
            p.position.x = currX;
            p.position.y = currY;
            p.position.z = currZ + res_z*i;
            p.orientation = currentPose.orientation;
            generated_poses.push_back(p);
        }
        
        for (int i=-1; i<=1; i+=2){
            Pose p;
            p.position.x = currX;
            p.position.y = currY;
            p.position.z = currZ;
            p.orientation = getQuaternionFromYaw(currYaw + res_yaw*i);
            generated_poses.push_back(p);
        }
        
        std::cout << "[ViewpointGen::NN] Generated " << generated_poses.size() << " poses" << std::endl;
    }
}

// ==============
// Frontier class
// ==============
void ViewGenerator_Frontier::generate(){
    generated_poses.clear();
    
    if (cloudPtr->points.size() < 0){
        std::cout << "[ViewpointGen::Frontier] No points in map. Rotating" << std::endl;
        
        Pose p;
        p.position.x = std::numeric_limits<double>::quiet_NaN(); //Set to NaN
        p.position.y = std::numeric_limits<double>::quiet_NaN();;
        p.position.z = std::numeric_limits<double>::quiet_NaN();;
        p.orientation = getQuaternionFromYaw(res_yaw); //Rotate 22.5 deg
        
        generated_poses.push_back(p);
    }
    else{
        std::cout << "[ViewpointGen::Frontier] Point in map. Generating viewpoint" << std::endl;
    }
}
