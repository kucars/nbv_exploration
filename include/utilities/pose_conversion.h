#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <octomap/octomap.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

#include <nbv_exploration/common.h>

namespace pose_conversion{
  // ================
  // Point conversion
  // ================
  static inline Eigen::Vector3d convertToEigenVector(geometry_msgs::Point p_in)
  {
    Eigen::Vector3d v (p_in.x, p_in.y, p_in.z);
    return v;
  }

  static inline Eigen::Vector3d convertToEigenVector(geometry_msgs::Pose  p_in)
  {
    Eigen::Vector3d v = convertToEigenVector(p_in.position);
    return v;
  }

  static inline Eigen::Vector3d convertToEigenVector(octomap::point3d p_in)
  {
    Eigen::Vector3d v (p_in.x(), p_in.y(), p_in.z());
    return v;
  }

  static inline Eigen::Vector3d convertToEigenVector(pcl::PointXYZRGB  p_in)
  {
    Eigen::Vector3d v (p_in.x, p_in.y, p_in.z);
    return v;
  }

  static inline geometry_msgs::Point convertToGeometryMsgPoint(octomap::point3d p_in)
  {
    geometry_msgs::Point p;
    p.x = p_in.x(); p.y = p_in.y(); p.z = p_in.z();
    
    return p;
  }

  static inline geometry_msgs::Point convertToGeometryMsgPoint(Eigen::Vector3d p_in)
  {
    geometry_msgs::Point p;
    p.x = p_in[0]; p.y = p_in[1]; p.z = p_in[2];
    return p;
  }

  static inline geometry_msgs::Point convertToGeometryMsgPoint(pcl::PointXYZRGB p_in)
  {
    geometry_msgs::Point p;
    p.x = p_in.x; p.y = p_in.y; p.z = p_in.z;
    return p;
  }

  static inline octomap::point3d convertToOctomapPoint(geometry_msgs::Point p_in)
  {
    octomap::point3d p (p_in.x, p_in.y, p_in.z);
    return p;
  }

  static inline octomap::point3d convertToOctomapPoint(geometry_msgs::Pose p_in)
  {
    octomap::point3d p = convertToOctomapPoint(p_in.position);
    return p;
  }

  static inline octomap::point3d convertToOctomapPoint(Eigen::Vector3d p_in){
    octomap::point3d p (p_in[0], p_in[1], p_in[2]);
    return p;
  }

  static inline octomap::point3d convertToOctomapPoint(pcl::PointXYZRGB p_in)
  {
    octomap::point3d p (p_in.x, p_in.y, p_in.z);
    return p;
  }

  static inline pcl::PointXYZRGB convertToPclPoint(Eigen::Vector3d p_in)
  {
    pcl::PointXYZRGB p;
    p.x = p_in[0]; p.y = p_in[1]; p.z = p_in[2];
    return p;
  }

  static inline pcl::PointXYZRGB convertToPclPoint(geometry_msgs::Point p_in)
  {
    pcl::PointXYZRGB p;
    p.x = p_in.x; p.y = p_in.y; p.z = p_in.z;
    return p;
  }

  static inline pcl::PointXYZRGB convertToPclPoint(geometry_msgs::Pose  p_in)
  {
    pcl::PointXYZRGB p = convertToPclPoint(p_in.position);
    return p;
  }

  static inline pcl::PointXYZRGB convertToPclPoint(octomap::point3d p_in)
  {
    pcl::PointXYZRGB p;
    p.x = p_in.x(); p.y = p_in.y(); p.z = p_in.z();
    return p;
  }




  // ================
  // Rotation conversion
  // ================
  static inline Eigen::Matrix3d getRotationMatrix(tf::StampedTransform t)
  {
    tf::Quaternion qt = t.getRotation();
    tf::Matrix3x3 R1(qt);
    
    Eigen::Matrix3d R;
    tf::matrixTFToEigen(R1,R);
    
    return R;
  }
  
  static inline Eigen::Matrix3d getRotationMatrix(geometry_msgs::Quaternion q_in)
  {
    tf::Quaternion qt(q_in.x, q_in.y, q_in.z, q_in.w);
    tf::Matrix3x3 R1(qt);
    
    Eigen::Matrix3d R;
    tf::matrixTFToEigen(R1,R);
    
    return R;
  }
  
  static inline Eigen::Matrix3d getRotationMatrix(geometry_msgs::Pose p)
  {
    Eigen::Matrix3d R = getRotationMatrix(p.orientation);
    
    return R;
  }
  
  static inline Eigen::Matrix4d convertStampedTransform2Matrix4d(tf::StampedTransform t)
  {
    Eigen::Matrix4d tf_eigen;
    
    Eigen::Vector3d T1(
        t.getOrigin().x(),
        t.getOrigin().y(),
        t.getOrigin().z()
    );
    
    Eigen::Matrix3d R = getRotationMatrix(t);
    
    // Set
    tf_eigen.setZero ();
    tf_eigen.block (0, 0, 3, 3) = R;
    tf_eigen.block (0, 3, 3, 1) = T1;
    tf_eigen (3, 3) = 1;
    
    return tf_eigen;
  }
  
  static inline Eigen::Quaterniond convertToEigenQuaternoid(geometry_msgs::Quaternion p)
  {
    Eigen::Quaterniond q(p.x, p.y, p.z, p.w); 
    return q;
    
    //tf::Matrix3x3(q1).getRPY(roll1, pitch1, yaw1);
  }

  static inline Eigen::Quaterniond convertToEigenQuaternoid(geometry_msgs::Pose p)
  {
    Eigen::Quaterniond q = convertToEigenQuaternoid(p.orientation); 
    return q;
    
    //tf::Matrix3x3(q1).getRPY(roll1, pitch1, yaw1);
    //Eigen::Matrix3d = q.toRotationMatrix();
  }

  // ================
  // Angle conversion
  // ================

  static inline double getYawFromQuaternion(geometry_msgs::Quaternion gm_q)
  {
    double roll, pitch, yaw;

    tf::Quaternion quat (gm_q.x, gm_q.y, gm_q.z, gm_q.w);
      
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    return yaw;
  }

  static inline geometry_msgs::Quaternion getQuaternionFromYaw(double yaw)
  {
    geometry_msgs::Quaternion quat;
    
    tf::Quaternion tf_q;
    tf_q = tf::createQuaternionFromYaw(yaw);
    
    quat.x = tf_q.getX();
    quat.y = tf_q.getY();
    quat.z = tf_q.getZ();
    quat.w = tf_q.getW();
    
    return quat;
  }

  // ================
  // Direction Vector
  // ================
  static inline Eigen::Vector3d getDirectionVectorFromTransform(tf::StampedTransform transform)
  {
    // Camera direction
    Eigen::Vector3d T1(0,0,1);
    
    // Get pose rotation
    tf::Quaternion qt = transform.getRotation();
    tf::Matrix3x3 R1(qt);
    Eigen::Matrix3d R;
    tf::matrixTFToEigen(R1,R);
    
    // Get direction vector
    T1 = R*T1;
    T1.normalize();
    
    return T1;
  }

  static inline octomap::point3d getOctomapDirectionVectorFromTransform(tf::StampedTransform transform)
  {
    Eigen::Vector3d T1 = getDirectionVectorFromTransform(transform);
    
    octomap::point3d p(T1.x(), T1.y(), T1.z());
    return p;
  }
}
