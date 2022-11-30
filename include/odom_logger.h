#ifndef ODOM_LOGGER_H
#define ODOM_LOGGER_H

#include <memory>
#include <chrono>
#include <signal.h>
#include <cmath>
#include <numeric>
#include <iostream>
#include <fstream>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <time.h>
#include <tuple>
#include <functional>
#include <eigen3/Eigen/Geometry>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>


class OdomLogger
{
  public:

    OdomLogger(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~OdomLogger();

  private:

    // Nodehandles
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // Name of this package
    std::string PKG_NAME;


    // Odom topic subscriber
    ros::Subscriber odom_sub_;

    // Odom reset publisher
    ros::Publisher odom_reset_pub_;

    // Path estimate publisher
    ros::Publisher path_estimate_pub_;


    // Clearing the estimated trajectory service
    ros::ServiceServer clear_trajectory_service_;

    // Initial pose setting service
    ros::ServiceServer set_bf_to_map_init_service_;

    // Starts lidar odometry
    ros::ServiceServer start_service_;

    // Stops lidar odometry
    ros::ServiceServer stop_service_;


    // The topic where odom messages crash
    std::string odom_topic_;

    // The topic where odometry is reset
    std::string odometry_reset_topic_;

    // The topic where the initial pose may be provided (optional)
    std::string bf_to_map_init_topic_;

    // The topic where fsm's path estimate is published
    std::string path_estimate_topic_;

    // Params

    // Locks callback execution
    bool lock_;

    // Previous odom receipt time
    ros::Time tv_;

    // Current odom receipt time
    ros::Time tr_;

    // The origin (0,0,0)
    std::tuple<double,double,double> origin;

    // initial pose
    std::tuple<double,double,double> bf_to_map_init_;

    // initial odometry
    std::tuple<double,double,double> bf_to_odom_init_;

    // The transform
    Eigen::Matrix3d M;

    // The path estimate
    std::vector< std::tuple<double,double,double> > path_estimate_;

    // The path estimate
    nav_msgs::Path path_estimate_msg_;

    // Map frame name
    std::string global_frame_id_;

    // Base frame name
    std::string base_frame_id_;

    // The init frame name
    std::string init_frame_id_;

    // Lidar odometry frame name
    std::string odom_frame_id_;

    // log path
    std::string log_path_;

    // The big guns
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster tfstatic_broadcaster_;

    tf::Transform binit_to_map_tf_;
    tf::Transform binit_to_odom_tf_;

    // **** methods

    Eigen::Matrix3d computeTransform(const std::tuple<double,double,double>& d,
      const Eigen::Matrix3d& M);

    double extractYawFromPose( const geometry_msgs::Pose& pose);
    double extractYawFromQuaternion(const geometry_msgs::Quaternion& quat);

    bool getTransform(
      const std::string &frame1,
      const std::string &frame2,
      geometry_msgs::TransformStamped* lg);


    void initParams();
    void initPSS();

    void logOdom();
    void logOdomPose();

    Eigen::Matrix3d poseToTransformMatrix(
      const std::tuple<double,double,double>& pose);

    void publishOdomPathMessage();

    geometry_msgs::Pose retypePose(
      const std::tuple<double,double,double>& pose);

    geometry_msgs::PoseStamped retypePoseStamped(
      const std::tuple<double,double,double>& pose,
      const std::string& frame_id);

    bool serviceClearTrajectory(
      std_srvs::Empty::Request &req,
      std_srvs::Empty::Response &res);
    bool serviceInitialPose(
      std_srvs::Empty::Request &req,
      std_srvs::Empty::Response &res);
    bool serviceStart(
      std_srvs::Empty::Request &req,
      std_srvs::Empty::Response &res);
    bool serviceStop(
      std_srvs::Empty::Request &req,
      std_srvs::Empty::Response &res);

    std::tuple<double,double,double> transformMatrixToPose(
      const Eigen::Matrix3d& M);

    void odomCallback(const nav_msgs::Odometry::Ptr& odom_msg);
    void wrapAngle(double* angle);


};

#endif // ODOM_LOGGER_H
