#include <odom_logger.h>

/*******************************************************************************
*/
OdomLogger::OdomLogger(ros::NodeHandle nh, ros::NodeHandle nh_private) :
  nh_(nh),
  nh_private_(nh_private),
  lock_(true),
  origin{0.0,0.0,0.0},
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  ROS_INFO("[OdomLogger] Init-ing...");

  // init params
  initParams();

  // init publishers, subscribers, and services
  initPSS();

  ROS_INFO("[%s] Init-ed.",                                   PKG_NAME.c_str());
  ROS_INFO("[%s] To start production of lidar odometry issue",PKG_NAME.c_str());
  ROS_INFO("%*s rosservice call /odom_logger/start", (int)(PKG_NAME.size()+2),"");
}


/*******************************************************************************
*/
OdomLogger::~OdomLogger()
{
  printf("[%s] Destroying OdomLogger\n", PKG_NAME.c_str());
}

/*******************************************************************************
*/
bool
OdomLogger::computeOdom()
{
  // odom <-- base_footprint tf message
  geometry_msgs::TransformStamped::Ptr bf_to_odom_tf_msg =
    boost::make_shared<geometry_msgs::TransformStamped>();
  if (getTransform(odom_frame_id_,base_frame_id_, ros::Time::now(),
      bf_to_odom_tf_msg))
  {
    // odom <-- base_footprint tf
    tf::Transform bf_to_odom_tf;
    tf::transformMsgToTF(bf_to_odom_tf_msg->transform, bf_to_odom_tf);

    // odom_static <-- odom_footprint tf: this is the odometry we want, the
    // odometry that we deserve
    tf::Transform of_to_os_tf = binit_to_odom_tf_.inverse() * bf_to_odom_tf;

    // the odom_static <-- odom_footprint tf in msg form
    geometry_msgs::Transform of_to_os_tf_msg;
    tf::transformTFToMsg(of_to_os_tf, of_to_os_tf_msg);

    // the odom_static <-- odom_footprint tf in msg stamped form
    geometry_msgs::TransformStamped of_to_os_tf_msg_stamped;
    of_to_os_tf_msg_stamped.transform = of_to_os_tf_msg;
    of_to_os_tf_msg_stamped.header.stamp = ros::Time::now();
    of_to_os_tf_msg_stamped.header.frame_id = init_frame_id_;
    of_to_os_tf_msg_stamped.child_frame_id = "odom_footprint";

    // this is it: odom_static <- odom_footprint; after this it's clear sailing
    // from here on out
    tf_broadcaster_.sendTransform(of_to_os_tf_msg_stamped);

    // Return the desired transform `referent_frame_id_` <-- odom_footprint
    logged_transform_ptr_ = boost::make_shared<geometry_msgs::TransformStamped>();
    if (getTransform(referent_frame_id_, "odom_footprint",
       of_to_os_tf_msg_stamped.header.stamp, logged_transform_ptr_))
      return true;
    else
      return false;
  }

  return false;
}


/*******************************************************************************
*/
double
OdomLogger::extractYawFromPose(const geometry_msgs::Pose& pose)
{
  return extractYawFromQuaternion(pose.orientation);
}


/*******************************************************************************
*/
double
OdomLogger::extractYawFromQuaternion(const geometry_msgs::Quaternion& quat)
{
  tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);

  tf::Matrix3x3 mat(q);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  wrapAngle(&yaw);

  return yaw;
}


/*******************************************************************************
* Returns frame1 <-- frame2 transform
*/
bool OdomLogger::getTransform(const std::string &frame1,
  const std::string &frame2, const ros::Time& t,
  geometry_msgs::TransformStamped::Ptr lg)
{
  try
  {
    *lg = tf_buffer_.lookupTransform(frame1, frame2, t, ros::Duration(0.5));
    return true;
  }
  catch (tf2::TransformException ex)
  {
    ROS_WARN("[%s] Could not get transform from", PKG_NAME.c_str());
    ROS_WARN("base frame to %s: %s", frame2.c_str(), ex.what());
    return false;
  }
}


/*******************************************************************************
*/
void
OdomLogger::initParams()
{
  // getParam does not recognise unsigned int
  int int_param;

  // ---------------------------------------------------------------------------
  nh_private_.param<std::string>("pkg_name", PKG_NAME, "OdomLogger");

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("log_path", log_path_))
  {
    ROS_WARN("[%s] no log_path param found; resorting to defaults",
      PKG_NAME.c_str());
    log_path_ = "/home/relief-user0/catkin_ws/src/odom_logger/results/log.txt";
  }

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("path_estimate_topic", path_estimate_topic_))
  {
    ROS_WARN("[%s] no path_estimate_topic param found; resorting to defaults",
      PKG_NAME.c_str());
    path_estimate_topic_ = "/odom_logger/path_estimate";

  }

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("odom_topic", odom_topic_))
  {
    ROS_WARN("[%s] no odom_topic param found; resorting to defaults",
      PKG_NAME.c_str());
    odom_topic_ = "/odom";
  }

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("global_frame_id", global_frame_id_))
  {
    ROS_WARN("[%s] no global_frame_id param found; resorting to defaults",
      PKG_NAME.c_str());
    global_frame_id_ = "map";
  }

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("base_frame_id", base_frame_id_))
  {
    ROS_WARN("[%s] no base_frame_id param found; resorting to defaults",
      PKG_NAME.c_str());
    base_frame_id_ = "base_footprint";
  }

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("odom_frame_id", odom_frame_id_))
  {
    ROS_WARN("[%s] no odom_frame_id param found; resorting to defaults",
      PKG_NAME.c_str());
    odom_frame_id_ = "odom";
  }

  // ---------------------------------------------------------------------------
  // This is the static odometry frame. AKA the pose of base_footprint
  // after amcl has established the pose of the robot (after moving the robot
  // slightly so that the pose estimate of amcl has converged)
  if (!nh_private_.getParam ("init_frame_id", init_frame_id_))
  {
    ROS_WARN("[%s] no init_frame_id param found; resorting to defaults",
      PKG_NAME.c_str());
    init_frame_id_ = "odom_static";
  }

  // ---------------------------------------------------------------------------
  // This is the reference frame of the ouput
  if (!nh_private_.getParam ("referent_frame_id", referent_frame_id_))
  {
    ROS_WARN("[%s] no referent_frame_id param found; resorting to defaults",
      PKG_NAME.c_str());
    referent_frame_id_ = "map";
  }

  // Create the logfile
  std::ofstream logfile(log_path_.c_str());

  if (logfile.is_open())
    logfile.close();
  else
    ROS_ERROR("[OdomLogger] logpath file not open");
}


/*******************************************************************************
*/
void
OdomLogger::initPSS()
{
  // The subscriber to the input scans topic
  odom_sub_ =
    nh_.subscribe(odom_topic_, 1, &OdomLogger::odomCallback, this);

  // Clearing the estimated trajectory service
  clear_trajectory_service_ = nh_.advertiseService(
    "odom_logger/clear_estimated_trajectory", &OdomLogger::serviceClearTrajectory, this);

  // Initial pose setting service
  set_bf_to_map_init_service_ = nh_.advertiseService(
    "odom_logger/set_initial_pose", &OdomLogger::serviceInitialPose, this);

  // Start service
  start_service_= nh_.advertiseService(
    "odom_logger/start", &OdomLogger::serviceStart, this);

  // Stop service
  stop_service_= nh_.advertiseService(
    "odom_logger/stop", &OdomLogger::serviceStop, this);

  // the path estimate
  path_estimate_pub_ =
    nh_.advertise<nav_msgs::Path>(path_estimate_topic_, 1);
}


/*******************************************************************************
 * The current pose estimate with respect to the `referent_frame_id`
 */
void
OdomLogger::logOdomPose()
{
  std::string prefix;

  if (static_cast<double>(logged_transform_ptr_->header.stamp.nsec) / 10 < 1)
    prefix = "00000000";
  else if (static_cast<double>(logged_transform_ptr_->header.stamp.nsec) / 100 < 1)
    prefix = "0000000";
  else if (static_cast<double>(logged_transform_ptr_->header.stamp.nsec) / 1000 < 1)
    prefix = "000000";
  else if (static_cast<double>(logged_transform_ptr_->header.stamp.nsec) / 10000 < 1)
    prefix = "00000";
  else if (static_cast<double>(logged_transform_ptr_->header.stamp.nsec) / 100000 < 1)
    prefix = "0000";
  else if (static_cast<double>(logged_transform_ptr_->header.stamp.nsec) / 1000000 < 1)
    prefix = "000";
  else if (static_cast<double>(logged_transform_ptr_->header.stamp.nsec) / 10000000 < 1)
    prefix = "00";
  else if (static_cast<double>(logged_transform_ptr_->header.stamp.nsec) / 100000000 < 1)
    prefix = "0";

  std::string nsec_str = prefix + std::to_string(logged_transform_ptr_->header.stamp.nsec);

  std::ofstream pose_file(log_path_.c_str(), std::ios::app);

  if (pose_file.is_open())
  {
    pose_file << logged_transform_ptr_->header.stamp.sec;
    pose_file << ".";
    pose_file << nsec_str;
    pose_file << ", ";
    pose_file << logged_transform_ptr_->transform.translation.x;
    pose_file << ", ";
    pose_file << logged_transform_ptr_->transform.translation.y;
    pose_file << ", ";
    pose_file << extractYawFromQuaternion(logged_transform_ptr_->transform.rotation);
    pose_file << std::endl;

    pose_file.close();
  }
}


/*******************************************************************************
 * The total path estimate with respect to the global frame
 */
void
OdomLogger::publishOdomPathMessage()
{
  // Construct trajectory estimate message and publish it
  path_estimate_msg_.header.stamp = logged_transform_ptr_->header.stamp;
  path_estimate_msg_.header.frame_id = logged_transform_ptr_->header.frame_id;

  geometry_msgs::PoseStamped gmp;
  gmp.header.stamp = logged_transform_ptr_->header.stamp;
  gmp.header.frame_id = logged_transform_ptr_->header.frame_id;

  gmp.pose.position.x = logged_transform_ptr_->transform.translation.x;
  gmp.pose.position.y = logged_transform_ptr_->transform.translation.y;
  gmp.pose.position.z = 0.0;

  gmp.pose.orientation = logged_transform_ptr_->transform.rotation;
  path_estimate_msg_.poses.push_back(gmp);
  path_estimate_pub_.publish(path_estimate_msg_);
}


/*******************************************************************************
*/
  geometry_msgs::Pose
OdomLogger::retypePose(const std::tuple<double,double,double>& pose)
{
  geometry_msgs::Pose pose_msg;

  // Set position
  pose_msg.position.x = std::get<0>(pose);
  pose_msg.position.y = std::get<1>(pose);
  pose_msg.position.z = 0.0;

  // Set orientation
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, std::get<2>(pose));
  q.normalize();
  tf::quaternionTFToMsg(q, pose_msg.orientation);

  return pose_msg;
}


/*******************************************************************************
*/
  geometry_msgs::PoseStamped
OdomLogger::retypePoseStamped(const std::tuple<double,double,double>& pose,
  const std::string& frame_id)
{
  geometry_msgs::PoseStamped posestamped_msg;
  posestamped_msg.header.stamp = ros::Time::now();
  posestamped_msg.header.frame_id = frame_id;

  // Set pose
  posestamped_msg.pose = retypePose(pose);

  return posestamped_msg;
}


/*******************************************************************************
 *
 */
  bool
OdomLogger::serviceClearTrajectory(std_srvs::Empty::Request &req,
  std_srvs::Empty::Response &res)
{
  ROS_INFO("[%s] Clearing trajectory ...", PKG_NAME.c_str());
  lock_ = true;
  path_estimate_msg_.poses.clear();
  lock_ = false;

  return true;
}


/*******************************************************************************
 * If there is an initial pose then set it
 */
  bool
OdomLogger::serviceInitialPose(std_srvs::Empty::Request &req,
  std_srvs::Empty::Response &res)
{
  bool success = true;

  // ---------------------------------------------------------------------------
  // map <-- base_footprint (init)
  geometry_msgs::TransformStamped::Ptr bf_to_map_tf_msg =
    boost::make_shared<geometry_msgs::TransformStamped>();
  if (getTransform(global_frame_id_,base_frame_id_, ros::Time::now(),
      bf_to_map_tf_msg))
  {
    double dx = bf_to_map_tf_msg->transform.translation.x;
    double dy = bf_to_map_tf_msg->transform.translation.y;
    double dt = extractYawFromQuaternion(bf_to_map_tf_msg->transform.rotation);

    // The transform of `base_frame_id` to the `global_frame_id` (map)
    // at the time when initialisation of odometry happens.
    tf::transformMsgToTF(bf_to_map_tf_msg->transform,  binit_to_map_tf_);

    // Publish as transform: init_frame_id to map
    geometry_msgs::TransformStamped init_to_map_tf_msg_stamped;
    init_to_map_tf_msg_stamped.transform = bf_to_map_tf_msg->transform;
    init_to_map_tf_msg_stamped.header.stamp = ros::Time::now();
    init_to_map_tf_msg_stamped.header.frame_id = global_frame_id_;
    init_to_map_tf_msg_stamped.child_frame_id = init_frame_id_;

    tfstatic_broadcaster_.sendTransform(init_to_map_tf_msg_stamped);
  }
  else
    success = false;

  // ---------------------------------------------------------------------------
  // odom <-- base_footprint (or else odom <-- odom_static)---(init)
  geometry_msgs::TransformStamped::Ptr bf_to_odom_tf_msg =
    boost::make_shared<geometry_msgs::TransformStamped>();
  if (getTransform(odom_frame_id_,base_frame_id_, ros::Time::now(),
      bf_to_odom_tf_msg))
  {
    double dx = bf_to_odom_tf_msg->transform.translation.x;
    double dy = bf_to_odom_tf_msg->transform.translation.y;
    double dt = extractYawFromQuaternion(bf_to_odom_tf_msg->transform.rotation);

    tf::transformMsgToTF(bf_to_odom_tf_msg->transform, binit_to_odom_tf_);
  }
  else
    success = false;

  return success;
}


/*******************************************************************************
 *
 */
  bool
OdomLogger::serviceStart(std_srvs::Empty::Request &req,
  std_srvs::Empty::Response &res)
{
  ROS_INFO("[%s] odometry is now being logged.",    PKG_NAME.c_str());
  ROS_INFO("[%s] To shut down issue",               PKG_NAME.c_str());
  ROS_INFO("%*s rosservice call /odom_logger/stop", (int)(PKG_NAME.size()+2),"");
  lock_ = false;

  return true;
}


/*******************************************************************************
 *
 */
  bool
OdomLogger::serviceStop(std_srvs::Empty::Request &req,
  std_srvs::Empty::Response &res)
{
  ROS_INFO("[%s] odometry logging is shut down.",    PKG_NAME.c_str());
  ROS_INFO("[%s] To bring up issue",                 PKG_NAME.c_str());
  ROS_INFO("%*s rosservice call /odom_logger/start", (int)(PKG_NAME.size()+2),"");
  lock_ = true;

  return true;
}


/*******************************************************************************
*/
  void
OdomLogger::wrapAngle(double* angle)
{
  *angle = fmod(*angle + 5*M_PI, 2*M_PI) - M_PI;
}


/*******************************************************************************
*/
  void
OdomLogger::odomCallback(const nav_msgs::Odometry::Ptr& odom_msg)
{
  if (lock_)
    return;
  else
    lock_ = true;

  // `referent_frame_id_` <-- odom_footprint
  if(computeOdom())
  {
    logOdomPose();
    publishOdomPathMessage();
  }

  // Unlock
  lock_ = false;
}
