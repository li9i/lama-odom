#include <odom_logger.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_logger");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  OdomLogger l(nh, nh_private);

  ros::spin();
  return 0;
}
