#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

using namespace std;

class TurtleBotOdometryReader {
protected:
  // NodeHandle instance must be created before this line. Otherwise strange
  // error occurs.

  int rate_hz_;
  float position_x_;
  float position_y_;
  float orientation_z_;

  ros::Rate *rate_;
  ros::Subscriber sub_odometry_;

public:
  TurtleBotOdometryReader(ros::NodeHandle *nh);
  void topic_callback(const nav_msgs::Odometry::ConstPtr &odometry);

  ~TurtleBotOdometryReader(void) {}
};

TurtleBotOdometryReader::TurtleBotOdometryReader(ros::NodeHandle *nh) {
  // ROS Service
  sub_odometry_ =
      nh->subscribe("/odom", 1000, &TurtleBotOdometryReader::topic_callback, this);
  rate_hz_ = 1;
  rate_ = new ros::Rate(rate_hz_);
}

void TurtleBotOdometryReader::topic_callback(
    const nav_msgs::Odometry::ConstPtr &odometry) {
  position_x_ = odometry->pose.pose.position.x;
  position_y_ = odometry->pose.pose.position.y;
  orientation_z_ = odometry->pose.pose.orientation.z;
  //   ROS_INFO("%f, %f, %f: ", odometry_position_x_, odometry_position_y_,
  //            odometry_orientation_z_);
}

int main(int argc, char **argv) {
  string node_name = "turtle_bot_odometry_reader_node";
  ros::init(argc, argv, node_name);

  ros::NodeHandle nh;
  TurtleBotOdometryReader turtleBotOdometryReader(&nh);

  ros::spin();

  return 0;
}