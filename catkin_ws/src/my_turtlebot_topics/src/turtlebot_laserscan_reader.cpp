#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>

using namespace std;

class TurtleBotLaserScanReader {
protected:
  // NodeHandle instance must be created before this line. Otherwise strange
  // error occurs.

  int rate_hz_;
  vector<float> ranges_;

  ros::Rate *rate_;
  ros::Subscriber sub_laser_scan_odom_;

public:
  TurtleBotLaserScanReader(ros::NodeHandle *nh);
  void topic_callback(const sensor_msgs::LaserScan::ConstPtr &laserMsg);

  ~TurtleBotLaserScanReader(void) {}
};

TurtleBotLaserScanReader::TurtleBotLaserScanReader(ros::NodeHandle *nh) {

  // ROS Service
  sub_laser_scan_odom_ = nh->subscribe(
      "/kobuki/laser/scan", 1000, &TurtleBotLaserScanReader::topic_callback, this);

  rate_hz_ = 1;
  rate_ = new ros::Rate(rate_hz_);
}

void TurtleBotLaserScanReader::topic_callback(
    const sensor_msgs::LaserScan::ConstPtr &laserMsg) {
  ranges_ = laserMsg->ranges;
}

int main(int argc, char **argv) {
  string node_name = "turtle_bot_laserscan_reader_node";
  ros::init(argc, argv, node_name);

  ros::NodeHandle nh;
  TurtleBotLaserScanReader turtleBotLaserScanReader(&nh);

  ros::spin();

  return 0;
}