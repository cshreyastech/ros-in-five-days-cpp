#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

using namespace std;

class TurtleBotTwistDriver {
protected:
  // NodeHandle instance must be created before this line. Otherwise strange
  // error occurs.

  int rate_hz_;

  ros::Rate *rate_;
  ros::Publisher pub_cmd_vel_;
  geometry_msgs::Twist twistMsg_;

public:
  TurtleBotTwistDriver(ros::NodeHandle *nh);
  void topic_callback(const geometry_msgs::Twist::ConstPtr &twistMsg);
  void publishVelocity(float position_x, float position_y, float orientation_z);

  ~TurtleBotTwistDriver(void) {}
};

TurtleBotTwistDriver::TurtleBotTwistDriver(ros::NodeHandle *nh) {

  // ROS Publisher
  pub_cmd_vel_ = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  //,&TurtleBotTwistDriver::topic_callback, this);
  // pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  rate_hz_ = 1;
  rate_ = new ros::Rate(rate_hz_);
}

void TurtleBotTwistDriver::topic_callback(
    const geometry_msgs::Twist::ConstPtr &laserMsg) {
  //   ranges_ = laserMsg->ranges;
}

void TurtleBotTwistDriver::publishVelocity(float position_x, float position_y,
                                           float orientation_z) {
  twistMsg_.linear.x = position_x;
  twistMsg_.linear.y = position_y;
  twistMsg_.angular.z = orientation_z;

  pub_cmd_vel_.publish(
      twistMsg_); // Publish the message within the 'count' variable

  ROS_INFO("%f, %f, %f: ", twistMsg_.linear.x, twistMsg_.linear.y,
           orientation_z);
  // ros::spinOnce();
}

int main(int argc, char **argv) {
  string node_name = "turtle_bot_twist_driver_node";
  ros::init(argc, argv, node_name);

  ros::NodeHandle nh;
  TurtleBotTwistDriver turtleBotTwistDriver(&nh);

  ros::Rate loop_rate(2);
  while (ros::ok()) {
    turtleBotTwistDriver.publishVelocity(0.0, 0.0, 0.0);
    ros::spinOnce();
    loop_rate.sleep(); // Make sure the publish rate maintains at 2 Hz
  }
  //   turtleBotTwistDriver.publishVelocity(0.0, 0.0, 0.0);
  // turtleBotTwistDriver.topic_callback(const geometry_msgs::Twist::ConstPtr
  // &laserMsg)
  return 0;
}