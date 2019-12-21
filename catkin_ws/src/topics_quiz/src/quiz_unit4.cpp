#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

float linX, angZ;

void counterCallback(const sensor_msgs::LaserScan::ConstPtr &laserMsg) {
  // ROS_INFO("%lu", laserMsg->ranges.size());

  // if front is free move forward
  if (laserMsg->ranges[360] > 1) {
    linX = 0.1;
    angZ = 0.0;
  }
  if (laserMsg->ranges[360] < 1) {
    // obstacle in front, turn left
    linX = 0.0;
    angZ = 0.2;
  }
  if (laserMsg->ranges[719] < 0.3) {
    // obstacle in left, turn right
    linX = 0.0;
    angZ = -0.2;
  }
  if (laserMsg->ranges[0] < 0.3) {
    // obstacle rigth, tur left
    linX = 0.0;
    angZ = 0.2;
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "topics_quiz_node");
  ros::NodeHandle nh;

  ros::Subscriber sub =
      nh.subscribe("/kobuki/laser/scan", 1000, counterCallback);

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Rate loop_rate(2);

  geometry_msgs::Twist twistMsg; // Create a variable of type Int32

  while (ros::ok()) // Create a loop that will go until someone stops the
                    // program execution
  {
    pub.publish(twistMsg); // Publish the message within the 'count' variable
    twistMsg.linear.x = linX;
    twistMsg.angular.z = angZ;

    ros::spinOnce();
    loop_rate.sleep(); // Make sure the publish rate maintains at 2 Hz
  }

  return 0;
}