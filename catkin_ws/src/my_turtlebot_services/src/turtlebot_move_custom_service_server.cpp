#include "geometry_msgs/Twist.h"
#include <my_turtlebot_services/MyCustomTrigger.h>
#include <ros/ros.h>
#include <string>
#include <unistd.h>
#include <vector>

using namespace std;
// enum class State {moveDown, moveLeft, moveUp, moveRight};
// struct Direction {
//   enum type { MOVEDOWN, MOVELEFT, MOVEUP, MOVERIGHT };
// };

class MoveTurtleBot {

public:
  MoveTurtleBot(ros::NodeHandle *nh);
  bool my_callback(my_turtlebot_services::MyCustomTrigger::Request &req,
                   my_turtlebot_services::MyCustomTrigger::Response &res);
  geometry_msgs::Twist GetVelocity(int direction);
  void rateSleep(void);
    void Navigate(void);
    void ChangeState(int state, float duration);
  ~MoveTurtleBot(void);

private:
  bool is_running_;

  // ROS objects
  // ros::NodeHandle nh_;
  ros::Rate *rate_;

  // ROS ServiceServer
  ros::ServiceServer srv_perform_square_;

  // ROS Publishers
  ros::Publisher pub_cmd_vel_;

  bool running_;
  int side_;
  int repetitions_;
  double duration_;
  float rate_hz_;
  int state_;
  float vel_;
  float scalling_factor_; // adjust square side
  vector<string> directions_;

  geometry_msgs::Twist twist_;
};

MoveTurtleBot::MoveTurtleBot(ros::NodeHandle *nh) {
  running_ = false;
  repetitions_ = 0;
  vel_ = 0.2;
  duration_ = 0.0;
  rate_hz_ = 20.0;
  state_ = 0;
  scalling_factor_ = 1.2;
  side_ = 0;

  // ROS Publisher
  pub_cmd_vel_ = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  // ROS Service
  srv_perform_square_ =
  nh->advertiseService("/move_turtle_bot_in_maze_custom",
                                             &MoveTurtleBot::my_callback,
                                             this);
  // ROS objects
  rate_ = new ros::Rate(rate_hz_);
  directions_.push_back("MOVEDOWN");
  directions_.push_back("MOVELEFT");
  directions_.push_back("MOVEUP");
  directions_.push_back("MOVERIGHT");
}

void MoveTurtleBot::Navigate(void) {
  geometry_msgs::Twist vel;

  if (!running_) {
    vel.linear.x = 0;
    vel.angular.z = 0;
    pub_cmd_vel_.publish(vel);
    return;
  }

  duration_ -= 1 / (float)rate_hz_;
  ROS_INFO("Vel[%.2f, %.2f], Duration[%.2lf]]", vel.linear.x, vel.angular.z,
           duration_);
  //   if (duration_ <= 0) {
  //     //     float state_duration[4] = {side_, 3.8, 4.0, 0.1};
  //     //     int next_state = state_ + 1;
  //     //     if (state_ == 3) {
  //     //       next_state = 0;
  //     //       times_ -= 1;
  //     //     }
  //     //     int next_state_duration = state_duration[next_state];
  //     //     this->changeState(next_state, next_state_duration);
  //     return;
  //   }

  //   if (times_ == 0) {
  //     running_ = false;
  //     vel.linear.x = 0;
  //     vel.angular.z = 0;
  //     pub_cmd_vel_.publish(vel);
  //   }
}

void MoveTurtleBot::ChangeState(int state, float duration) {
  duration_ = duration;
}

bool MoveTurtleBot::my_callback(
    my_turtlebot_services::MyCustomTrigger::Request &req,
    my_turtlebot_services::MyCustomTrigger::Response &res) {

  //   running_ = !running_;
  //   side_ = req.side;
  //   repetitions_ = req.repetitions;

  //   if (!running_) {
  //     twist_.linear.x = 0;
  //     twist_.angular.z = 0;
  //     ROS_INFO("Finished service move_turtle_bot_in_maze_custom_server -
  //     stop"); pub_cmd_vel_.publish(twist_);

  //     res.success = true;
  //     return res.success;
  //   }

  //   float delta_ = 1 / (float)rate_hz_;

  //   for (int i = 0; i < directions_.size(); i++) {
  //     twist_ = this->GetVelocity(i);

  //     float distance_delta = side_;
  //     // ROS_INFO("deltas[%.2f, %.2f]", delta_, distance_delta);
  //     while (distance_delta >= 0) {
  //       ROS_INFO("direction[%d], Vel[%.2f, %.2f]", i, twist_.linear.x,
  //                twist_.angular.z);
  //       pub_cmd_vel_.publish(twist_);
  //       distance_delta -= delta_;
  //       this->rateSleep();
  //     }
  //   }

  //   ROS_INFO("Vel[%.2f, %.2f], Duration[%.2lf]]", twist_.linear.x,
  //            twist_.angular.z, duration_);

  res.movement_successfull = true;

  //   ROS_INFO("Finished service move_turtle_bot_in_maze_custom_server");
  return res.movement_successfull;
}

geometry_msgs::Twist MoveTurtleBot::GetVelocity(int direction) {
  geometry_msgs::Twist twist_msg;

  //   switch (direction) {
  //   case 0:
  //     twist_msg.linear.x = 0.0;
  //     twist_msg.angular.z = 0.2;
  //     // ROS_INFO("direction[%d], Vel[%.2f, %.2f]", direction,
  //     twist_msg.linear.x,
  //     //          twist_msg.angular.z);
  //     break;
  //   case 1:
  //     twist_msg.linear.x = -0.2;
  //     twist_msg.angular.z = 0.0;
  //     // ROS_INFO("direction[%d], Vel[%.2f, %.2f]", direction,
  //     twist_msg.linear.x,
  //     //          twist_msg.angular.z);
  //     break;
  //   case 2:
  //     twist_msg.linear.x = 0.0;
  //     twist_msg.angular.z = -0.2;
  //     // ROS_INFO("direction[%d], Vel[%.2f, %.2f]", direction,
  //     twist_msg.linear.x,
  //     //          twist_msg.angular.z);
  //     break;
  //   case 3:
  //     twist_msg.linear.x = 0.2;
  //     twist_msg.angular.z = 0.0;
  //     // ROS_INFO("direction[%d], Vel[%.2f, %.2f]", direction,
  //     twist_msg.linear.x,
  //     //          twist_msg.angular.z);
  //     break;
  //   default:
  //     // ROS_INFO("direction[%d], Vel[%.2f, %.2f]", direction,
  //     twist_msg.linear.x,
  //     //          twist_msg.angular.z);
  //     break;
  //   }

  return twist_msg;
}

void MoveTurtleBot::rateSleep(void) { rate_->sleep(); }

MoveTurtleBot::~MoveTurtleBot(void) {}

int main(int argc, char **argv) {
  ros::init(argc, argv, "move_turtle_bot_in_maze_custom_server");
  ros::NodeHandle nh;
  MoveTurtleBot moveTurtleBot(&nh);

  // moveTurtleBot.Navigate();
  ros::spin();

  //   while (ros::ok()) {
  //     moveBB8.navigateBB8();
  //     moveBB8.rateSleep();
  //     ros::spinOnce();
  //   }

  return 0;
}