#include "geometry_msgs/Twist.h"
#include <ros/ros.h>
#include <services_quiz/BB8CustomServiceMessage.h>
#include <unistd.h>

using namespace std;

class MoveBB8 {

public:
  MoveBB8();
  bool my_callback(services_quiz::BB8CustomServiceMessage::Request &req,
                   services_quiz::BB8CustomServiceMessage::Response &res);
  geometry_msgs::Twist getStateVelocity();
  void rateSleep(void);
  void runTimeStateMachine(void);
  void changeState(int state, float duration);
  ~MoveBB8(void);

private:
  bool is_running_;

  // ROS objects
  ros::NodeHandle nh_;
  ros::Rate *rate_;

  // ROS ServiceServer
  ros::ServiceServer srv_perform_square_;

  // ROS Publishers
  ros::Publisher pub_cmd_vel_;

  bool running_;
  int side_;
  int repetitions_;
  int duration_;
  int times_;
  float rate_hz_;
  int state_;
};

MoveBB8::MoveBB8() {
  running_ = false;
  side_ = 0;
  repetitions_ = 0;
  duration_ = 0;
  times_ = 0;
  rate_hz_ = 20.0;
  state_ = 0;

  // ROS objects
  rate_ = new ros::Rate(rate_hz_);

  // ROS Service
  srv_perform_square_ = nh_.advertiseService("/move_bb8_in_square_custom",
                                             &MoveBB8::my_callback, this);

  // ROS Publisher
  pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
}

void MoveBB8::runTimeStateMachine(void) {
  geometry_msgs::Twist vel;

  if (!running_) {
    vel.linear.x = 0;
    vel.angular.z = 0;
    pub_cmd_vel_.publish(vel);
    return;
  }

  vel = this->getStateVelocity();
  pub_cmd_vel_.publish(vel);

  duration_ -= 1 / (float)rate_hz_;
  // ROS_INFO("State [%d], Vel[%.2f, %.2f], Duration [%.2f]", state_,
  // vel.linear.x, vel.angular.z, duration_);

  if (duration_ <= 0) {
    float state_duration[4] = {side_, 3.8, 4.0, 0.1};
    int next_state = state_ + 1;
    if (state_ == 3) {
      next_state = 0;
      times_ -= 1;
    }
    int next_state_duration = state_duration[next_state];
    this->changeState(next_state, next_state_duration);
  }

  if (times_ == 0) {
    running_ = false;
    vel.linear.x = 0;
    vel.angular.z = 0;
    pub_cmd_vel_.publish(vel);
  }
}

void MoveBB8::changeState(int state, float duration) {
  state_ = state;
  duration_ = duration;
}

bool MoveBB8::my_callback(
    services_quiz::BB8CustomServiceMessage::Request &req,
    services_quiz::BB8CustomServiceMessage::Response &res) {
  // ROS_INFO("The Service move_bb8_in_square_custom has been called");

  running_ = !running_;
  side_ = req.side;
  repetitions_ = req.repetitions;
  times_ = 4 * repetitions_;
  res.success = true;

  ROS_INFO("Finished service move_bb8_in_square_custom");
  return true;
}

geometry_msgs::Twist MoveBB8::getStateVelocity() {
  geometry_msgs::Twist vel_msg;

  vel_msg.linear.x = 0.2;
  vel_msg.angular.z = 0.2;

  return vel_msg;
}

void MoveBB8::rateSleep(void) { rate_->sleep(); }

MoveBB8::~MoveBB8(void) {}

int main(int argc, char **argv) {
  ros::init(argc, argv, "service_move_bb8_in_square_custom_server");
  MoveBB8 moveBB8;

  while (ros::ok()) {
    moveBB8.runTimeStateMachine();
    moveBB8.rateSleep();
    ros::spinOnce();
  }

  return 0;
}