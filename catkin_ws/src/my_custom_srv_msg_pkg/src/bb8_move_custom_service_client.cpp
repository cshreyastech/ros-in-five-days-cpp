#include "ros/ros.h"
#include "my_custom_srv_msg_pkg/MyCustomServiceMessage.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_bb8_in_circle_custom"); // Initialise a ROS node
  ros::NodeHandle nh;

  // Create the connection to the service /move_bb8_in_circle_custom
  ros::ServiceClient move_bb8_in_circle_service_client = nh.serviceClient<my_custom_srv_msg_pkg::MyCustomServiceMessage>("/move_bb8_in_circle_custom");

  my_custom_srv_msg_pkg::MyCustomServiceMessage srv;
  srv.request.duration = 10; // Create an object of type MyCustomServiceMessage

  if (move_bb8_in_circle_service_client.call(srv))
  {
    ROS_INFO("Service successfully called. Moving BB8 in a circle.");
  }
  else
  {
    ROS_ERROR("Failed to call service /move_bb8_in_circle_custom");
    return 1;
  }

  return 0;
}