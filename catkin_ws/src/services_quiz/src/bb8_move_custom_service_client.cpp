#include "ros/ros.h"
#include "services_quiz/BB8CustomServiceMessage.h"

int main(int argc, char **argv) {
  ros::init(
      argc, argv,
      "service_move_bb8_in_square_custom_client"); // Initialise a ROS node
  ros::NodeHandle nh;

  // Create the connection to the service /move_bb8_in_circle_custom
  ros::ServiceClient move_bb8_in_square_service_client =
      nh.serviceClient<services_quiz::BB8CustomServiceMessage>(
          "/move_bb8_in_square_custom");

  services_quiz::BB8CustomServiceMessage srv;
  srv.request.side = 3.0; // Create an object of type MyCustomServiceMessage
  srv.request.repetitions = 2;
  if (move_bb8_in_square_service_client.call(srv)) {
    ROS_INFO("Service successfully called. Moving BB8 in a square.");
  } else {
    ROS_ERROR("Failed to call service /move_bb8_in_square_custom");
    return 1;
  }

  return 0;
}