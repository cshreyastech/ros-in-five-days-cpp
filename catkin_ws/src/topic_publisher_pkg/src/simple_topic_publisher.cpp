#include <ros/ros.h>
#include <std_msgs/Int32.h>
// Import all the necessary ROS libraries and import the Int32 message from the std_msgs package

int main(int argc, char** argv) {

    ros::init(argc, argv, "topic_publisher"); // Initiate a Node named 'topic_publisher'
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Int32>("counter", 1000); // Create a Publisher object, that will                                                                                // publish on the /counter topic messages
                                                                         // of type Int32
    ros::Rate loop_rate(2); // Set a publish rate of 2 Hz

    std_msgs::Int32 count; // Create a variable of type Int32
    count.data = 0; // Initialize 'count' variable

    while (ros::ok()) // Create a loop that will go until someone stops the program execution
    {
        pub.publish(count); // Publish the message within the 'count' variable
        ros::spinOnce();
        loop_rate.sleep(); // Make sure the publish rate maintains at 2 Hz
        ++count.data; // Increment 'count' variable
    }

    return 0;
}