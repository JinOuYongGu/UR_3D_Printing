#include "ros/ros.h"

#include "std_msgs/String.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arduino_sender");
    ros::NodeHandle n;

    ros::Publisher gcode_pub = n.advertise<std_msgs::String>("gcode_command", 1000);

    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        std_msgs::String msg;

        cout << "Input the command here:\n";
        string mystring;
        getline(cin, mystring);

        msg.data = mystring + "\n";

        ROS_INFO("The command is: %s", msg.data.c_str());

        gcode_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}