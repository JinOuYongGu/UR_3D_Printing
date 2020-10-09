#include <ros/ros.h>

#include <iostream>

#include <serial/serial.h>
#include <std_msgs/Bool.h>

using namespace serial;
using namespace std;

Serial ros_ser;
bool extrude_status = false;

//回调函数
void callback(const std_msgs::Bool::ConstPtr& extrude_status_msg)
{
    bool current_statue = extrude_status_msg->data;
    if (current_statue == false && extrude_status == true)
    {
        ros::Duration(0.05).sleep();
        ros_ser.write("G1 E-5 F2500\n");
    }
    else if (current_statue == true && extrude_status == false)
    {
        ros::Duration(0.05).sleep();
        ros_ser.write("G1 E5 F2500\n");
    }
    
    extrude_status = extrude_status_msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arduino_serial");
    ros::NodeHandle n;
    ros::Subscriber command_sub = n.subscribe("extrude_status", 1000, callback);

    cout << "Input the usb port index:\n";
    int portIdx = 0;
    cin >> portIdx;
    string portName = "/dev/ttyUSB" + to_string(portIdx);

    ros_ser.setPort(portName);
    ros_ser.setBaudrate(115200);
    Timeout to = Timeout::simpleTimeout(1000);
    ros_ser.setTimeout(to);
    ros_ser.open();

    ROS_INFO("Starting marlin firmware ...\n");
    sleep(2);
    ros_ser.write("G91\n");
    sleep(1);
    ros_ser.write("M104 S210\n");
    sleep(1);
    ros_ser.write("M140 S60\n");
    sleep(1);

    ROS_INFO("Input the print speed (mm/s):\n");
    double print_speed;
    cin >> print_speed;
    print_speed = print_speed > 20 ? 20 : print_speed;
    ROS_INFO("Print speed set as %d", print_speed);

    double extrude_speed = print_speed * 0.8 * 0.4 * 60.0 / 2.4;
    double extrude_length = extrude_speed / 200.0 / 60.0;

    string extrude_speed_string = to_string(extrude_speed).substr(0, 3);
    string extrude_length_string = to_string(extrude_length).substr(0, 6);
    string gcode = "G1 E" + extrude_length_string + " F" + extrude_speed_string + "\n";

    ros::Rate loop_rate(200);
    while (ros::ok())
    {
        ros::spinOnce();

        if (extrude_status == true)
            ros_ser.write(gcode);

        loop_rate.sleep();
    }
}