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
    sleep(5);
    ros_ser.write("G91\n");
    sleep(1);
    ros_ser.write("M104 S240\n");
    sleep(1);
    ros_ser.write("M140 S70\n");
    sleep(1);

    ROS_INFO("Input the print speed (mm/s):");
    double print_speed;
    cin >> print_speed;
    print_speed = print_speed > 60 ? 60 : print_speed;
    ROS_INFO("Print speed set as %4.1d", print_speed);

    ROS_INFO("Input the extrude ratio (%):");
    double ratio = 100;
    cin >> ratio;
    ratio = ratio < 150 || ratio > 50 ? ratio / 100 : 1;
    ROS_INFO("Extrude ratio set as %4.1d%", ratio * 100);

    double nozzle_diameter = 0.4;
    double layer_thickness = 0.2;
    double freq = 200; // the loop frequency of extruder

    double extrude_speed = print_speed * layer_thickness * nozzle_diameter * 60.0 / 2.4;
    double extrude_length = extrude_speed * ratio / 60.0 / freq;

    string extrude_speed_string = to_string(extrude_speed).substr(0, 3);
    string extrude_length_string = to_string(extrude_length).substr(0, 6);
    string gcode = "G1 E" + extrude_length_string + " F" + extrude_speed_string + "\n";

    ros::Rate loop_rate(freq);
    while (ros::ok())
    {
        ros::spinOnce();

        if (extrude_status == true)
            ros_ser.write(gcode);

        loop_rate.sleep();
    }
}