#include <ros/ros.h>

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
        ros::Duration(0.25).sleep();
        ros_ser.write("G1 E-6 P1800\n");
    }
    else if (current_statue == true && extrude_status == false)
    {
        ros_ser.write("G1 E6 P1800\n");
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
    
    sleep(2);
    ros_ser.write("G91\n");
    sleep(1);
    ros_ser.write("M104 S210\n");
    sleep(1);

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();

        // if (ros_ser.available())
        // {
        //     string serial_data = ros_ser.read(ros_ser.available());
        //     ROS_INFO_STREAM(serial_data);
        // }

        if (extrude_status == true)
            ros_ser.write("G1 E0.1 P30\n");

        loop_rate.sleep();
    }
}