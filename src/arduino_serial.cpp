#include <ros/ros.h>

#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

using namespace serial;
using namespace std;

Serial ros_ser;

//回调函数
void callback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO_STREAM("Write to serial port: " << msg->data);
    size_t bytes_wrote = ros_ser.write(msg->data);
    cout << "writed " << bytes_wrote << " bytes" << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arduino_serial");
    ros::NodeHandle n;
    //订阅主题command
    ros::Subscriber command_sub = n.subscribe("gcode_command", 1000, callback);

    cout << "Input the usb port index:\n";
    int portIdx = 0;
    cin >> portIdx;
    string portName = "/dev/ttyUSB" + to_string(portIdx);

    ros_ser.setPort(portName);
    ros_ser.setBaudrate(115200);
    Timeout to = Timeout::simpleTimeout(1000);
    ros_ser.setTimeout(to);
    ros_ser.open();

    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        ros::spinOnce();

        if (ros_ser.available())
        {
            std_msgs::String serial_data;
            serial_data.data = ros_ser.read(ros_ser.available());
            ROS_INFO_STREAM(serial_data.data);
        }

        loop_rate.sleep();
    }
}