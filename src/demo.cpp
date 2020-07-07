#include <ros/ros.h>

#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
    // Standard C++ lib
    cout << "hello world!" << endl;

    // ROS lib method
    ROS_INFO("hello world! --from Ros");

    return 0;
}