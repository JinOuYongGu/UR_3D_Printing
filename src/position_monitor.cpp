#include <ros/ros.h>

#include <iostream>

#include <moveit/move_group_interface/move_group_interface.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_monitor");
    ros::NodeHandle nodeHandle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string armName = "manipulator";
    moveit::planning_interface::MoveGroupInterface arm(armName);

    string end_effector = arm.getEndEffectorLink();

    int idx = 1;
    int intervalSec = 5;
    while (true)
    {
        geometry_msgs::PoseStamped end_pose = arm.getCurrentPose(end_effector);

        cout << "=== [" << idx * intervalSec / 60 << "mins " << idx * intervalSec % 60 << "secs past] ===" << endl;
        cout << "Current pose w: " << end_pose.pose.orientation.w << endl;
        cout << "Current pose x: " << end_pose.pose.orientation.x << endl;
        cout << "Current pose y: " << end_pose.pose.orientation.y << endl;
        cout << "Current pose z: " << end_pose.pose.orientation.z << endl;

        idx++;
        sleep(intervalSec);
    }

    return 0;
}