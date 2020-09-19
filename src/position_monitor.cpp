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
    int intervalSec = 3;
    while (true)
    {
        geometry_msgs::PoseStamped end_pose = arm.getCurrentPose(end_effector);

        cout << "=== [" << idx * intervalSec / 60 << "mins " << idx * intervalSec % 60 << "secs past] ===" << endl;
        
        cout << "Current pos : " 
        << end_pose.pose.position.x << ", " 
        << end_pose.pose.position.y << ", " 
        << end_pose.pose.position.z << endl;

        cout << "Current orient xyzw : " 
        << end_pose.pose.orientation.x << ", " 
        << end_pose.pose.orientation.y << ", " 
        << end_pose.pose.orientation.z << ", " 
        << end_pose.pose.orientation.w << endl;

        vector<double> vJointValues = arm.getCurrentJointValues();
        for(auto&& value:vJointValues)
        {
            cout << value << ", ";
        }
        cout << endl;

        idx++;
        sleep(intervalSec);
    }

    return 0;
}