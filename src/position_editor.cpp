// Set the position and orientation of end effector

#include <ros/ros.h>

#include <iostream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_editor");
    ros::NodeHandle nodeHandle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string armName = "manipulator";
    moveit::planning_interface::MoveGroupInterface arm(armName);

    const string end_effector = arm.getEndEffectorLink();

    while (ros::ok())
    {
        geometry_msgs::PoseStamped end_pose = arm.getCurrentPose(end_effector);
        geometry_msgs::Pose target_pose = end_pose.pose;

        string type = "";
        cout << "Set param to edit: [pos/rpy]" << endl;
        cin >> type;

        if (type == "pos")
        {
            cout << "Input pos x, y, z :" << endl;
            double x, y, z;
            cin >> x;
            cin >> y;
            cin >> z;
            target_pose.position.x = x;
            target_pose.position.y = y;
            target_pose.position.z = z;

            arm.setPoseTarget(target_pose, end_effector);
        }

        else if (type == "rpy")
        {
            cout << "Input R, P, Y :" << endl;

            double r, p, y;
            cin >> r;
            cin >> p;
            cin >> y;

            r = r * M_PI / 180.0;
            p = p * M_PI / 180.0;
            y = y * M_PI / 180.0;
            tf2::Quaternion quaternion;
            quaternion.setRPY(r, p, y);
            quaternion.normalize();
            ROS_INFO_STREAM(quaternion);

            double quater_x = quaternion.getX();
            double quater_y = quaternion.getY();
            double quater_z = quaternion.getZ();
            double quater_w = quaternion.getW();

            target_pose.orientation.x = quater_x;
            target_pose.orientation.y = quater_y;
            target_pose.orientation.z = quater_z;
            target_pose.orientation.w = quater_w;

            arm.setPoseTarget(target_pose, end_effector);
        }

        else
        {
            cout << "Invalid option, please input again." << endl;
            continue;
        }

        arm.move();
        cout << "Robot moved" << endl;
    }

    return 0;
}