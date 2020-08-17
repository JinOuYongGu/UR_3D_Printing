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

    while (true)
    {
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

            arm.setPositionTarget(x, y, z, end_effector);
        }

        else if (type == "rpy")
        {
            cout << "Input R, P, Y :" << endl;

            double r, p, y;
            cin >> r;
            cin >> p;
            cin >> y;

            tf2::Quaternion quaternion;
            quaternion.setRPY(r, p, y);
            quaternion.normalize();
            ROS_INFO_STREAM(quaternion);

            double quater_x = quaternion.getX();
            double quater_y = quaternion.getY();
            double quater_z = quaternion.getZ();
            double quater_w = quaternion.getW();

            arm.setOrientationTarget(quater_x, quater_y, quater_z, quater_w, end_effector);
            cout << "Orientation seted as " << quater_x << ", " << quater_y << ", " << quater_z << ", " << quater_w << endl;
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