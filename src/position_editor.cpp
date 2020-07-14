#include <ros/ros.h>

#include <iostream>

#include <moveit/move_group_interface/move_group_interface.h>

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
        cout << "Input x, y, z, w :" << endl;
        double x, y, z, w;
        cin >> x;
        cin >> y;
        cin >> z;
        cin >> w;

        if (x > 1 || x < -1 || y > 1 || y < -1 || z > 1 || z < -1 || w > 1 || w < -1)
            cout << "Invalid input!" << endl;
        else
        {
            arm.setOrientationTarget(x, y, z, w, end_effector);
            cout << "Orientation seted as " << x << ", " << y << ", " << z << ", " << w << endl;

            arm.move();
            cout << "Robot moved" << endl;
        }
    }

    return 0;
}