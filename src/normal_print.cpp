// #D Print in a normal way

#include <ros/ros.h>

#include <math.h>
#include <fstream>
#include <string>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <const_cartesian_speed.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cartesian_cubic_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    string end_effector_link = arm.getEndEffectorLink();

    string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);

    arm.allowReplanning(true);
    arm.setGoalPositionTolerance(0.0002);
    arm.setGoalOrientationTolerance(0.01);
    arm.setMaxAccelerationScalingFactor(0.5);
    arm.setMaxVelocityScalingFactor(0.5);

    // Initialize the position
    arm.setNamedTarget("work");
    arm.move();
    sleep(1);

    // Set the starting position
    geometry_msgs::Pose target_pose;

    target_pose.orientation.x = -0.5;
    target_pose.orientation.y = 0.5;
    target_pose.orientation.z = 0.5;
    target_pose.orientation.w = 0.5;

    target_pose.position.x = 0.0;
    target_pose.position.y = 0.40;
    target_pose.position.z = 0.15;

    arm.setPoseTarget(target_pose);
    arm.move();
    sleep(1);

    vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose);

    cout << "Input the file name:\n";
    string fileName = "/home/jinou/catkin_ws/data.txt";

    ifstream inFile;
    inFile.open(fileName.data());
    if (inFile.is_open())
    {
        cout << "File Loaded!\n";
    }
    else
    {
        cout << "Open data file failed!\n";
        return 1;
    }

    string data;
    vector<double> vXdata;
    vector<double> vYdata;
    vector<double> vZdata;
    double value;
    while (getline(inFile, data))
    {
        istringstream dataStream(data);
        int idx = 0;
        while (dataStream >> value)
        {
            value = value / 1000.0;
            
            if (idx == 0)
                vXdata.push_back(value);
            if (idx == 1)
                vYdata.push_back(value);
            if (idx == 2)
                vZdata.push_back(value);

            idx++;
        }
    }

    if (vXdata.size() == vYdata.size() && vXdata.size() == vZdata.size())
        cout << vXdata.size() << "waypoints read success\n";
    else
    {
        cout << vXdata.size() << endl
             << vYdata.size() << endl
             << vZdata.size();
        return 1;
    }

    const double xpos = target_pose.position.x;
    const double ypos = target_pose.position.y;
    const double zpos = target_pose.position.z;

    for (int idx = 0; idx < vXdata.size(); idx++)
    {
        target_pose.position.x = xpos + vXdata[idx];
        target_pose.position.y = ypos + vYdata[idx];
        target_pose.position.z = zpos + vZdata[idx];

        waypoints.push_back(target_pose);
    }

    // 笛卡尔空间下的路径规划
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.0005;
    double fraction = 0.0;
    int maxtries = 100; //最大尝试规划次数
    int attempts = 0;   //已经尝试规划次数

    while (fraction < 1.0 && attempts < maxtries)
    {
        fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        attempts++;

        if (attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }

    if (fraction == 1)
    {
        ROS_INFO("Path computed successfully. Moving the arm.");

        // 生成机械臂的运动规划数据
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        double speed = 0;
        cout << "Input the speed(m/s):\n";
        cin >> speed;
        speed = speed > 0.1 ? 0.1 : speed;
        cout << "Speed has set to " << speed << endl;

        setAvgCartesianSpeed(plan, end_effector_link, speed);
        arm.execute(plan);
        sleep(1);
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }

    arm.setNamedTarget("work");
    arm.move();
    sleep(1);

    ros::shutdown();
    return 0;
}