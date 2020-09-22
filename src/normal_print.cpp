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

bool GetDataFromFile(const string &fileName, vector<double>& xData, vector<double>& yData, vector<double>& zData)
{
    ifstream file;
    file.open(fileName.data());
    if (file.is_open())
        cout << "File loaded! Path is " << fileName << endl;
    else
    {
        cout << "Open data file failed!\n";
        return false;
    }

    string dataInLine;
    double value;
    while (getline(file, dataInLine))
    {
        istringstream dataStream(dataInLine);
        int idx = 0;
        while (dataStream >> value)
        {
            value = value / 1000.0; // convert the unit from MM to M

            if (idx == 0)
                xData.push_back(value);
            if (idx == 1)
                yData.push_back(value);
            if (idx == 2)
                zData.push_back(value);

            idx++;
        }
    }

    if (xData.size() == yData.size() && xData.size() == zData.size())
    {
        cout << xData.size() << " waypoints read successfully.\n";
        return true;
    }
    else
    {
        cout << "Data read error!\n";
        return false;
    }
}

    int main(int argc, char **argv)
{
    ros::init(argc, argv, "cartesian_cubic_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    string end_effector_link = arm.getEndEffectorLink();

    cout << "End effector is: " << end_effector_link << endl;

    string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);

    arm.allowReplanning(true);
    arm.setGoalPositionTolerance(0.0002);
    arm.setGoalOrientationTolerance(0.01);
    arm.setMaxAccelerationScalingFactor(0.5);
    arm.setMaxVelocityScalingFactor(0.5);

    // Initialize the position
    arm.setNamedTarget("up");
    arm.move();
    sleep(1);

    arm.setNamedTarget("work");
    arm.move();
    sleep(1);

    // Set the starting position
    geometry_msgs::Pose target_pose;

    target_pose.orientation.x = 0.5;
    target_pose.orientation.y = 0.5;
    target_pose.orientation.z = -0.5;
    target_pose.orientation.w = 0.5;

    target_pose.position.x = 0.0;
    target_pose.position.y = 0.4;
    target_pose.position.z = 0.1;

    arm.setPoseTarget(target_pose);
    arm.move();
    sleep(1);

    vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose);

    cout << "Input the file name:\n";
    string fileName = "/home/jinou/catkin_ws/data.txt";

    vector<double> xData;
    vector<double> yData;
    vector<double> zData;
    if(!GetDataFromFile(fileName, xData, yData, zData))
        return 1;

    const double xpos = target_pose.position.x;
    const double ypos = target_pose.position.y;
    const double zpos = target_pose.position.z;

    int numDataSize = xData.size();

    for (int idx = 0; idx < xData.size(); idx++)
    {
        target_pose.position.x = xpos + xData[idx] / 2;
        target_pose.position.y = ypos + yData[idx] / 2;
        target_pose.position.z = zpos + zData[idx];

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
        ROS_INFO("Path computed successfully.");

        // 生成机械臂的运动规划数据
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        ROS_INFO("Input the velocity:");
        double speed = 0;
        cin >> speed;
        speed = speed > 0.1 ? 0.1 : speed;
        cout << "Speed has set to " << speed << endl;

        setAvgCartesianSpeed(plan, end_effector_link, speed);

        int numWayPoints = plan.trajectory_.joint_trajectory.points.size();
        cout << "cartesian points number is : " << numWayPoints << endl;

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