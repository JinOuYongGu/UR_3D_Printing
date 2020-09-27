// The robot end effector will move along a circle in XY plane
// Path resolution and velocity can be seted

#include <ros/ros.h>

#include <math.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <robot_print_utils.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cartesian_circle_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    std::string end_effector_link = arm.getEndEffectorLink();

    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);

    arm.allowReplanning(true);
    arm.setGoalPositionTolerance(0.0005);
    arm.setGoalOrientationTolerance(0.01);
    arm.setMaxAccelerationScalingFactor(0.8);
    arm.setMaxVelocityScalingFactor(0.8);

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
    target_pose.position.y = 0.30;
    target_pose.position.z = 0.1;

    arm.setPoseTarget(target_pose);
    arm.move();
    sleep(1);

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose);

    double centerA = target_pose.position.x;
    double centerB = target_pose.position.y;
    double radius = 0.05;

    std::cout << "Input the step distance:\n";
    double step = 0;
    std::cin >> step;
    step = step > 0 && step < 1 ? step : 0.1;

    for (double th = 0.0; th < 2 * M_PI; th = th + step)
    {
        target_pose.position.x = centerA + radius * cos(th);
        target_pose.position.y = centerB + radius * sin(th);
        waypoints.push_back(target_pose);
    }

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.0005;

    double fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    if (fraction != 1.0)
    {
        ROS_INFO("Path planning failed with only %0.6f success.", fraction);
        return 1;
    }
    else
        ROS_INFO("Path computed successfully. Moving the arm.");

    // 生成机械臂的运动规划数据
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    double speed = 0;
    std::cout << "Input the speed(m/s):\n";
    std::cin >> speed;
    speed = speed > 0.1 ? 0.1 : speed;
    std::cout << "Speed has set to " << speed << std::endl;

    // replan the velocity and acceleration of trajectory
    setAvgCartesianSpeed(plan, end_effector_link, 0.03);
    arm.execute(plan);

    arm.setNamedTarget("work");
    arm.move();
    sleep(1);

    ros::shutdown();
    return 0;
}