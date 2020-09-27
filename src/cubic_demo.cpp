// The robot end effector will move with Zigzag path in XY plane
// Path resolution and velocity can be seted

#include <ros/ros.h>

#include <math.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <robot_print_utils.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cartesian_cubic_demo");
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
    target_pose.position.y = 0.3;
    target_pose.position.z = 0.15;

    arm.setPoseTarget(target_pose);
    arm.move();
    sleep(1);

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose);

    double step = 0.001;
    double size = 0.1;
    double center_x = target_pose.position.x;
    double center_y = target_pose.position.y;
    double direction = 1.0;

    std::cout << "Input the step length(m):\n";
    std::cin >> step;

    while (ros::ok())
    {
        target_pose.position.x += direction * step;

        double distance_x = fabs(target_pose.position.x - center_x);
        double distance_y = fabs(target_pose.position.y - center_y);

        if (distance_x > size)
        {
            direction = -1.0 * direction;
            target_pose.position.y += step;
        }

        if (distance_y > size)
            break;

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
        std::cout << "Input the speed(m/s):\n";
        std::cin >> speed;
        speed = speed > 0.1 ? 0.1 : speed;
        std::cout << "Speed has set to " << speed << std::endl;

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