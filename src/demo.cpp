#include <ros/ros.h>

#include <iostream>

#include <moveit/move_group_interface/move_group_interface.h>

using namespace std;

int main(int argc, char** argv)
{
    // Initialize the demo node
    ros::init(argc, argv, "demo");
    ros::NodeHandle nodeHandle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Setup the move group, which is a manipulator robot here.
    // To find the correct string of name, reference the file ur3.srdf
    static const std::string armName = "manipulator";

    moveit::planning_interface::MoveGroupInterface arm(armName);

    // Test code
    ROS_INFO("=============Hello World=============");
    ROS_INFO_NAMED("tutorial", "Current reference frame: %s", arm.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", arm.getEndEffectorLink().c_str());
    ROS_INFO("=============  Bye World  =============");
    // End of test code

    // allow replanning
    arm.allowReplanning(true);
    // the unit is m
    arm.setGoalPositionTolerance(0.001);
    // the unit is rad
    arm.setGoalOrientationTolerance(0.01);

    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);

    arm.setNamedTarget("up");
    arm.move();
    sleep(1);

    return 0;
}