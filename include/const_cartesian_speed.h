#ifndef _CONST_CARTESIAN_SPEED_H_
#define _CONST_CARTESIAN_SPEED_H_

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

void setAvgCartesianSpeed(moveit::planning_interface::MoveGroupInterface::Plan &plan, const std::string end_effector, const double speed)
{
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();

    //gets the number of waypoints in the trajectory
    int num_waypoints = plan.trajectory_.joint_trajectory.points.size();
    //gets the names of the joints being updated in the trajectory
    const std::vector<std::string> joint_names = plan.trajectory_.joint_trajectory.joint_names;

    //set joint positions of zeroth waypoint
    kinematic_state->setVariablePositions(joint_names, plan.trajectory_.joint_trajectory.points.at(0).positions);

    Eigen::Affine3d current_end_effector_state = kinematic_state->getGlobalLinkTransform(end_effector);
    Eigen::Affine3d next_end_effector_state;
    double euclidean_distance, new_timestamp, old_timestamp, q1, q2, q3, dt1, dt2, v1, v2, a;
    trajectory_msgs::JointTrajectoryPoint *prev_waypoint, *curr_waypoint, *next_waypoint;

    for (int i = 0; i < num_waypoints - 1; i++) //loop through all waypoints
    {
        curr_waypoint = &plan.trajectory_.joint_trajectory.points.at(i);
        next_waypoint = &plan.trajectory_.joint_trajectory.points.at(i + 1);

        //set joints for next waypoint
        kinematic_state->setVariablePositions(joint_names, next_waypoint->positions);

        //do forward kinematics to get cartesian positions of end effector for next waypoint
        next_end_effector_state = kinematic_state->getGlobalLinkTransform(end_effector);

        //get euclidean distance between the two waypoints
        euclidean_distance = pow(
            pow(next_end_effector_state.translation()[0] - current_end_effector_state.translation()[0], 2) +
                pow(next_end_effector_state.translation()[1] - current_end_effector_state.translation()[1], 2) +
                pow(next_end_effector_state.translation()[2] - current_end_effector_state.translation()[2], 2),
            0.5);

        new_timestamp = curr_waypoint->time_from_start.toSec() + (euclidean_distance / speed); //start by printing out all 3 of these!
        old_timestamp = next_waypoint->time_from_start.toSec();

        //update next waypoint timestamp & joint velocities/accelerations if joint velocity/acceleration constraints allow
        if (new_timestamp > old_timestamp)
            next_waypoint->time_from_start.fromSec(new_timestamp);
        else
            ROS_WARN_NAMED("setAvgCartesianSpeed", "Average speed is too fast. Moving as fast as joint constraints allow.");

        //update current_end_effector_state for next iteration
        current_end_effector_state = next_end_effector_state;
    }
}

#endif