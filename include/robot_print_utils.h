#ifndef _ROBOT_PRINT_UTILS_H_
#define _ROBOT_PRINT_UTILS_H_

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

void setAvgCartesianSpeed(moveit_msgs::RobotTrajectory& trajectory, const std::string end_effector, const double speed)
{
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();

    int num_waypoints = trajectory.joint_trajectory.points.size();
    const std::vector<std::string> joint_names = trajectory.joint_trajectory.joint_names;
    //set joint positions of zeroth waypoint
    kinematic_state->setVariablePositions(joint_names, trajectory.joint_trajectory.points.at(0).positions);

    Eigen::Affine3d current_end_effector_state = kinematic_state->getGlobalLinkTransform(end_effector);
    Eigen::Affine3d next_end_effector_state;
    double euclidean_distance, new_timestamp, old_timestamp, q1, q2, q3, dt1, dt2, v1, v2, accel;
    trajectory_msgs::JointTrajectoryPoint *prev_waypoint, *curr_waypoint, *next_waypoint;

    for (int idx = 0; idx < num_waypoints - 1; idx++) //loop through all waypoints
    {
        curr_waypoint = &trajectory.joint_trajectory.points.at(idx);
        next_waypoint = &trajectory.joint_trajectory.points.at(idx + 1);

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
        {
            //ROS_WARN_NAMED("setAvgCartesianSpeed", "Average speed is too fast. Moving as fast as joint constraints allow.");
        }

        //update current_end_effector_state for next iteration
        current_end_effector_state = next_end_effector_state;
    }

    //now that timestamps are updated, update joint velocities/accelerations (used updateTrajectory from iterative_time_parameterization as a reference)
    for (int i = 0; i < num_waypoints; i++)
    {
        curr_waypoint = &trajectory.joint_trajectory.points.at(i); //set current, previous & next waypoints
        if (i > 0)
            prev_waypoint = &trajectory.joint_trajectory.points.at(i - 1);
        if (i < num_waypoints - 1)
            next_waypoint = &trajectory.joint_trajectory.points.at(i + 1);

        if (i == 0) //update dt's based on waypoint (do this outside of loop to save time)
            dt1 = dt2 = next_waypoint->time_from_start.toSec() - curr_waypoint->time_from_start.toSec();
        else if (i < num_waypoints - 1)
        {
            dt1 = curr_waypoint->time_from_start.toSec() - prev_waypoint->time_from_start.toSec();
            dt2 = next_waypoint->time_from_start.toSec() - curr_waypoint->time_from_start.toSec();
        }
        else
            dt1 = dt2 = curr_waypoint->time_from_start.toSec() - prev_waypoint->time_from_start.toSec();

        for (int j = 0; j < joint_names.size(); j++) //loop through all joints in waypoint
        {
            if (i == 0) //first point
            {
                q1 = next_waypoint->positions.at(j);
                q2 = curr_waypoint->positions.at(j);
                q3 = q1;
            }
            else if (i < num_waypoints - 1) //middle points
            {
                q1 = prev_waypoint->positions.at(j);
                q2 = curr_waypoint->positions.at(j);
                q3 = next_waypoint->positions.at(j);
            }
            else //last point
            {
                q1 = prev_waypoint->positions.at(j);
                q2 = curr_waypoint->positions.at(j);
                q3 = q1;
            }

            if (dt1 == 0.0 || dt2 == 0.0)
                v1 = v2 = accel = 0.0;
            else
            {
                v1 = (q2 - q1) / dt1;
                v2 = (q3 - q2) / dt2;
                accel = 2.0 * (v2 - v1) / (dt1 + dt2);
            }

            //actually set the velocity and acceleration
            curr_waypoint->velocities.at(j) = (v1 + v2) / 2;
            curr_waypoint->accelerations.at(j) = accel;
        }
    }
}

void setAvgCartesianSpeed(moveit::planning_interface::MoveGroupInterface::Plan &plan, const std::string end_effector, const double speed)
{
    setAvgCartesianSpeed(plan.trajectory_, end_effector, speed);
}

#endif