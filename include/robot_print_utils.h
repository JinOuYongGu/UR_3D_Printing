#ifndef _ROBOT_PRINT_UTILS_H_
#define _ROBOT_PRINT_UTILS_H_

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

using namespace std;

void SetAvgCartesianSpeed(moveit_msgs::RobotTrajectory &trajectory, const std::string end_effector, const double speed)
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

void SetAvgCartesianSpeed(moveit::planning_interface::MoveGroupInterface::Plan &plan, const std::string end_effector, const double speed)
{
    SetAvgCartesianSpeed(plan.trajectory_, end_effector, speed);
}

bool CalculateOrient(vector<double> pt1, vector<double> pt2, vector<double> &orient, double maxAngle = 30)
{
    if (pt1.size() != 3 || pt2.size() != 3 || pt1 == pt2)
    {
        cout << "CalculateOrient: Invalid Input!\n";
        return false;
    }
    orient.clear();

    double dx = pt1[0] - pt2[0];
    double dy = pt1[1] - pt2[1];
    double dz = pt1[2] - pt2[2];

    double i = 0;
    double j = 0;
    double k = 0;

    if (dx != 0)
    {
        double a = dy / dx;
        double b = -1.0;
        double d = pt2[1] - dy / dx * pt2[0];

        i = b * dz;
        j = -1.0 * dz * a;
        k = dy * a - dx * b;
    }
    else
    {
        i = 0;
        j = -1.0 * dz;
        k = dy;
    }

    double length = pow(i * i + j * j + k * k, 0.5);
    i /= length;
    j /= length;
    k /= length;

    double rx = abs(acos(i));
    double ry = abs(acos(j));
    double rz = abs(acos(k));

    maxAngle = maxAngle * 3.1415926 / 180;
    double initAngle = 90 * 3.1415926 / 180;
    rx = rx < initAngle - maxAngle ? initAngle - maxAngle : rx;
    rx = rx > initAngle + maxAngle ? initAngle + maxAngle : rx;
    ry = ry < initAngle - maxAngle ? initAngle - maxAngle : ry;
    ry = ry > initAngle + maxAngle ? initAngle + maxAngle : ry;
    rz = rz > maxAngle ? maxAngle : rz;

    orient = {rx, ry, rz};
    // tf2::Quaternion qua;
    // qua.setRPY(rx, ry, rz);
    // qua.normalize();
    // orient = {qua.getX(), qua.getY(), qua.getZ(), qua.getW()};

    return true;
}

void cubicSmooth7(const vector<double> in, vector<double> &out)
{
    out.clear();
    out = in;

    if (in.size() < 7)
    {
        return;
    }
    else
    {
        out[0] = (39.0 * in[0] + 8.0 * in[1] - 4.0 * in[2] - 4.0 * in[3] + 1.0 * in[4] + 4.0 * in[5] - 2.0 * in[6]) / 42.0;
        out[1] = (8.0 * in[0] + 19.0 * in[1] + 16.0 * in[2] + 6.0 * in[3] - 4.0 * in[4] - 7.0 * in[5] + 4.0 * in[6]) / 42.0;
        out[2] = (-4.0 * in[0] + 16.0 * in[1] + 19.0 * in[2] + 12.0 * in[3] + 2.0 * in[4] - 4.0 * in[5] + 1.0 * in[6]) / 42.0;

        for (int idx = 3; idx < in.size() - 3; idx++)
        {
            out[idx] = (-2.0 * (in[idx - 3] + in[idx + 3]) + 3.0 * (in[idx - 2] + in[idx + 2]) + 6.0 * (in[idx - 1] + in[idx + 1]) + 7.0 * in[idx]) / 21.0;
        }

        int N = in.size();
        out[N - 3] = (-4.0 * in[N - 1] + 16.0 * in[N - 2] + 19.0 * in[N - 3] + 12.0 * in[N - 4] + 2.0 * in[N - 5] - 4.0 * in[N - 6] + 1.0 * in[N - 7]) / 42.0;
        out[N - 2] = (8.0 * in[N - 1] + 19.0 * in[N - 2] + 16.0 * in[N - 3] + 6.0 * in[N - 4] - 4.0 * in[N - 5] - 7.0 * in[N - 6] + 4.0 * in[N - 7]) / 42.0;
        out[N - 1] = (39.0 * in[N - 1] + 8.0 * in[N - 2] - 4.0 * in[N - 3] - 4.0 * in[N - 4] + 1.0 * in[N - 5] + 4.0 * in[N - 6] - 2.0 * in[N - 7]) / 42.0;
    }
}

#endif