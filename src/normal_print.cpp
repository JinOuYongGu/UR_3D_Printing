// 3D Print in a normal way

#include <ros/ros.h>

#include <math.h>
#include <fstream>
#include <string>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/Bool.h>

#include <tf/tf.h>

#include <robot_print_utils.h>

using namespace std;
namespace rvt = rviz_visual_tools;

typedef vector<geometry_msgs::Pose> PoseVector;
typedef pair<PoseVector, bool> PoseVector_Bool_Pair;

// I: file_name: the data file that extracted from the g-code file
// I: init_pose: the robot pose where the print start
// O: paths:  contain a set of paths, if the extruder works during the path, the paired bool is true
bool GetPathsFromFile(const string &file_name, const geometry_msgs::Pose &init_pose, vector<PoseVector_Bool_Pair> &paths)
{
    ifstream file;
    file.open(file_name.data());
    if (file.is_open())
        cout << "File loaded! Path is " << file_name << endl;
    else
    {
        cout << "Open data file failed!" << endl;
        return false;
    }
    paths.clear();

    // the 5 datas are: X position, Y position, Z position, Extrude length, Velocity
    double data_in_line[5];
    double prev_data[5] = {0};
    double value = 0;
    PoseVector waypoints;
    geometry_msgs::Pose pose = init_pose;
    int idx = 0;
    string file_line;
    while (getline(file, file_line))
    {
        istringstream file_line_stream(file_line);
        for (int jdx = 0; jdx < 5; jdx++)
        {
            if (file_line_stream >> value)
                // convert the unit from MM to M
                data_in_line[jdx] = value / 1000.0;
        }

        // check whether the points in path are duplicated
        if (data_in_line[0] == prev_data[0] &&
            data_in_line[1] == prev_data[1] &&
            data_in_line[2] == prev_data[2])
        {
            continue;
        }

        // when the extrude status changed, a path end
        if (data_in_line[3] > 0 && prev_data[3] <= 0 ||
            data_in_line[3] <= 0 && prev_data[3] > 0)
        {
            bool extrude;
            if (data_in_line[3] > 0 && prev_data[3] <= 0)
            {
                extrude = false;
            }
            if (data_in_line[3] <= 0 && prev_data[3] > 0)
            {
                extrude = true;
            }
            // push the path to paths vector
            PoseVector_Bool_Pair poseVector_bool_p(waypoints, extrude);
            paths.push_back(poseVector_bool_p);
            waypoints.clear();
            // then reset the path
            pose.position.x = init_pose.position.x + prev_data[0];
            pose.position.y = init_pose.position.y + prev_data[1];
            pose.position.z = init_pose.position.z + prev_data[2];
            waypoints.push_back(pose);
        }

        pose.position.x = init_pose.position.x + data_in_line[0];
        pose.position.y = init_pose.position.y + data_in_line[1];
        pose.position.z = init_pose.position.z + data_in_line[2];
        waypoints.push_back(pose);

        memcpy(prev_data, data_in_line, sizeof(prev_data));
        idx++;
    }
    return true;
}

// find the center point and move the path to it.
// I/O: paths
void LayoutPaths(double x_offset, double y_offset, vector<PoseVector_Bool_Pair> &paths)
{
    // vector<double> x_pos;
    // vector<double> y_pos;
    // for (auto p_b_p : paths)
    // {
    //     if (p_b_p.second == false)
    //         continue;
    //     for (auto pose : p_b_p.first)
    //     {
    //         x_pos.push_back(pose.position.x);
    //         y_pos.push_back(pose.position.y);
    //     }
    // }

    // double center_x = (*max_element(x_pos.begin(), x_pos.end()) + *min_element(x_pos.begin(), x_pos.end())) / 2.0;
    // double center_y = (*max_element(y_pos.begin(), y_pos.end()) + *min_element(y_pos.begin(), y_pos.end())) / 2.0;
    // ROS_INFO("the center point is %.2f, %.2f", center_x * 1000, center_y * 1000);

    // string dbg_file_name = "/home/jinou/catkin_ws/debug_file.txt";
    // ofstream out_file(dbg_file_name);
    // for (auto p_b_p : paths)
    // {
    //     for (auto pose : p_b_p.first)
    //     {
    //         double x_position = (pose.position.x - center_x) * 1000;
    //         double y_position = (pose.position.y - center_y) * 1000;
    //         out_file << x_position << "\t"
    //                  << y_position << "\t"
    //                  << pose.position.z * 1000 << "\t"
    //                  << p_b_p.second << "\t"
    //                  << "dbg_speed"
    //                  << "\n";
    //     }
    // }
    // out_file.close();
    // cout << "debug file outputed!\n";

    for (auto &&p_b_p : paths)
    {
        for (auto &&pose : p_b_p.first)
        {
            pose.position.x -= x_offset;
            pose.position.y -= y_offset;
        }
    }
}

void SetPathOrient(vector<PoseVector_Bool_Pair> &paths, double maxAngle)
{
    if (paths.size() == 0)
        return;

    vector<vector<double>> rpys;
    for (int jdx = 0; jdx < paths.size(); jdx++)
    {
        const PoseVector &vPose = paths[jdx].first;
        for (int idx = 0; idx < vPose.size() - 1; idx++)
        {
            vector<double> pt1 = {vPose[idx].position.x, vPose[idx].position.y, vPose[idx].position.z};
            vector<double> pt2 = {vPose[idx + 1].position.x, vPose[idx + 1].position.y, vPose[idx + 1].position.z};
            vector<double> target_rpy;
            CalculateOrient(pt1, pt2, target_rpy);
            rpys.push_back(target_rpy);

            // boundary condition
            if (idx + 1 == vPose.size() - 1)
            {
                // set the last point's pose
                if (jdx < paths.size() - 1)
                {
                    pt1 = {vPose[idx + 1].position.x, vPose[idx + 1].position.y, vPose[idx + 1].position.z};
                    geometry_msgs::Point next_path_first_pos = paths[jdx + 1].first.at(0).position;
                    pt2 = {next_path_first_pos.x, next_path_first_pos.y, next_path_first_pos.z};
                    CalculateOrient(pt1, pt2, target_rpy);
                    rpys.push_back(target_rpy);
                }
                else
                {
                    rpys.push_back(target_rpy);
                }
            }
        }
    }

    vector<double> vPoseR;
    vector<double> vPoseP;
    vector<double> vPoseY;
    for (int idx = 0; idx < rpys.size(); idx++)
    {
        vPoseR.push_back(rpys[idx][0]);
        vPoseP.push_back(rpys[idx][1]);
        vPoseY.push_back(rpys[idx][2]);
    }
    cubicSmooth7(vPoseR, vPoseR);
    cubicSmooth7(vPoseP, vPoseP);
    cubicSmooth7(vPoseY, vPoseY);
    cubicSmooth7(vPoseR, vPoseR);
    cubicSmooth7(vPoseP, vPoseP);
    cubicSmooth7(vPoseY, vPoseY);
    for (int idx = 0; idx < rpys.size(); idx++)
    {
        rpys[idx] = {vPoseR[idx], vPoseP[idx], vPoseY[idx]};
    }

    int kdx = 0;
    for (int jdx = 0; jdx < paths.size(); jdx++)
    {
        PoseVector &vPose = paths[jdx].first;
        for (int idx = 0; idx < vPose.size(); idx++)
        {
            tf2::Quaternion qua;
            qua.setRPY(rpys[kdx][0], rpys[kdx][1], rpys[kdx][2]);
            qua.normalize();

            vPose[idx].orientation.x = qua.getX();
            vPose[idx].orientation.y = qua.getY();
            vPose[idx].orientation.z = qua.getZ();
            vPose[idx].orientation.w = qua.getW();

            if (kdx < rpys.size())
            {
                kdx++;
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "normal_print");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    string end_effector_link = arm.getEndEffectorLink();

    string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);
    arm.allowReplanning(true);
    arm.setGoalPositionTolerance(0.0002);
    arm.setGoalOrientationTolerance(0.01);
    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);

    // =========== Create Visualization Tool =========== //
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.loadMarkerPub();
    visual_tools.deleteAllMarkers();

    // Initialize the position
    arm.setNamedTarget("work");
    arm.move();
    sleep(1);

    ROS_INFO("Input base Z height:");
    double z_height;
    cin >> z_height;
    geometry_msgs::Pose init_pose;
    init_pose.orientation.x = 0.5;
    init_pose.orientation.y = 0.5;
    init_pose.orientation.z = -0.5;
    init_pose.orientation.w = 0.5;
    init_pose.position.x = 0.0;
    init_pose.position.y = 0.4;
    init_pose.position.z = z_height / 1000.0;

    tf::Quaternion qua(0.5, 0.5, -0.5, 0.5);
    double roll, pitch, yaw;
    tf::Matrix3x3(qua).getRPY(roll, pitch, yaw);
    cout << "The current rpy is: " << roll << " " << pitch << " " << yaw << endl;

    // Move to the starting position
    geometry_msgs::Pose start_pose;
    start_pose.orientation.x = 0.5;
    start_pose.orientation.y = 0.5;
    start_pose.orientation.z = -0.5;
    start_pose.orientation.w = 0.5;
    start_pose.position.x = 0.0;
    start_pose.position.y = 0.4;
    start_pose.position.z = 0.1;
    arm.setPoseTarget(start_pose);
    arm.move();
    sleep(1);

    ROS_INFO("Input the file name here:");
    string file_name;
    cin >> file_name;
    file_name = "/home/jinou/catkin_ws/" + file_name;
    vector<PoseVector_Bool_Pair> paths;
    if (!GetPathsFromFile(file_name, init_pose, paths))
        return 1;
    ROS_INFO("Input the offset:");
    double x_offset, y_offset;
    cin >> x_offset >> y_offset;
    LayoutPaths(x_offset, y_offset, paths);
    //SetPathOrient(paths, 10);

    ROS_INFO("Input the velocity (mm/s):");
    double speed = 0;
    cin >> speed;
    speed = speed > 60 ? 60 : speed;
    ROS_INFO("The speed has been set to : %4.2f mm/s", speed);
    speed /= 1000.0; // convert the speed unit to m/s, since the ROS uses this unit

    // === Check the path availability == //
    const double jump_threshold = 0.0;
    const double eef_step = 0.0008;

    PoseVector all_poses;
    for (int idx = 0; idx < paths.size(); idx++)
    {
        all_poses.insert(all_poses.end(), paths[idx].first.begin(), paths[idx].first.end());
    }

    moveit_msgs::RobotTrajectory check_trajectory;
    double fraction = arm.computeCartesianPath(all_poses, eef_step, jump_threshold, check_trajectory);
    if (fraction != 1.0)
    {
        ROS_INFO("Path generating failed!");
        return 1;
    }
    else
        ROS_INFO("Path check passed!");

    // === Create a publisher to control the extruder == //
    ros::NodeHandle node_handle;
    ros::Publisher extrude_pub = node_handle.advertise<std_msgs::Bool>("extrude_status", 100);

    // === Execute the trajectories one by one === //
    for (int idx = 0; idx < paths.size(); idx++)
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit_msgs::RobotTrajectory trajectory;

        arm.computeCartesianPath(paths[idx].first, eef_step, jump_threshold, trajectory);

        // === shows the printing path (optional) === //
        if (paths[idx].second == true)
        {
            PoseVector display_path;
            display_path.push_back(paths[idx].first[0]);

            for (int jdx = 1; jdx < paths[idx].first.size(); jdx++)
            {
                geometry_msgs::Pose &last_display_pose = display_path[display_path.size() - 1];
                geometry_msgs::Pose &pose_in_path = paths[idx].first[jdx];

                double distance = pow((
                                          pow(pose_in_path.position.x - last_display_pose.position.x, 2) +
                                          pow(pose_in_path.position.y - last_display_pose.position.y, 2) +
                                          pow(pose_in_path.position.z - last_display_pose.position.z, 2)),
                                      0.5);
                if (distance > 0.001) // only shows points that distance > 1mm to save PC resource
                    display_path.push_back(pose_in_path);
            }
            visual_tools.publishPath(display_path, rvt::LIME_GREEN, rvt::XXXXSMALL);
            visual_tools.trigger();
        }

        plan.trajectory_ = trajectory;

        // if don't need to extrude when moving (when trade), uses a fast speed to avoid leaking
        if (paths[idx].second == false)
        {
            double trade_speed = speed * 3 > 60.0 ? 60.0 : speed * 3;
            SetAvgCartesianSpeed(plan, end_effector_link, trade_speed);
        }
        else
            SetAvgCartesianSpeed(plan, end_effector_link, speed);

        std_msgs::Bool extrude_status_msg;
        extrude_status_msg.data = paths[idx].second;
        extrude_pub.publish(extrude_status_msg);
        ros::Duration(0.2).sleep();

        arm.execute(plan);

        extrude_status_msg.data = false;
        extrude_pub.publish(extrude_status_msg);
        ros::Duration(0.2).sleep();
    }

    geometry_msgs::Pose end_pose = arm.getCurrentPose().pose;
    end_pose.position.z += 0.01; // lift nozzle 1cm after print
    arm.setPoseTarget(end_pose);
    arm.move();
    sleep(1);
    arm.setPoseTarget(start_pose);
    arm.move();
    sleep(1);

    // remove all paths in Rviz GUI
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

    ros::shutdown();
    return 0;
}