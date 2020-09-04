#include <ros/ros.h>

#include <math.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

void setAvgCartesianSpeed(moveit::planning_interface::MoveGroupInterface::Plan &plan, const std::string end_effector, const double speed)
{
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();

    int num_waypoints = plan.trajectory_.joint_trajectory.points.size();                        //gets the number of waypoints in the trajectory
    const std::vector<std::string> joint_names = plan.trajectory_.joint_trajectory.joint_names; //gets the names of the joints being updated in the trajectory

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
        euclidean_distance = pow(pow(next_end_effector_state.translation()[0] - current_end_effector_state.translation()[0], 2) +
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
        curr_waypoint = &plan.trajectory_.joint_trajectory.points.at(i); //set current, previous & next waypoints
        if (i > 0)
            prev_waypoint = &plan.trajectory_.joint_trajectory.points.at(i - 1);
        if (i < num_waypoints - 1)
            next_waypoint = &plan.trajectory_.joint_trajectory.points.at(i + 1);

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
                v1 = v2 = a = 0.0;
            else
            {
                v1 = (q2 - q1) / dt1;
                v2 = (q3 - q2) / dt2;
                a = 2.0 * (v2 - v1) / (dt1 + dt2);
            }

            //actually set the velocity and acceleration
            curr_waypoint->velocities.at(j) = (v1 + v2) / 2;
            curr_waypoint->accelerations.at(j) = a;
        }
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "moveit_cartesian_demo");
	ros::AsyncSpinner spinner(1);
	spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    //获取终端link的名称
    std::string end_effector_link = arm.getEndEffectorLink();

    //设置目标位置所使用的参考坐标系
    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);

    //当运动规划失败后，允许重新规划
    arm.allowReplanning(true);

    //设置位置(单位：米)和姿态（单位：弧度）的允许误差
    arm.setGoalPositionTolerance(0.0005);
    arm.setGoalOrientationTolerance(0.01);

    //设置允许的最大速度和加速度
    arm.setMaxAccelerationScalingFactor(0.8);
    arm.setMaxVelocityScalingFactor(0.8);

    // 控制机械臂先回到初始化位置
    // arm.setNamedTarget("up");
    // arm.move();
    // sleep(5);

    arm.setNamedTarget("work");
    arm.move();
    sleep(1);

    // 设置机器人终端的目标位置
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

	std::vector<geometry_msgs::Pose> waypoints;

    //将初始位姿加入路点列表
	waypoints.push_back(target_pose);

    double centerA = target_pose.position.x;
    double centerB = target_pose.position.y;
    double radius = 0.05;

    std::cout << "Input the step distance:\n";
    double step = 0;
    std::cin >> step;
    step = step > 0 && step < 1 ? step : 0.1;

    for (double th = 0.0; th < 6.28; th = th + step)
    {
        target_pose.position.x = centerA + radius * cos(th);
        target_pose.position.y = centerB + radius * sin(th);
        waypoints.push_back(target_pose);
    }

    // 笛卡尔空间下的路径规划
	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.0005;
	double fraction = 0.0;
    int maxtries = 100;   //最大尝试规划次数
    int attempts = 0;     //已经尝试规划次数

    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        attempts++;
        
        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }
    
    if(fraction == 1)
    {   
        ROS_INFO("Path computed successfully. Moving the arm.");

	    // 生成机械臂的运动规划数据
	    moveit::planning_interface::MoveGroupInterface::Plan plan;
	    plan.trajectory_ = trajectory;

        std::cout << "Input the speed:\n";
        double speed = 0;
        std::cin >> speed;
        speed = speed > 0.1 ? 0.1 : speed;
        setAvgCartesianSpeed(plan, end_effector_link, speed);

        // 执行运动
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