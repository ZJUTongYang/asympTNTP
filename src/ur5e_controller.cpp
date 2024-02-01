#include "ur5e_controller.h"
#include "prmstar.h"

trajectory_msgs::JointTrajectoryPoint initialiseTrajectoryPoint(const std::vector<double> joint_space, double duration)
{
    trajectory_msgs::JointTrajectoryPoint points;

    points.positions.resize(6);
    points.velocities.resize(6);
    points.accelerations.resize(6);
    for (unsigned int i = 0; i < 6; i++)
    {
        points.positions[i] = joint_space.at(i);
        points.velocities[i] = 0.0;
        points.accelerations[i] = 0.0;
    }

    points.time_from_start = ros::Duration(duration);

    return points;
}

void wrapToPi(std::vector<double> &joint_space)
{
    for (unsigned int i = 0; i < joint_space.size(); i++)
    {
        if (joint_space.at(i) > M_PI)
            joint_space.at(i) -= 2 * M_PI;
        else if (joint_space.at(i) < -M_PI)
            joint_space.at(i) += 2 * M_PI;
    }
}

std::vector<double> wrapToPiJointSpacee(std::vector<double> joint_space)
{
    std::vector<double> result;
    result = joint_space;

    for (unsigned int i = 0; i < result.size(); i++)
    {
        if (result.at(i) >= M_PI)
            result.at(i) -= 2 * M_PI;
        else if (result.at(i) < -M_PI)
            result.at(i) += 2 * M_PI;
    }

    return result;
}

namespace controller
{
    double getExpectedExecutionTime6D(std::vector<optimal_tntp::RobotState>& latest_motion_segment, const double joint_space_6d_velocity)
    {
        return optimal_tntp::pathLength(latest_motion_segment) / joint_space_6d_velocity;
    }

    void linearInterpolateConfigurations(std::vector<std::vector<double>> &result)
    {
        // YT: note that here each configuration must have 6 elements, 
        // no 5 joint-angles (because then it would be different from the expected task execution time), 
        // and no [joint, color] pair

        double step = 0.1;

        unsigned int i = 0;
        while (i < result.size() - 1)
        {
            std::vector<std::vector<double>> temp;
            temp.clear();

            double sum_temp = 0;

            for (unsigned int j = 0; j < result.at(i).size(); j++)
                sum_temp += pow(angles::normalize_angle(result.at(i + 1).at(j) - result.at(i).at(j)), 2);

            double norm = sqrt(sum_temp);

            if (norm > 0.5)
            {
                std::vector<double> d_angle;
                d_angle.resize(6);

                for (unsigned int j = 0; j < result.at(i).size(); j++)
                    d_angle.at(j) = ((angles::normalize_angle(result.at(i + 1).at(j) - result.at(i).at(j))) / norm * step) + result.at(i).at(j);

                for (unsigned int k = 0; k < i + 1; k++)
                    temp.push_back(result.at(k));

                temp.push_back(d_angle);

                for (unsigned int k = i + 1; k < result.size(); k++)
                    temp.push_back(result.at(k));

                result = temp;
            }
            else
                i++;
        }

        for (unsigned int i = 0; i < result.size(); i++)
        {
            result.at(i) = wrapToPiJointSpacee(result.at(i));
        }
    }

    void linearInterpolateConfigurations(std::vector<optimal_tntp::RobotState>& result)
    {
        std::vector<std::vector<double>> result_temp;

        result_temp.resize(result.size());

        for(unsigned int i = 0; i < result.size(); ++i)
        {
            result_temp[i].resize(6);
            for(unsigned int j = 0; j < 6; ++j)
            {
                result_temp[i][j] = result[i].joint_[j];   
            }
        }
        linearInterpolateConfigurations(result_temp);

        result.clear();
        result.resize(result_temp.size());
        for(unsigned int i = 0; i < result_temp.size(); ++i)
        {
            result[i].joint_.resize(6);
            for(unsigned int j = 0; j < 6; ++j)
            {
                result[i].joint_[j] = result_temp[i][j];
            }
        }
    }

};

Manipulator_Controller::Manipulator_Controller()
{
    // client_ = new Client("/scaled_pos_joint_traj_controller/follow_joint_trajectory", true);
    client_ = new Client("/pos_joint_traj_controller/follow_joint_trajectory", true);

    ROS_INFO("Waiting for action server to start.");

    // Wait for the action server to start
    client_->waitForServer(); // Will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
}

Manipulator_Controller::~Manipulator_Controller() {}

void Manipulator_Controller::trajectoryBetween2Points(std::vector<double> start_point, std::vector<double> end_point)
{
    //
    goal_.trajectory.header.stamp = ros::Time::now() + ros::Duration(1);

    // Initialise joint names
    goal_.trajectory.joint_names.resize(6);
    goal_.trajectory.joint_names[0] = "shoulder_pan_joint";
    goal_.trajectory.joint_names[1] = "shoulder_lift_joint";
    goal_.trajectory.joint_names[2] = "elbow_joint";
    goal_.trajectory.joint_names[3] = "wrist_1_joint";
    goal_.trajectory.joint_names[4] = "wrist_2_joint";
    goal_.trajectory.joint_names[5] = "wrist_3_joint";

    goal_.trajectory.points.resize(2);
    goal_.trajectory.points.at(0) = initialiseTrajectoryPoint(start_point, 4.0);
    goal_.trajectory.points.at(1) = initialiseTrajectoryPoint(end_point, 8.0);

    client_->sendGoal(goal_);

    while (client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED && ros::ok())
    {
        client_->waitForResult(ros::Duration(1));

        ROS_INFO("Current State: %s", client_->getState().toString().c_str());
        sleep(2);
    }
    ROS_INFO("Action ended!");
}

void Manipulator_Controller::trajectoryFromArray(std::vector<std::vector<double>> array)
{
    //
    goal_.trajectory.header.stamp = ros::Time::now() + ros::Duration(1);

    // Initialise joint names
    goal_.trajectory.joint_names.resize(6);
    goal_.trajectory.joint_names[0] = "shoulder_pan_joint";
    goal_.trajectory.joint_names[1] = "shoulder_lift_joint";
    goal_.trajectory.joint_names[2] = "elbow_joint";
    goal_.trajectory.joint_names[3] = "wrist_1_joint";
    goal_.trajectory.joint_names[4] = "wrist_2_joint";
    goal_.trajectory.joint_names[5] = "wrist_3_joint";

    goal_.trajectory.points.resize(array.size());
    for (unsigned int i = 0; i < array.size(); i++)
    {
        // wrapToPi(array.at(i));
        double time;
        time = 4 + (i * 1);
        goal_.trajectory.points.at(i) = initialiseTrajectoryPoint(array.at(i), time);
    }

    client_->sendGoal(goal_);

    // while (client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED && ros::ok())
    // {
    //     client_->waitForResult(ros::Duration(1));

    //     ROS_INFO("Current State: %s", client_->getState().toString().c_str());
    //     sleep(2);
    // }
    // ROS_INFO("Action ended!");
}

void Manipulator_Controller::trajectoryFromArray(std::vector<optimal_tntp::RobotState> array)
{
    std::vector<std::vector<double> > array_temp;
    array_temp.resize(array.size());

    for(unsigned int i = 0; i < array.size(); ++i)
    {
        array_temp[i].resize(6);
        for(unsigned int j = 0; j < 6; ++j)
        {
            array_temp[i][j] = array[i].joint_[j];
        }
    }

    trajectoryFromArray(array_temp);
}
