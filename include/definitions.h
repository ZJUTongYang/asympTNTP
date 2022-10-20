#pragma once

#include <vector>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "ur5e_kinematics.h"
#include "prmstar.h"

struct ReconfigurationMotion{
    bool no_use_;

    int source_color_;
    int target_color_;
    int source_last_configuration_index_;

    ros::WallTime latest_construction_time_;

    std::vector<optimal_tntp::RobotState> the_motion_;
    double pathLength_;

    ReconfigurationMotion()
    {
        
    }

    ReconfigurationMotion(int source_color, int target_color)
    {
        source_color_ = source_color;
        target_color_ = target_color;
        no_use_ = false;
    }
};

struct ContinuousTrackingMotion{
    int color_;
    int first_point_index_;
    int last_point_index_; 

    // // Used in binary form. 
    // unsigned long parents_;

};

struct TaskSpaceWaypoint{
    double ee_matrix44_[16];// The end-effector pose
    std::vector<std::vector<double> > iks_;
    std::vector<int> color_;

    std::vector<double> getConfiguration(int color){
        std::vector<double> result;
        int loc = std::find(color_.begin(), color_.end(), color) - color_.begin();
        return iks_[loc];
    }

    TaskSpaceWaypoint(){
        iks_.clear();
        color_.clear();
    }

    TaskSpaceWaypoint(const std::vector<double>& ee_pose){
        for(unsigned int i = 0; i < 16; ++i)
            ee_matrix44_[i] = ee_pose[i];
        
        calculate_iks();
    }

    TaskSpaceWaypoint(const geometry_msgs::Pose& ee_pose){
        tf2::Quaternion quat_tf;
        tf2::convert(ee_pose.orientation, quat_tf);

        tf2::Matrix3x3 temp(quat_tf);
        ee_matrix44_[0] = temp[0][0];
        ee_matrix44_[1] = temp[0][1];
        ee_matrix44_[2] = temp[0][2];
        ee_matrix44_[3] = ee_pose.position.x;
        ee_matrix44_[4] = temp[1][0];
        ee_matrix44_[5] = temp[1][1];
        ee_matrix44_[6] = temp[1][2];
        ee_matrix44_[7] = ee_pose.position.y;
        ee_matrix44_[8] = temp[2][0];
        ee_matrix44_[9] = temp[2][1];
        ee_matrix44_[10] = temp[2][2];
        ee_matrix44_[11] = ee_pose.position.z;
        ee_matrix44_[12] = 0;
        ee_matrix44_[13] = 0;
        ee_matrix44_[14] = 0;
        ee_matrix44_[15] = 1;

        calculate_iks();
    }

private: 
    void calculate_iks(){
        double q_sols[48];
        int num_of_ik = ur5e_kinematics::inverse(ee_matrix44_, q_sols);
        if(num_of_ik == 0){
            iks_.clear();
            std::cout << "Error: All task-space waypoints should be reachable." << std::endl;
        }

        else{
            for(unsigned int i = 0; i < num_of_ik; ++i){
                std::vector<double> state_temp;
                state_temp.resize(6);
                memcpy(&(state_temp[0]), &(q_sols[i*6]), 6*sizeof(double));
                iks_.push_back(state_temp);
                color_.push_back(-1);
            }
        }
    }
};