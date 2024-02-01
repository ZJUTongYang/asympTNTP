#include <fstream>

#include <vector>
#include <geometry_msgs/Pose.h>

#include "input_data.h"

#include "definitions.h"

std::vector<geometry_msgs::Pose> task_space_discrete_waypoints_for_test()
{
	std::vector<geometry_msgs::Pose> result;
	result.clear();
	geometry_msgs::Pose start, goal;
	start.position.x = 0.4;
	start.position.y = 0.1;
	start.position.z = 0.5;
	start.orientation.w = 1.0;
	goal.position.x = 0.1;
	goal.position.y = 0.4;
	goal.position.z = 0.3;
	goal.orientation.w = 1.0;
	const int n = 10;
	for (unsigned int i = 0; i < n; ++i)
	{
		geometry_msgs::Pose temp;
		temp.position.x = start.position.x * (n - i) / n + goal.position.x * i / n;
		temp.position.y = start.position.y * (n - i) / n + goal.position.y * i / n;
		temp.position.z = start.position.z * (n - i) / n + goal.position.z * i / n;
		temp.orientation.w = 1.0;
		result.push_back(temp);
	}
	return result;
}

std::vector<TaskSpaceWaypoint> case_study_1_matrix44(std::string file_name)
{
	std::vector<TaskSpaceWaypoint> result;

	std::ifstream infile;
	std::cout << file_name.data() << std::endl;
	infile.open(file_name.data(), std::ios::in);
	assert(infile.is_open());


	std::string s;
	std::getline(infile, s);
	std::istringstream is(s);

	int point_num;
	is >> point_num;

	for (unsigned int i = 0; i < point_num; ++i)
	{
		std::getline(infile, s);
		std::istringstream is(s);

		std::vector<double> conf_temp;
		double conf_temptemp;
		for(unsigned int j = 0; j < 16; ++j)
		{
			is >> conf_temptemp;
			conf_temp.push_back(conf_temptemp);
		}
		result.push_back(TaskSpaceWaypoint(conf_temp));

	}

    return result;

}

std::vector<TaskSpaceWaypoint> input_data(std::string file_name)
{
    std::vector<TaskSpaceWaypoint> result;

    std::ifstream infile;
	std::cout << file_name.data() << std::endl;
	infile.open(file_name.data(), std::ios::in);
	assert(infile.is_open());

	std::string s;
	std::getline(infile, s);
	std::istringstream is(s);

	int conf_num;
	is >> conf_num;

	for (unsigned int i = 0; i < conf_num; ++i)
	{
		std::getline(infile, s);
		std::istringstream is(s);

		std::vector<double> conf_temp;
		double joint_angle;
		for (unsigned int j = 0; j < 6; ++j)
		{
			is >> joint_angle;
			conf_temp.emplace_back(joint_angle);
		}
		int waypoint_index;
		is >> waypoint_index;
		--waypoint_index;
		int ik_index;
		is >> ik_index;
		--ik_index;

		if (result.size() < waypoint_index + 1)
			result.resize(waypoint_index + 1);


		result[waypoint_index].iks_.push_back(conf_temp);
		result[waypoint_index].color_.push_back(-1);

		// ++result[waypoint_index].num_ik_;
	}

    return result;

}

void output_data(std::string file_name, const std::vector<optimal_tntp::RobotState>& robot_state)
{
	std::ofstream out(file_name);
	for(auto iter = robot_state.begin(); iter != robot_state.end(); ++iter)
	{
		for(auto iter2 = iter->joint_.begin(); iter2 != iter->joint_.end(); ++iter2)
		{
			out << *iter2 << " ";
		}
		out << ";" << std::endl;
	}
	out << std::endl;
	out.close();
}

std::vector<std::vector<double>> input_data_joint_space(std::string file_name)
{
    std::vector<std::vector<double>> result;
    result.clear();

    std::ifstream infile;
    infile.open(file_name.data(), std::ios::in);
    assert(infile.is_open());

    while (infile){
        std::string s;
        std::getline(infile, s);
        std::istringstream is(s);
        std::vector<double> join_space;
        join_space.resize(6);
        is >> join_space.at(0) >> join_space.at(1) >> join_space.at(2) >> join_space.at(3) >> join_space.at(4) >> join_space.at(5);
        // wrapToPi(join_space);
        result.push_back(join_space);
    }
    result.pop_back();

    return result;
}



