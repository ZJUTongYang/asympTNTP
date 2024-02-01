#include <ros/ros.h>
#include <time.h>

#include "ur5e_kinematics.h"
#include "ur5e_controller.h"
#include "prmstar.h"
#include "input_data.h"
#include "environmental_obstacles.h"
#include "definitions.h"
#include "asymp_tntp_algorithm.h"
#include "ral22_min_reconfig_tntp.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene/planning_scene.h>
#include <geometry_msgs/Pose.h>

#include <moveit/ompl_interface/ompl_interface.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <moveit/kinematic_constraints/utils.h>
#include <ompl/base/State.h>
#include <ompl/geometric/PathGeometric.h>

bool isStateValid(const std::vector<double> *joint_angles,
                  const planning_scene::PlanningScenePtr my_scene,
                  const moveit::core::JointModelGroup *joint_model_group)
{
  moveit::core::RobotState &current_state = my_scene->getCurrentStateNonConst();

  collision_detection::CollisionRequest req;

  collision_detection::CollisionResult res;
  res.clear();

  current_state.setJointGroupPositions(joint_model_group, *joint_angles);

  my_scene->checkCollision(req, res);

  return !res.collision;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ur5e_optimal_tntp_planner");

  ros::NodeHandle nh;

  const double joint_space_6d_velocity = 0.5;

  const std::string PLANNING_GROUP = "manipulator";

  // YT: we construct basic structures
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr &kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(kinematic_model));

  std::cout << "YT: We create basic scene structures" << std::endl;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  moveit::planning_interface::MoveGroupInterface ur5e_("manipulator");
  // std::cout << ur5e_.getCurrentPose() << std::endl;

  addObstacles_case_study_1(planning_scene_interface_, ur5e_);

  std::cout << "We print the name of all environmental obstacles. " << std::endl;
  int count = 0;
  std::map<std::string, moveit_msgs::CollisionObject> objects_in_environment = planning_scene_interface_.getObjects();
  for (auto iter = objects_in_environment.begin(); iter != objects_in_environment.end(); ++iter)
    std::cout << "Object " << count++ << " : " << iter->second.id << std::endl;

  std::cout << "YT: We add the objects into the planning scene. " << std::endl;
  for (auto iter = objects_in_environment.begin(); iter != objects_in_environment.end(); ++iter)
    planning_scene->processCollisionObjectMsg(iter->second);

  moveit::core::RobotState &current_state = planning_scene->getCurrentStateNonConst();
  const moveit::core::JointModelGroup *joint_model_group = current_state.getJointModelGroup(PLANNING_GROUP);

  ros::Duration(5.0).sleep(); // We wait all modules to be loaded. Then we calculate time

  ///////////////////////////////////////////////////WE SETUP OUR PRMSTAR////////////////////////////////////
  std::function<bool(const std::vector<double> *)> collisionChecker =
      std::bind(&isStateValid, std::placeholders::_1, planning_scene, joint_model_group);

  srand(0);
  optimal_tntp::PRMstar our_prmstar(collisionChecker);

  ros::WallTime initial_guess_time_start = ros::WallTime::now();
  for (unsigned int i = 0; i < 500; ++i)
    our_prmstar.add_milestone();

  our_prmstar.showDetails();

  /////////////////////////////////////////// TESTING PRM STAR ALGORITHM //////////////////////////////////////////

  // std::vector<double> def{0, -M_PI / 2, 0, -M_PI / 2, 0, 0};
  // std::vector<double> start{0, -M_PI, 0, -M_PI / 2, 0, 0};

  // optimal_tntp::RobotState source(def);
  // optimal_tntp::RobotState target(start);
  // std::vector<optimal_tntp::RobotState> result;
  // result.clear();

  // our_prmstar.findPath(source, target, result);
  // std::vector<std::vector<double>> joint_result;
  // for (unsigned int j = 0; j < result.size(); j++)
  // {
  //   joint_result.push_back(result.at(j).joint_);
  // }

  // controller::linearInterpolateConfigurationss(joint_result);

  std::shared_ptr<Manipulator_Controller> controller(new Manipulator_Controller());
  // controller->trajectoryFromArray(joint_result);

  //////////////////////////////////////////////////// THE PRE-DEFINED TASK-SPACE CURVE///////////////////////////
  std::cout << "YT: we define the task-space curve. " << std::endl;

  std::vector<double> homeConfiguration = {0.0, -M_PI/2, 0.0, -M_PI/2, 0.0, 0.0};

  // std::string file_name = "/home/yangtong/prm_ws/src/tntp_implementation/data/case_study_1";
  std::string file_name;
  nh.getParam("/asympTNTP/task_space_motion", file_name);

  std::string input_file_name = file_name + ".txt";

  std::vector<TaskSpaceWaypoint> TSpoints;
  TSpoints = case_study_1_matrix44(input_file_name);

  // std::cout << "YT: We do collision checking for all the IKs" << std::endl;
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  for(auto iter = TSpoints.begin(); iter != TSpoints.end(); ++iter)
  {
  	int i = 0;
  	while(i < iter->iks_.size()){
  		std::vector<double> temp = iter->iks_[i];
  			current_state.setJointGroupPositions(joint_model_group, temp);

  		collision_result.clear();
  		planning_scene->checkCollision(collision_request, collision_result);

  		if(collision_result.collision)
  			iter->iks_.erase(iter->iks_.begin()+i);

  		else
  			++i;
  	}
  	// std::cout << "Task-space point " << iter-TSpoints.begin() << " has " << iter->iks_.size() << " valid IKs" << std::endl;
  }

  /////////////////////////////////////////WE CREATE CONTINUOUS TRACKING MOTIONS//////////////////////////////////
	std::vector<ContinuousTrackingMotion> CTM;

	double epsilon_cont = 0.1;
	optimal_tntp_algorithm::defineStates(TSpoints, epsilon_cont, CTM);
	// Note that here the continuous sets do not have an index

	// We create the home configuration as the first continuous set, and adjust all data structures
	ContinuousTrackingMotion CTM0;
	CTM0.color_ = 0;
	CTM0.first_point_index_ = 0;
	CTM0.last_point_index_ = 0;
	CTM.insert(CTM.begin(), CTM0);
	for (auto iter = CTM.begin() + 1; iter != CTM.end(); ++iter)
	{
		iter->color_ = iter - CTM.begin();
		iter->first_point_index_++;
		iter->last_point_index_++;
	}

	std::cout << "We check all CTMs: " << std::endl;
	for(auto iter = CTM.begin(); iter != CTM.end(); ++iter)
	{
		std::cout << "[color, first_point_index, last_point_index] = [" << iter->color_ << ", " << iter->first_point_index_ << ", " << iter->last_point_index_ << "]" << std::endl;
	}

	for(auto iter = TSpoints.begin(); iter != TSpoints.end(); ++iter)
	{
		for(auto iter2 = iter->color_.begin(); iter2 != iter->color_.end(); ++iter2)
		{
			++*iter2;
		}
	}

  // We insert the home configuration as the first TSpoint
  TaskSpaceWaypoint homePoint;
  homePoint.color_.push_back(0);
  homePoint.iks_.push_back(homeConfiguration);
  TSpoints.insert(TSpoints.begin(), homeConfiguration);

  ////////We find all TNTP solutions with minimum number of reconfigurations
  std::vector<std::vector<int> > min_reconfig_solution;
  min_reconfig_tntp(CTM, min_reconfig_solution);

  

  std::vector<int> valid_first_ctm;
  unsigned long bool_flag = 0;
  for(unsigned int i = 0; i < min_reconfig_solution.size(); ++i)
  {
    int ctm_index = min_reconfig_solution[i][1];
    if (bool_flag & 1 << ctm_index == 0)
    {
      bool_flag |= 1 << ctm_index;
      valid_first_ctm.push_back(ctm_index);
    }
  }

  std::vector<std::vector<optimal_tntp::RobotState> > all_first_rms;
  std::vector<double> all_first_rms_cost;

  // We use the reconfiguration motion of the shortest length
  for(unsigned int i = 0; i < valid_first_ctm.size(); ++i)
  {
    optimal_tntp::RobotState source(TSpoints[0].getConfiguration(0));
    optimal_tntp::RobotState target(TSpoints[1].getConfiguration(valid_first_ctm[i]));
    std::vector<optimal_tntp::RobotState> path_temp;
    our_prmstar.findPath(source, target, path_temp);
    all_first_rms.push_back(path_temp);
    all_first_rms_cost.push_back(optimal_tntp::pathLength(path_temp));
  }

  // We choose the RM with shortest length
  int loc = std::min_element(all_first_rms_cost.begin(), all_first_rms_cost.end()) - all_first_rms_cost.begin();

  std::vector<optimal_tntp::RobotState> result_motion;
  result_motion.clear();

  std::vector<optimal_tntp::RobotState> interpolated_first_rm(all_first_rms[loc]);
  
  controller::linearInterpolateConfigurations(interpolated_first_rm);

  result_motion.insert(result_motion.end(), interpolated_first_rm.begin(), interpolated_first_rm.end()); 

  // prev_CTM_index: the CTM that the manipulator stays
  int prev_CTM_index = 0;

  // next_CTM_index: the CTM that the manipulator will reach (hence the "current CTM" of the planner)
  int next_CTM_index = valid_first_ctm[loc];

  int last_TSpoint_index = 0;

  ros::WallTime command_publish_time = ros::WallTime::now();
  int command_end_index = 0;

  std::vector<optimal_tntp::RobotState> latest_motion_segment(result_motion.begin()+command_end_index, result_motion.end());


  controller->trajectoryFromArray(latest_motion_segment);
  command_end_index = result_motion.size()-1;

  ros::WallTime command_next_publish_time = command_publish_time + ros::WallDuration(controller::getExpectedExecutionTime6D(latest_motion_segment, joint_space_6d_velocity));


////////The asympTNTP algorithm starts here

  // We find all TNTP motions with the least number of reconfiguration motions
  optimal_tntp_algorithm::OptimalTNTPSolver asymp_tntp_solver(&TSpoints, &CTM, &our_prmstar);
  asymp_tntp_solver.showDetails();

  // We store the goal CTMs indices, so that we know when the manipulator reaches the last CTM
  unsigned long goal_CTMs_binary = 0;
  for(auto iter = TSpoints.back().color_.begin(); iter != TSpoints.back().color_.end(); ++iter)
  {
    goal_CTMs_binary |= 1 << *iter;
  }

  while( ((1 << next_CTM_index) & goal_CTMs_binary) == 0 )
  {
    // We eliminate the RMs that will not be visited after the manipulator decides the first CTM that it will visit. 
    asymp_tntp_solver.eliminateRMs(prev_CTM_index, next_CTM_index);

    // We optimise the roadmap
    // We preserve one second for updating all RMs
    ros::WallTime optimisation_end_time = command_next_publish_time - ros::WallDuration(1.0);
    while(ros::WallTime::now() < optimisation_end_time)
    {
      // We do optimisation here      
    }

    // We update all RMs
    asymp_tntp_solver.updateAllRMs();

    // Stage 5: Whole TNTP Solution Generation 
    // Here the solver only collects all the RMs that might still be usable
    // std::vector<optimal_tntp::RobotState> initial_whole_solution(initial_motion.begin(), initial_motion.end());
    std::vector<std::vector<int> > all_possible_solution;
    all_possible_tntp(CTM, all_possible_solution, next_CTM_index);

    std::cout << "YT: There are still " << all_possible_solution.size() << " different topological motions when the manipulator is about to visit CTM " << next_CTM_index << std::endl;
    double min_cost = 10000.0;
    int min_cost_index = -1;
    int min_cost_first_CTM_end_index = -1;
    std::vector<int> min_topo_motion;
    std::vector<optimal_tntp::RobotState> min_cost_motion;

    for(unsigned int i = 0; i < all_possible_solution.size(); ++i)
    {
      std::vector<optimal_tntp::RobotState> path_in_specific_topology;
      
      // The starting point of the motion must be exactly after "the last task-space index whose configuration has been sent to the manipulator"
      int first_CTM_end_index;
      asymp_tntp_solver.reportSolutionInSpecificTopology(all_possible_solution[i], path_in_specific_topology, last_TSpoint_index, first_CTM_end_index);

      double cost = optimal_tntp::pathLength(path_in_specific_topology);

      if(cost < min_cost)
      {
        min_cost_index = i;
        min_cost_first_CTM_end_index = first_CTM_end_index;
        min_topo_motion.assign(all_possible_solution[i].begin(), all_possible_solution[i].end());
        min_cost_motion.assign(path_in_specific_topology.begin(), path_in_specific_topology.end());
      }

    }

    // We have got the updated current optimal motion. But we will still only send the first part of it
    latest_motion_segment.clear();
    for(auto iter = min_cost_motion.begin(); iter != min_cost_motion.end(); ++iter)
    {
      latest_motion_segment.emplace_back(*iter);
      if(iter->joint_.back() == -1 && std::next(iter)->joint_.back() != -1)
      {
        break;
      }
    }

    std::cout << "We check the latest_motion_segment: " << std::endl;
    for(auto iter = latest_motion_segment.begin(); iter != latest_motion_segment.end(); ++iter)
    {
      for(auto iter2 = iter->joint_.begin(); iter2 != iter->joint_.end(); ++iter2)
      {
        std::cout << *iter2 << " ";
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;

    // update all parameters
    prev_CTM_index = next_CTM_index;
    next_CTM_index = min_topo_motion[1];
    last_TSpoint_index = min_cost_first_CTM_end_index;

  }




  ros::spin();

  /**
   * Let's cleanup everything, shutdown ros and join the thread
   */
  ros::shutdown();

  return 0;
}
