#include <ros/ros.h>
#include <time.h>

#include "ur5e_kinematics.h"
#include "ur5e_controller.h"
#include "prmstar.h"
#include "input_data.h"
#include "environmental_obstacles.h"
#include "definitions.h"

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
  const double joint_speed_velocity = 0.5;

  ros::init(argc, argv, "ur5e_optimal_tntp_planner");

  ros::NodeHandle nh;

  const std::string PLANNING_GROUP = "manipulator";

  // YT: we construct basic structures
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr &kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(kinematic_model));

  std::cout << "YT: We create basic scene structures" << std::endl;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  moveit::planning_interface::MoveGroupInterface ur5e_("manipulator");

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
  for (unsigned int i = 0; i < 100; ++i)
    our_prmstar.add_milestone();
  
//////////////////////////////////////////////////// THE PRE-DEFINED TASK-SPACE CURVE///////////////////////////
	std::cout << "YT: we define the task-space curve. " << std::endl;
	// std::string file_name = "/home/yangtong/PRM_test/src/ur5_moveit_perception/data/case_study_1";
	std::string file_name = "/home/mintnguyen/Documents/NRMDTS_Implementation/tntp_implementation/data/case_study_1";
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




  ros::spin();

  /**
   * Let's cleanup everything, shutdown ros and join the thread
   */
  ros::shutdown();

  return 0;
}
