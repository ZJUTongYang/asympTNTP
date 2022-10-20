#include <cstdlib>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene/planning_scene.h>
#include <geometry_msgs/Pose.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/version.h>


void addObstacles(const moveit::planning_interface::PlanningSceneInterface& planning_scene_interface_, 
    const moveit::planning_interface::MoveGroupInterface& ur5_)
{
  // We add all environmental obstacles here

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = ur5_.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "ground"; // YT: we must have a ground plane, as gazebo does not allow the manipulator to use elbow-down configurations

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 2.0;
  primitive.dimensions[primitive.BOX_Y] = 2.0;
  primitive.dimensions[primitive.BOX_Z] = 0.01;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.0;
  box_pose.position.y = 0.0;
  box_pose.position.z = -0.01;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  planning_scene_interface_.addCollisionObjects(collision_objects);
}


void addObstacles_case_study_1(const moveit::planning_interface::PlanningSceneInterface& planning_scene_interface_, 
    const moveit::planning_interface::MoveGroupInterface& ur5_)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = ur5_.getPlanningFrame();

    // The id of the object is used to identify it.
    collision_object.id = "ground"; // YT: we must have a ground plane, as gazebo does not allow the manipulator to use elbow-down configurations
    // Define a box to add to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 2.0;
    primitive.dimensions[primitive.BOX_Y] = 1.2;
    primitive.dimensions[primitive.BOX_Z] = 0.01;

    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.28;

    // collision_object.pose = box_pose;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);


    // The id of the object is used to identify it.
    collision_object.id = "block 1";
    // Define a box to add to the world.
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.1;
    primitive.dimensions[primitive.BOX_Y] = 0.7;
    primitive.dimensions[primitive.BOX_Z] = 0.1;

    // Define a pose for the box (specified relative to frame_id)
    box_pose.orientation.w = 1.0;
    box_pose.position.x = -0.42;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.4;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);


    // The id of the object is used to identify it.
    collision_object.id = "block 2";
    // Define a box to add to the world.
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.2;
    primitive.dimensions[primitive.BOX_Y] = 0.7;
    primitive.dimensions[primitive.BOX_Z] = 0.15;

    // Define a pose for the box (specified relative to frame_id)
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.02;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.08;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);


   // The id of the object is used to identify it.
    collision_object.id = "sphere";
    // Define a box to add to the world.
    primitive.type = primitive.SPHERE;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.SPHERE_RADIUS] = 0.05;

    // Define a pose for the box (specified relative to frame_id)
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.3;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.5;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);

    // Now, let's add the collision object into the world
    planning_scene_interface_.addCollisionObjects(collision_objects);
}

void addObstacles_case_study_2(const moveit::planning_interface::PlanningSceneInterface& planning_scene_interface_, 
    const moveit::planning_interface::MoveGroupInterface& ur5e_)
{

    std::vector<moveit_msgs::CollisionObject> collision_objects;

    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = ur5e_.getPlanningFrame();

    // The id of the object is used to identify it.
    collision_object.id = "ground"; // YT: we must have a ground plane, as gazebo does not allow the manipulator to use elbow-down configurations
    // Define a box to add to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 1.2;
    primitive.dimensions[primitive.BOX_Y] = 1.2;
    primitive.dimensions[primitive.BOX_Z] = 0.01;

    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 0.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.28;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);

    // The id of the object is used to identify it.
    collision_object.id = "block 1"; // YT: we must have a ground plane, as gazebo does not allow the manipulator to use elbow-down configurations
    // Define a box to add to the world.
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.05;
    primitive.dimensions[primitive.BOX_Y] = 0.05;
    primitive.dimensions[primitive.BOX_Z] = 0.05;

    // Define a pose for the box (specified relative to frame_id)
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.05;
    box_pose.position.y = -0.30;
    box_pose.position.z = 0.15;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);

   // The id of the object is used to identify it.
    collision_object.id = "sphere"; // YT: we must have a ground plane, as gazebo does not allow the manipulator to use elbow-down configurations
    // Define a box to add to the world.
    primitive.type = primitive.SPHERE;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.SPHERE_RADIUS] = 0.05;

    // Define a pose for the box (specified relative to frame_id)
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.15;
    box_pose.position.y = 0.3;
    box_pose.position.z = 0.35;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);

    // Now, let's add the collision object into the world
    planning_scene_interface_.addCollisionObjects(collision_objects);
}

void addObstacles_case_study_real_world(const moveit::planning_interface::PlanningSceneInterface& planning_scene_interface_, 
    const moveit::planning_interface::MoveGroupInterface& ur5_)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = ur5_.getPlanningFrame();

    // The id of the object is used to identify it.
    collision_object.id = "ground"; // YT: we must have a ground plane, as gazebo does not allow the manipulator to use elbow-down configurations
    // Define a box to add to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 1.2;
    primitive.dimensions[primitive.BOX_Y] = 1.2;
    primitive.dimensions[primitive.BOX_Z] = 0.01;

    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 0.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.28;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);

    // The id of the object is used to identify it.
    collision_object.id = "block 1"; // YT: we must have a ground plane, as gazebo does not allow the manipulator to use elbow-down configurations
    // Define a box to add to the world.
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.68;
    primitive.dimensions[primitive.BOX_Y] = 0.70;
    primitive.dimensions[primitive.BOX_Z] = 0.4;

    // Define a pose for the box (specified relative to frame_id)
    box_pose.orientation.w = 1.0;
    box_pose.position.x = -0.42;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.045;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);

    // The id of the object is used to identify it.
    collision_object.id = "block 2"; // YT: we must have a ground plane, as gazebo does not allow the manipulator to use elbow-down configurations
    // Define a box to add to the world.
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.2;
    primitive.dimensions[primitive.BOX_Y] = 0.7;
    primitive.dimensions[primitive.BOX_Z] = 0.15;

    // Define a pose for the box (specified relative to frame_id)
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.02;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.08;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);

    // The id of the object is used to identify it.
    collision_object.id = "table"; // YT: we must have a ground plane, as gazebo does not allow the manipulator to use elbow-down configurations
    // Define a box to add to the world.
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.4;
    primitive.dimensions[primitive.BOX_Y] = 0.6;
    primitive.dimensions[primitive.BOX_Z] = 0.3;

    // Define a pose for the box (specified relative to frame_id)
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.4;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.13;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);

    // The id of the object is used to identify it.
    collision_object.id = "bbox_object"; // YT: we must have a ground plane, as gazebo does not allow the manipulator to use elbow-down configurations
    // Define a box to add to the world.
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.35;
    primitive.dimensions[primitive.BOX_Y] = 0.4;
    primitive.dimensions[primitive.BOX_Z] = 0.1;

    // Define a pose for the box (specified relative to frame_id)
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.4;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.07;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);

    // Now, let's add the collision object into the world
    planning_scene_interface_.addCollisionObjects(collision_objects);
}