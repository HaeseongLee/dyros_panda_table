#include "interface.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dyros_cocktail_interface");
  ros::NodeHandle nh("~");

  std::string planning_group = "panda_left";

  MoveitInterface mi(nh, planning_group);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  mi.loadObject("glass");  
  mi.loadObject("j1");

  // display collision objects
  mi.showCollisionObjects();

  // set hyper parameters for motion planning
  mi.setMaxVelocityScalingFactor(0.5);
  mi.setPlanningTime(20.0);
  mi.setNumPlanningAttempts(5);
  
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  mi.robot_model_ = &robot_model_loader.getModel();
  mi.joint_model_group_ = mi.move_group_->getCurrentState()->getJointModelGroup(planning_group);

  std::string planner_plugin_name;
  double time;
  if(nh.getParam("planning_plugin", planner_plugin_name)){
    mi.setPlannerId(planner_plugin_name);
  }
  if(nh.getParam("planning_time", time)){
    mi.setPlanningTime(time);
  }

  // Variables used in planning
  // double gripper_offset = 0.09; // dyros_gripper
  double gripper_offset = 0.1034; // franka_hand

  std::vector<std::string> touch_links_left;
  touch_links_left.push_back("panda_left_hand");

  // Start planning
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  int character_input;
  std::vector<targetPose> targets;
  while(ros::ok()){
    ROS_INFO_NAMED("Dyros cocktail bar!",
              "==========================\n"
              "Press a key and hit Enter to execute an action.\n"
              "0 to exit\n"
              "1 to grasp objects\n"
              "2 to assemble\n"
              "3 to homing\n");
    std::cin >> character_input;

    switch(character_input){
      case 0: {
        
        // Remove the objects from the world.
        ROS_INFO_NAMED("dyros", "Remove the objects from the world");
        std::vector<std::string> obj_names = mi.getKnownObjectNames();
        mi.removeCollisionObjects(obj_names);

        
        // END_SIMULATION
        ros::shutdown();
        return 0;
      }
      case 1: {
      
        geometry_msgs::PoseStamped grasp_pose;
        grasp_pose.header.frame_id = "j1/top";
        grasp_pose.pose.orientation.x = -0.7071068;
        grasp_pose.pose.orientation.y = 0.7071068;
        grasp_pose.pose.position.z = (0.02 + gripper_offset);

        targets.clear();
        targets.push_back(make_pair(grasp_pose, "panda_left_hand"));        
        if(!mi.execute(targets)) break;

        mi.attachObject("j1", "panda_left_hand", touch_links_left);  
        mi.clearPathConstraints();
        break;
      }

      case 2: {
        geometry_msgs::PoseStamped pouring_pose;
        pouring_pose.header.frame_id = "glass/top";
        pouring_pose.pose.orientation.x = 0.0;
        pouring_pose.pose.orientation.y = 0.0;
        pouring_pose.pose.orientation.z = 0.0;
        pouring_pose.pose.orientation.w = 1.0;
        pouring_pose.pose.position.z = 0.1;
            
        targets.clear();
        targets.push_back(make_pair(pouring_pose, "j1/bottom"));
    
        if(!mi.execute(targets)) break;

        mi.clearPathConstraints();
        break;
      }

      case 3: {
        // mi.detachObject("side1");
        // mi.detachObject("side2");
        // mi.detachObject("inner1");
        // mi.detachObject("inner2");

        // vector<moveit_msgs::CollisionObject> moved_objects;

        // moved_objects.push_back(mi.moveObject("side1", "coil/top", {0.0, 0.0, 0.01}, {0, 0, 0, 1}));
        // moved_objects.push_back(mi.moveObject("side1", "coil/top", {0.0, 0.0, 0.01}, {0, 0, 0, 1}));
        // moved_objects.push_back(mi.moveObject("side2", "coil/bottom", {0.0, 0.0, 0.01}, {0, 0, 0, 1}));
        // moved_objects.push_back(mi.moveObject("inner1", "coil/top", {0.0, 0.0, 0.01}, {0, 0, 0, 1}));
        // moved_objects.push_back(mi.moveObject("inner2", "coil/bottom", {0.0, 0.0, 0.01}, {0, 0, 0, 1}));

        // mi.applyCollisionObjects(moved_objects);
        // mi.initJoint();
        // mi.clearPathConstraints();
        break;
      }

      default:
        ROS_INFO_NAMED("Dyros", "Invalid input. Please type other number");
        break;
    }  
  }


  ros::waitForShutdown();
  return 0;
}
