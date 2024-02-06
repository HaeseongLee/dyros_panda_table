#include "interface.h"


shape_msgs::SolidPrimitive MoveitInterface::defineCylinder(vector<double>& dim){
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[primitive.CYLINDER_HEIGHT] = dim[0];
  primitive.dimensions[primitive.CYLINDER_RADIUS] = dim[1];
  return primitive;
  
}

void MoveitInterface::loadObject(string name){
    std::cout<<name<<std::endl;
  string group_name;
  if (!nh_.getParam("obj_group", group_name)){
    ROS_FATAL_STREAM("Object group is not given!");
    return;
  }
  vector<double> position;
  vector<double> euler_zyx;
  if (!nh_.getParam(group_name+"/"+name+"/dim", dimensions_[name])){
    ROS_FATAL_STREAM("Object dimension is not given!");
    return;
  }
  if (!nh_.getParam(group_name+"/"+name+"/position", position)){
    ROS_FATAL_STREAM("Object position is not given!");
    return;
  }
  if (!nh_.getParam(group_name+"/"+name+"/orientation", euler_zyx)){
    ROS_FATAL_STREAM("Object orientation is not given!");
    return;
  }

  moveit_msgs::CollisionObject obj;
  obj.header.frame_id = move_group_->getPlanningFrame();

  // The id of the object is used to identify it.
  obj.id = name;

  // Define a cylinder to add to the world.
  shape_msgs::SolidPrimitive primitive = defineCylinder(dimensions_[name]);

  // Define a pose for the object (specified relative to frame_id)
  geometry_msgs::Pose pose;
  pose.position.x = position[0];
  pose.position.y = position[1];
  int isLeaning = dimensions_[name][2];
  pose.position.z = position[2] + dimensions_[name][isLeaning] / (2-isLeaning);

  double d2r = M_PI/180;
  tf2::Quaternion orientation;
  orientation.setEuler(euler_zyx[0]*d2r, euler_zyx[1]*d2r, euler_zyx[2]*d2r);
  pose.orientation = tf2::toMsg(orientation);

  obj.primitives.push_back(primitive);
  obj.primitive_poses.push_back(pose);
  obj.operation = obj.ADD;

  // Define subframes of the object (top, bottom)
  obj.subframe_names.resize(2);
  obj.subframe_poses.resize(2);

  obj.subframe_names[0] = "top";
  obj.subframe_poses[0].position.z = dimensions_[name][0]/2;
  obj.subframe_poses[0].orientation.w = 1;

  obj.subframe_names[1] = "bottom";
  obj.subframe_poses[1].position.z = -dimensions_[name][0]/2;
  obj.subframe_poses[1].orientation.x = 1;

  collision_objects_.push_back(obj);
}

moveit_msgs::CollisionObject MoveitInterface::moveObject(string obj_id, string ref_frame,
                                                         array<float, 3> position,
                                                         array<float, 4> orientation)
{
    geometry_msgs::Pose pose;
    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.position.z = position[2];
    pose.orientation.x = orientation[0];
    pose.orientation.y = orientation[1];
    pose.orientation.z = orientation[2];
    pose.orientation.w = orientation[3];

    moveit_msgs::CollisionObject obj;

    obj.header.frame_id = ref_frame;
    obj.id = obj_id;
    obj.pose = pose;
    obj.operation = obj.MOVE;

    return obj;
}

void MoveitInterface::attachObject(string obj_id, string end_effector, vector<string> touch_links)
{
	// It uses the frame_id to determine which robot link it is attached to.
  // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
	move_group_->attachObject(obj_id, end_effector, touch_links);
	attached_objects_.push_back(obj_id);
    ros::Duration(0.001).sleep();  // Pause the code for one second


}

void MoveitInterface::detachObject(string obj_id)
{
    move_group_->detachObject(obj_id);
    attached_objects_.erase(remove(attached_objects_.begin(), attached_objects_.end(), obj_id), attached_objects_.end());
    ros::Duration(0.001).sleep();  // Pause the code for one second

}

double MoveitInterface::getTrackingTime(moveit_msgs::RobotTrajectory msg)
{
  int num_waypoint = msg.joint_trajectory.points.size();
  return msg.joint_trajectory.points[num_waypoint-1].time_from_start.toSec();
}

bool MoveitInterface::execute(vector<targetPose> targets)
{
    visual_tools_->deleteAllMarkers();

    double start_time, end_time;
    moveit::planning_interface::MoveGroupInterface::Plan plan;

	move_group_->clearPoseTargets();
	move_group_->setStartStateToCurrentState();

	vector<targetPose>::iterator target;
	for (target = targets.begin(); target != targets.end(); target++){
		move_group_->setPoseTarget((*target).first, (*target).second);    
    }
	
    start_time = ros::Time::now().toSec();
	for(int i = 0; i < n_trials_; i++){
		moveit::planning_interface::MoveItErrorCode result = move_group_->plan(plan);
        if(result == moveit::planning_interface::MoveItErrorCode::SUCCESS){
            
            // Visualize the plan as a line in RViz.
            end_time = ros::Time::now().toSec();
            std::cout << "Time: "<< (end_time - start_time)<<std::endl;
            ROS_INFO_NAMED("dyros", "Trial %d: Planning succeed", (i+1));
            cout << "planning with RRTconnect took " << plan.planning_time_ << "\n";
            cout << "initial tracking time: " << getTrackingTime(plan.trajectory_) << "\n";
            visual_tools_->publishTrajectoryLine(plan.trajectory_, joint_model_group_, rvt::YELLOW);
            clock_t start_simplify = clock();

            // Do smoothing
            traj_simplifier simplifier = traj_simplifier(vmax_, velocity_scaling_factor_, move_group_, psm_, robot_model_, joint_model_group_);

            simplifier.set_max_iter(100);
            simplifier.set_time_step(0.2);
            simplifier.simplify_traj(plan);
            
            clock_t finish_simplify = clock();
            cout << "simplification took " << (double)(finish_simplify - start_simplify) / CLOCKS_PER_SEC << "\n";
            cout << "simplified tracking time: " << getTrackingTime(plan.trajectory_) << "\n";
            ROS_INFO_NAMED("dyros", "Visualizing plan as trajectory line");
            visual_tools_->publishTrajectoryLine(plan.trajectory_, joint_model_group_, rvt::BLUE);
            visual_tools_->trigger();
            move_group_->execute(plan);
            return true;
        }
        else
        {
        ROS_INFO_NAMED("dyros", "Trial %d: Planning failed", (i+1));
        if (result == moveit::planning_interface::MoveItErrorCode::GOAL_CONSTRAINTS_VIOLATED)
        {
            ROS_ERROR("Goal constraints violated. Please check the specified constraints.");
        }
        else if (result == moveit::planning_interface::MoveItErrorCode::NO_IK_SOLUTION)
        {
            ROS_ERROR("No IK solution found for the specified goal pose.");
        }
        else if (result == moveit::planning_interface::MoveItErrorCode::TIMED_OUT)
        {
            ROS_ERROR("Timed out.");
        }
        else
        {
            ROS_ERROR("Motion planning failed for an unknown reason.");
        }
        }

	}	
	return false;
}
bool MoveitInterface::execute_sub(vector<targetPose> targets,
                                  moveit::planning_interface::MoveGroupInterface& move_group,
                                  const moveit::core::JointModelGroup *joint_model_group){
    
    visual_tools_->deleteAllMarkers();

    double start_time, end_time;
    move_group.clearPoseTargets();
    move_group.setStartStateToCurrentState();

    vector<targetPose>::iterator target;
    for (target = targets.begin(); target != targets.end(); target++)
    {
        move_group.setPoseTarget((*target).first, (*target).second);
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;	
    start_time = ros::Time::now().toSec();
	for(int i = 0; i < n_trials_; i++){
		moveit::planning_interface::MoveItErrorCode result = move_group.plan(plan);
        if(result == moveit::planning_interface::MoveItErrorCode::SUCCESS){
			// Visualize the plan as a line in RViz.
            end_time = ros::Time::now().toSec();
            std::cout << "Time: "<< (end_time - start_time)<<std::endl;
            ROS_INFO_NAMED("dyros", "Trial %d: Planning succeed", (i+1));
            cout << "planning with RRTconnect took " << plan.planning_time_ << "s\n";
            cout << "initial tracking time: " << getTrackingTime(plan.trajectory_) << "s\n";
            visual_tools_->publishTrajectoryLine(plan.trajectory_, joint_model_group, rvt::YELLOW);
            clock_t start_simplify = clock();
            // traj_simplifier simplifier = traj_simplifier(vmax_, velocity_scaling_factor_, move_group_, psm_, robot_model_, joint_model_group_);
            traj_simplifier simplifier = traj_simplifier(vmax_, velocity_scaling_factor_, &move_group, psm_, robot_model_, joint_model_group);

            simplifier.set_max_iter(100);
            simplifier.set_time_step(0.1);
            simplifier.simplify_traj(plan);
            clock_t finish_simplify = clock();
            cout << "simplification took " << (double)(finish_simplify - start_simplify) / CLOCKS_PER_SEC << "s\n";
            cout << "simplified tracking time: " << getTrackingTime(plan.trajectory_) << "s\n";
			ROS_INFO_NAMED("dyros", "Visualizing plan as trajectory line");
			visual_tools_->publishTrajectoryLine(plan.trajectory_, joint_model_group, rvt::BLUE);
			visual_tools_->trigger();
			move_group.execute(plan);
			return true;
		}
        else
        {
            if (result == moveit::planning_interface::MoveItErrorCode::GOAL_CONSTRAINTS_VIOLATED)
            {
                ROS_ERROR("Goal constraints violated. Please check the specified constraints.");
            }
            else if (result == moveit::planning_interface::MoveItErrorCode::NO_IK_SOLUTION)
            {
                ROS_ERROR("No IK solution found for the specified goal pose.");
            }
            else if (result == moveit::planning_interface::MoveItErrorCode::TIMED_OUT)
            {
                ROS_ERROR("Timed out.");
            }
            else
            {
                ROS_ERROR("Motion planning failed for an unknown reason.");
            }
        }

	}
	return false;
}

void MoveitInterface::initJoint(){
    visual_tools_->deleteAllMarkers();

    double start_time, end_time;
    start_time = ros::Time::now().toSec();
    ROS_INFO_NAMED("dyros", "move to initial pose");
    for(vector<string>::iterator obj = attached_objects_.begin(); obj != attached_objects_.end(); obj++){
        detachObject(*obj);
    }
    attached_objects_.clear();

    // Set joint position to initial value
    move_group_->clearPoseTargets();
    move_group_->setStartStateToCurrentState();
    move_group_->setNamedTarget("init_triple");
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    move_group_->plan(plan);
    end_time = ros::Time::now().toSec();
    std::cout << "Time: "<< (end_time - start_time)<<std::endl;
    cout << "planning with RRTconnect took " << plan.planning_time_ << "s\n";
    cout << "initial tracking time: " << getTrackingTime(plan.trajectory_) << "s\n";
    visual_tools_->publishTrajectoryLine(plan.trajectory_, joint_model_group_, rvt::YELLOW);

    clock_t start_simplify = clock();
    traj_simplifier simplifier = traj_simplifier(vmax_, velocity_scaling_factor_, move_group_, psm_, robot_model_, joint_model_group_);
    simplifier.set_max_iter(100);
    // simplifier.set_time_step(0.1);
    simplifier.set_time_step(0.2);
    simplifier.simplify_traj(plan);
    clock_t finish_simplify = clock();
    cout << "simplification took " << (double)(finish_simplify - start_simplify) / CLOCKS_PER_SEC << "s\n";
    cout << "simplified tracking time: " << getTrackingTime(plan.trajectory_) << "s\n";
    ROS_INFO_NAMED("dyros", "Visualizing plan as trajectory line");
    visual_tools_->publishTrajectoryLine(plan.trajectory_, joint_model_group_);
    visual_tools_->trigger();
    move_group_->execute(plan);

}

void MoveitInterface::applyCollisionObjects(std::vector< moveit_msgs::CollisionObject > &objs){
      planning_scene_interface_.applyCollisionObjects(objs);
}

void MoveitInterface::showCollisionObjects()
{
    planning_scene_interface_.applyCollisionObjects(collision_objects_);
}

void MoveitInterface::setPlannerId(string planner_plugin_name){
    move_group_->setPlannerId(planner_plugin_name);   
}

void MoveitInterface::setPlanningTime(double time){
    move_group_->setPlanningTime(time);   
}

void MoveitInterface::setMaxVelocityScalingFactor(double x){
    move_group_->setMaxVelocityScalingFactor(x);   
}

void MoveitInterface::setNumPlanningAttempts(int x){
    move_group_->setNumPlanningAttempts(x);   
}

void MoveitInterface::removeCollisionObjects(vector<string> obj_names){
    visual_tools_-> deleteAllMarkers();
    planning_scene_interface_.removeCollisionObjects(obj_names);
}

std::vector<std::string> MoveitInterface::getKnownObjectNames(){
    return planning_scene_interface_.getKnownObjectNames();
}

void MoveitInterface::clearPathConstraints()
{
    move_group_->clearPathConstraints();
}

