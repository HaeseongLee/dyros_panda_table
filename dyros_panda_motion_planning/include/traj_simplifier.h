#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <cmath>

using namespace std;

class traj_simplifier{
  private:
    double max_iter;
    double time_step;

    double* vmax;
    double velocity_scaling_factor;

    moveit::planning_interface::MoveGroupInterface* move_group_interface;
    const moveit::core::RobotModelPtr* robot_model_ptr;
    const moveit::core::JointModelGroup* joint_model_group;
    const moveit::core::RobotState* current_state;
    planning_scene_monitor::PlanningSceneMonitorPtr psm;

  public:
    traj_simplifier(double vmax_[], double velocity_scaling_factor_, 
                    moveit::planning_interface::MoveGroupInterface* move_group_interface_,
                    planning_scene_monitor::PlanningSceneMonitorPtr psm_,
                    const moveit::core::RobotModelPtr* robot_model_ptr,
                    const moveit::core::JointModelGroup* joint_model_group_);

    void set_max_iter(double max_iter_);
    void set_time_step(double time_step_);

    double parabolic_min_time(int joint_num, double q_0, double q_f, double v_0, double v_f);
    pair<double, double*> parabolic_acc(int joint_num, double q_0, double q_f, double v_0, double v_f, double T_max);
    robot_trajectory::RobotTrajectory parabolic_traj(std_msgs::Header header_, vector<string> joint_names_, 
                                                     trajectory_msgs::JointTrajectoryPoint start, 
                                                     trajectory_msgs::JointTrajectoryPoint end);
    void simplify_traj(moveit::planning_interface::MoveGroupInterface::Plan& plan);
};

traj_simplifier::traj_simplifier(double vmax_[], double velocity_scaling_factor_, 
                                 moveit::planning_interface::MoveGroupInterface* move_group_interface_,
                                 planning_scene_monitor::PlanningSceneMonitorPtr psm_,
                                 const moveit::core::RobotModelPtr* robot_model_ptr_,
                                 const moveit::core::JointModelGroup* joint_model_group_){
  // constructor
  vmax = vmax_;
  velocity_scaling_factor = velocity_scaling_factor_;
  move_group_interface = move_group_interface_;
  psm = psm_;
  robot_model_ptr = robot_model_ptr_;
  joint_model_group = joint_model_group_;

  // set default values
  max_iter = 100;
  time_step = 0.5;
  // time_step = 10.0;

}

void traj_simplifier::set_max_iter(double max_iter_){
  max_iter = max_iter_;
}

void traj_simplifier::set_time_step(double time_step_){
  time_step = time_step_;
}

double traj_simplifier::parabolic_min_time(int joint_num, double q_0, double q_f, double v_0, double v_f){
  double v_max = vmax[joint_num] * velocity_scaling_factor;
  // double a_max = amax[joint_num] * acceleration_scaling_factor;  // using robot's acceleration limit
  double a_max = 1.0;   // using default acceleration limit
  double e = 1e-4;

  double t = 0;
  double s = ((v_f + v_0) / 2) * (abs(v_f - v_0) / a_max);  // displacement w/o additional accelration & decceleration
  if((q_f - q_0) - s > e){
    // P+P-
    double v_p = sqrt(a_max * (q_f - q_0) + (pow(v_f, 2) + pow(v_0, 2)) / 2);
    t = (2 * v_p - v_f - v_0) / a_max;
    if (v_p > v_max){
      // P+L+P-
      t = t + pow(v_p - v_max, 2) / (a_max * v_max);
    }
  }
  else if((q_f - q_0) - s < -e){
    // P-P+
    double v_p = -sqrt(a_max * (q_0 - q_f) + (pow(v_f, 2) + pow(v_0, 2)) / 2);
    t = (v_f + v_0- 2 * v_p) / a_max;
    if (v_p < -v_max){
      // P-L-P+
      t = t + pow(v_p + v_max, 2) / (a_max * v_max);
    }
  }
  else{
    // P
    t = abs(v_f - v_0) / a_max;
  }

  return t;
}

pair<double, double*> traj_simplifier::parabolic_acc(int joint_num, double q_0, double q_f, double v_0, double v_f, double T_max){
  double v_max = vmax[joint_num] * velocity_scaling_factor;
  double e = 1e-4;

  double b = T_max * (v_0 + v_f) + 2 * (q_0 - q_f);
  double s = ((v_f + v_0) / 2) * T_max;   // displacement w/o additional accelration & decceleration
  if((q_f - q_0) - s > e){
    // P+P-
    double acc = (-b + sqrt(pow(b, 2) + pow(T_max * (v_f - v_0), 2))) / pow(T_max, 2);
    double t_s = 0.5 * (T_max + (v_f - v_0) / acc);
    if(v_0 + acc * t_s <= v_max){
      double T[2] = {t_s, t_s};
      return make_pair(acc, T);
    }
    // P+L+P-
    else{
      acc = (pow(v_max, 2) - v_max * (v_0 + v_f) + 0.5 * (pow(v_0, 2) + pow(v_f, 2))) / (T_max * v_max - q_f + q_0);
      double T[2] = {(v_max - v_0) / acc, T_max - ((v_max - v_f) / acc)};
      return make_pair(acc, T);
    }
  }
  else if((q_f - q_0) - s <  - e){
    // P-P+
    double acc = (b + sqrt(pow(b, 2) + pow(T_max * (v_f - v_0), 2))) / pow(T_max, 2);
    double t_s = 0.5 * (T_max + (v_0 - v_f) / acc);
    if(v_0 - acc * t_s >= -v_max){
      double T[2] = {t_s, t_s};
      return make_pair(-acc, T);
    }
    // P-L-P+
    else{
      acc = (pow(v_max, 2) + v_max * (v_0 + v_f) + 0.5 * (pow(v_0, 2) + pow(v_f, 2))) / (-T_max * v_max - q_f + q_0);
      double T[2] = {(-v_max - v_0) / acc, T_max - ((-v_max - v_f) / acc)};
      return make_pair(acc, T);
    }
  }
  else{
    // P or L
    double acc = (v_f - v_0) / T_max;
    double T[2] = {T_max, T_max};
    return make_pair(acc, T);
  }
}

robot_trajectory::RobotTrajectory traj_simplifier::parabolic_traj(std_msgs::Header header_, vector<string> joint_names_, 
                                                                  trajectory_msgs::JointTrajectoryPoint start, 
                                                                  trajectory_msgs::JointTrajectoryPoint end){
  int joint_num = joint_names_.size();

  // find required time for each joint with maxium acceleration & velocity
  double T_max = 0;
  for(int j = 0; j < joint_num; j++){
    double q_0 = start.positions[j];
    double v_0 = start.velocities[j];
    double q_f = end.positions[j];
    double v_f = end.velocities[j];
    T_max = max(T_max, parabolic_min_time(j%7, q_0, q_f, v_0, v_f));
  }

  // find required acceleration with fixed time for each joint
  double a[joint_num];
  double t_s[joint_num][2];
  for(int j = 0; j < joint_num; j++){
    double q_0 = start.positions[j];
    double v_0 = start.velocities[j];
    double q_f = end.positions[j];
    double v_f = end.velocities[j];
    pair<double, double*> result = parabolic_acc(j%7, q_0, q_f, v_0, v_f, T_max);
    a[j] = result.first;
    t_s[j][0] = result.second[0];
    t_s[j][1] = result.second[1];
  }

  // make trajectory from acquired acceleration and t_s
  moveit_msgs::RobotTrajectory simplified_traj;
  int num_points = (int)(T_max / time_step) + 1;
  double dt = T_max / num_points;
  simplified_traj.joint_trajectory.header = header_;
  simplified_traj.joint_trajectory.joint_names = joint_names_;
  for(int i = 0; i < num_points + 1; i++){
    trajectory_msgs::JointTrajectoryPoint point;
    double t_ = dt * i;
    point.time_from_start = ros::Duration(t_);
    for(int j = 0; j < joint_num; j++){
      double q_0 = start.positions[j];
      double v_0 = start.velocities[j];
      double q_f = end.positions[j];
      double v_f = end.velocities[j];
      double v_max = vmax[j%7] * velocity_scaling_factor;

      double a_, v_, q_;
      if(t_ <= t_s[j][0]){
        a_ = a[j];
        v_ = v_0 + a_ * t_;
        q_ = q_0 + v_0 * t_ + 0.5 * a_ * pow(t_, 2);
      }
      else if(t_ > t_s[j][1]){
        a_ = -a[j];
        v_ = v_f - a_ * (T_max - t_);
        q_ = q_f - v_ * (T_max - t_) - 0.5 * a_ * pow(T_max - t_, 2);
      }
      else{
        a_ = 0;
        if(a[j] > 0){
          v_ = v_max;
          q_ = q_0 + 0.5 * (v_0 + v_max) * t_s[j][0] + v_max * (t_ - t_s[j][0]);
        }
        else{
          v_ = -v_max;
          q_ = q_0 + 0.5 * (v_0 - v_max) * t_s[j][0] - v_max * (t_ - t_s[j][0]);
        }
      }

      point.positions.push_back(q_);
      point.velocities.push_back(v_);
      point.accelerations.push_back(a_);
    }
    simplified_traj.joint_trajectory.points.push_back(point);
  }
  robot_trajectory::RobotTrajectory robot_trajectory(*robot_model_ptr, joint_model_group);
  robot_trajectory.setRobotTrajectoryMsg(*current_state, simplified_traj);

  return robot_trajectory;
}

void traj_simplifier::simplify_traj(moveit::planning_interface::MoveGroupInterface::Plan& plan){
  psm->requestPlanningSceneState("/get_planning_scene");
  planning_scene_monitor::LockedPlanningSceneRO planning_scene(psm);
  current_state = &planning_scene->getCurrentState();

  for(int iter = 0; iter < max_iter; iter++){
    int num_waypoint = plan.trajectory_.joint_trajectory.points.size();
    if (num_waypoint < 3) return;
    srand(clock());
    int start = round((double)(rand()) / RAND_MAX * (num_waypoint - 3));
    int end = start + 2 + round((double)(rand()) / RAND_MAX * (num_waypoint - 3 - start));
    robot_trajectory::RobotTrajectory simplified_traj = parabolic_traj(plan.trajectory_.joint_trajectory.header, 
                                                                       plan.trajectory_.joint_trajectory.joint_names,
                                                                       plan.trajectory_.joint_trajectory.points[start],
                                                                       plan.trajectory_.joint_trajectory.points[end]);
    // check if the new trajectory is collision free
    string group_name = move_group_interface->getName();
    if(planning_scene->isPathValid(simplified_traj, group_name)){
      robot_trajectory::RobotTrajectory new_traj(*robot_model_ptr, joint_model_group);
      robot_trajectory::RobotTrajectory prev_traj(*robot_model_ptr, joint_model_group);
      prev_traj.setRobotTrajectoryMsg(*current_state, plan.trajectory_);

      double dt_front = plan.trajectory_.joint_trajectory.points[start].time_from_start.toSec() - plan.trajectory_.joint_trajectory.points[max(start-1, 0)].time_from_start.toSec();
      new_traj.append(prev_traj, 0.0, 0, start);
      new_traj.append(simplified_traj, dt_front);
      new_traj.append(prev_traj, 0.0, end + 1);

      new_traj.getRobotTrajectoryMsg(plan.trajectory_);
    }
  }
}