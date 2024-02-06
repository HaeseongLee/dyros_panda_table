#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <cmath>
#include <iostream>
#include <map>
#include <random>
#include <traj_simplifier.h>

using namespace std;
namespace rvt = rviz_visual_tools;

typedef pair<geometry_msgs::PoseStamped, string> targetPose;
double vmax_[] = {2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100};


class MoveitInterface{

    public:
        ros::NodeHandle &nh_;
        const string planning_group_;
        moveit::planning_interface::MoveGroupInterface* move_group_;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
        const moveit::core::JointModelGroup* joint_model_group_;
        const moveit::core::RobotModelPtr* robot_model_;

        planning_scene_monitor::PlanningSceneMonitorPtr psm_;
        moveit_visual_tools::MoveItVisualTools* visual_tools_;
        

        double velocity_scaling_factor_ = 0.3;
        int n_trials_ = 3; // number of trials for motion planning

        MoveitInterface(ros::NodeHandle &nh, const string pg) : nh_(nh), planning_group_(pg)
        {
            move_group_ = new moveit::planning_interface::MoveGroupInterface(planning_group_);             
            visual_tools_ = new moveit_visual_tools::MoveItVisualTools("base");
            psm_ = visual_tools_->getPlanningSceneMonitor();
        }

        // ~MoveitInterface(){
        //     delete move_group_;          
        //     delete visual_tools_;
        // }

        shape_msgs::SolidPrimitive defineCylinder(vector<double>& dim);
        void loadObject(string name);
        moveit_msgs::CollisionObject moveObject(string obj_id, string ref_frame, array<float, 3> position, array<float, 4> orientation);

        void attachObject(string obj_id, string end_effector, vector<string> touch_links);
        void detachObject(string obj_id);
        double getTrackingTime(moveit_msgs::RobotTrajectory msg);        

        bool execute(vector<targetPose> targets);

        bool execute_sub(vector<targetPose> targets,
                         moveit::planning_interface::MoveGroupInterface& move_group,
                         const moveit::core::JointModelGroup *joint_model_group);

        void initJoint();

        map<string, vector<double>> dimensions_;
        vector<moveit_msgs::CollisionObject> collision_objects_;
        vector<string> attached_objects_;

        void applyCollisionObjects(std::vector<moveit_msgs::CollisionObject> &objs);
        void showCollisionObjects();
        void setPlannerId(string planner_plugin_name);
        void setPlanningTime(double time);
        void setMaxVelocityScalingFactor(double x);
        void setNumPlanningAttempts(int x);
        void removeCollisionObjects(vector<string> obj_names);
        std::vector<std::string> getKnownObjectNames();
        void clearPathConstraints();


    private:
    

};
