#include <ros/ros.h>
#include <signal.h>
#include <stdlib.h>
#include <leatherman/utils.h>
#include <leatherman/print.h>
#include <arm_navigation_msgs/PlanningScene.h>
#include <arm_navigation_msgs/GetMotionPlan.h>
#include <sbpl_manipulation_components/kdl_robot_model.h>
#include <sbpl_manipulation_components_pr2/pr2_kdl_robot_model.h>
#include <sbpl_manipulation_components_pr2/ubr1_kdl_robot_model.h>
#include <sbpl_collision_checking/sbpl_collision_space.h>
#include <egraph_arm/GetArmPlan.h>
#include <egraph_arm/AddPathsToEGraph.h>
#include <egraph_arm/egraph_arm_planner_interface.h>

class EGraphArmPlannerNode{
  public:

    void fillConstraint(geometry_msgs::Pose pose, std::string frame_id, arm_navigation_msgs::Constraints &goals);
    arm_navigation_msgs::CollisionObject getCollisionCube(geometry_msgs::Pose pose, std::vector<double> &dims, std::string frame_id, std::string id);
    std::vector<arm_navigation_msgs::CollisionObject> getCollisionCubes(std::vector<std::vector<double> > &objects, std::vector<std::string> &object_ids, std::string frame_id);
    std::vector<arm_navigation_msgs::CollisionObject> getCollisionObjects(std::string filename, std::string frame_id);
    bool getInitialConfiguration(ros::NodeHandle &nh, arm_navigation_msgs::RobotState &state, std::vector<double> angles, std::vector<string> names);
    bool callPlanner(egraph_arm::GetArmPlan::Request& req, egraph_arm::GetArmPlan::Response& res);
    bool addPathsToEGraph(egraph_arm::AddPathsToEGraph::Request& req, egraph_arm::AddPathsToEGraph::Response& res);

    EGraphArmPlannerNode();

  private:
    ros::NodeHandle nh;
    ros::NodeHandle ph;
    ros::Publisher ma_pub;
    std::string planning_frame;

    std::vector<double> start_angles;

    //distance_field::PropagationDistanceField *df;
    sbpl_arm_planner::RobotModel *rm;
    sbpl_arm_planner::OccupancyGrid *grid;
    //sbpl_arm_planner::CollisionChecker *cc;
    sbpl_arm_planner::SBPLCollisionSpace* cc;
    sbpl_arm_planner::ActionSet *as;

    sbpl_arm_planner::EGraphSBPLArmPlannerInterface* planner_;
    arm_navigation_msgs::PlanningScenePtr scene;
    ros::ServiceServer plan_service_;
    ros::ServiceServer add_path_service_;

    ros::Subscriber interrupt_sub_;
    void interruptPlannerCallback(std_msgs::EmptyConstPtr);
};

