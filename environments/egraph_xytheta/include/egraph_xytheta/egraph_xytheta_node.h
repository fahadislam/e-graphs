#ifndef EGRAPH_XYTHETA_NODE
#define EGRAPH_XYTHETA_NODE

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <egraph_xytheta/GetXYThetaPlan.h>
#include <egraph_xytheta/egraph_xytheta.h>
#include <egraphs/egraphManager.h>
#include <egraphs/egraph_planner.h>

class EGraphXYThetaNode{
  public:
    EGraphXYThetaNode(costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(egraph_xytheta::GetXYThetaPlan::Request& req, 
                  egraph_xytheta::GetXYThetaPlan::Response& res);

  private:
    unsigned char costMapCostToSBPLCost(unsigned char newcost);

    std::string cost_map_topic_; /** what topic is being used for the costmap topic */
    std::string primitive_filename_; /** where to find the motion primitives for the current robot */
    unsigned char lethal_obstacle_;
    unsigned char inscribed_inflated_obstacle_;
    unsigned char sbpl_cost_multiplier_;
    costmap_2d::Costmap2DROS* costmap_ros_; /**< manages the cost map for us */
    costmap_2d::Costmap2D cost_map_;        /**< local copy of the costmap underlying cost_map_ros_ */
    std::vector<geometry_msgs::Point> footprint_;

    EGraphXYTheta* env_;
    EGraph* egraph_;
    EGraph2dGridHeuristic* heur_;
    EGraphManager<std::vector<int> >* egraph_mgr_;
    LazyAEGPlanner<std::vector<int> >* planner_;
    EGraphVisualizer* egraph_vis_;

    ros::Publisher plan_pub_;
    ros::ServiceServer plan_service_;

    ros::Subscriber interrupt_sub_;
    void interruptPlannerCallback(std_msgs::EmptyConstPtr);
};

#endif

