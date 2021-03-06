#include <ros/ros.h>
#include <egraphs/egraph.h>

#include <egraphs/egraphManager.h>
#include <egraphs/egraph_euclidean_heuristic.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <egraph_xy/GetXYPlan.h>
#include <egraph_xy/egraph_xy.h>
#include <egraphs/egraph_planner.h>

class EGraphXYNode{
  public:
    EGraphXYNode(costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(egraph_xy::GetXYPlan::Request& req, egraph_xy::GetXYPlan::Response& res);

  private:
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D cost_map_;

    EGraphXY* env_;
    EGraph* egraph_;
    EGraphEuclideanHeuristic* heur_;
    EGraphManager<std::vector<double> >* egraph_mgr_;
    LazyAEGPlanner<std::vector<double> >* planner_;
    EGraphVisualizer* egraph_vis_;

    ros::Publisher plan_pub_;
    ros::ServiceServer plan_service_;

    ros::Subscriber interrupt_sub_;
    int num_islands;    //fadi
    void interruptPlannerCallback(std_msgs::EmptyConstPtr);
};
