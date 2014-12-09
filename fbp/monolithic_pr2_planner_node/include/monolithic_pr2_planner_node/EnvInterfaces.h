#pragma once
#include <monolithic_pr2_planner/Environment.h>
#include <monolithic_pr2_planner/EGraphEnvironment.h>
#include <monolithic_pr2_planner_node/CollisionSpaceInterface.h>
#include <monolithic_pr2_planner_node/GetMobileArmPlan.h>
#include <boost/shared_ptr.hpp>
#include <string>
#include <Eigen/Core>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <egraphs/egraph_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <egraphs/egraph_3d_grid_heuristic.h>
#include <egraphs/egraph_euclidean_heuristic.h>
#include <monolithic_pr2_planner_node/StatsWriter.h>
#include <monolithic_pr2_planner/PathPostProcessor.h>
#include <std_srvs/Empty.h>

#include <monolithic_pr2_planner_node/ompl_pr2_planner.h>
#include <monolithic_pr2_planner/ExperimentFramework/randomStartGoalGenerator.h>

namespace monolithic_pr2_planner_node {
    struct InterfaceParams {
        std::string ref_frame;
    };

    class EnvInterfaces {
        public:
            EnvInterfaces(boost::shared_ptr<monolithic_pr2_planner::EGraphEnvironment> env,
                ros::NodeHandle nh);
            void getParams();
            bool planPathCallback(GetMobileArmPlan::Request &req, 
                                  GetMobileArmPlan::Response &res);
            void bindPlanPathToEnv(std::string service_name);
            void bindExperimentToEnv(std::string service_name);
            bool bindCollisionSpaceToTopic(std::string topic_name);
            void bindNavMapToTopic(std::string topic_name);
            void bindWriteExperimentToEnv(string service_name);
            void packageStats(std::vector<std::string>& stat_names,
                              std::vector<double>& stats,
                              int solution_cost,
                              size_t solution_size,
                              double total_planning_time);
            bool GenerateExperimentFile(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);



            void startOMPLPlanners();
            void weirdEGraphInit(std::vector<string>& names, std::vector<double>& min,
                                 std::vector<double>& max, std::vector<double>& res);

        private:
            void loadNavMap(const nav_msgs::OccupancyGridPtr& map);
            void crop2DMap(const nav_msgs::MapMetaData& map_info, const std::vector<unsigned char>& v,
                           double new_origin_x, double new_origin_y,
                           double width, double height);
            void interruptPlannerCallback(std_msgs::EmptyConstPtr);
            ros::NodeHandle m_nodehandle;
            InterfaceParams m_params;
            std::unique_ptr<EGraphVisualizer> m_egraph_vis;

            EGraphPtr egraph_;
            boost::shared_ptr<monolithic_pr2_planner::EGraphEnvironment> m_env;
            tf::TransformListener m_tf;
            std::unique_ptr<CollisionSpaceInterface> m_collision_space_interface;
            ros::ServiceServer m_plan_service;
            ros::ServiceServer m_experiment_service;
            ros::ServiceServer m_write_experiments_service;
            std::unique_ptr<LazyAEGPlanner<vector<double> > > m_ara_planner;
            ros::Subscriber m_nav_map;
            ros::Publisher m_costmap_pub;
            std::vector<signed char> m_final_map;
            std::vector<unsigned char> m_cropped_map;

            ros::Subscriber interrupt_sub_;
            boost::mutex mutex;
            
            // Doesn't have the need to store the Costmap2D object. Simply has
// to update the costmap of the heurMgr.
            std::unique_ptr<costmap_2d::Costmap2DROS> m_costmap_ros;
            std::unique_ptr<costmap_2d::Costmap2DPublisher> m_costmap_publisher;
            //EGraph3dGridHeuristic* heur_;
            EGraphEuclideanHeuristic* heur_;
            std::unique_ptr<StartGoalGenerator> m_generator;
            std::unique_ptr<OMPLPR2Planner> m_rrt;
            std::unique_ptr<OMPLPR2Planner> m_prm;
            std::unique_ptr<OMPLPR2Planner> m_rrtstar;
            std::unique_ptr<OMPLPR2Planner> m_rrtstar_first_sol;
            StatsWriter m_stats_writer;
    };
}