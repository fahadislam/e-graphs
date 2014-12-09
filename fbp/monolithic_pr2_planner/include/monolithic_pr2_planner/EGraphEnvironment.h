#pragma once
#include <ros/ros.h>
#include <monolithic_pr2_planner/Environment.h>
#include <egraphs/egraph.h>
#include <egraphs/egraphable.h>
#include <vector>
#include <pviz/pviz.h>
#include<egraph_vis/egraph_visualizer.h>

namespace monolithic_pr2_planner {

    class EGraphEnvironment : public Environment, public EGraphable<std::vector<double> >, public EGraphMarkerMaker, public EGraphDiscretize {
        public:
            EGraphEnvironment(ros::NodeHandle nh);
            bool snap(const std::vector<double>& from, const std::vector<double>& to, int& id, int& cost);
            bool getCoord(int id, std::vector<double>& coord);
            bool isGoal(int id);
            int getStateID(const std::vector<double>& coord);
            bool isValidEdge(const std::vector<double>& start, const std::vector<double>& end, bool& change_cost, int& cost);
            bool isValidVertex(const std::vector<double>& coord);
            void projectToHeuristicSpace(const std::vector<double>& coord, std::vector<double>& heur_coord) const;
            void projectGoalToHeuristicSpace(std::vector<double>& heur_coord) const;
            void discToCont(const std::vector<int>& d, std::vector<double>& c);
            void contToDisc(const std::vector<double>& c, std::vector<int>& d);
            visualization_msgs::MarkerArray stateToVisualizationMarker(std::vector<double> coord);
            visualization_msgs::MarkerArray stateToDetailedVisualizationMarker(std::vector<double> coord);
            visualization_msgs::MarkerArray edgeToVisualizationMarker(std::vector<double> coord, std::vector<double> coord2);
        private:
            PViz pviz_;
            void printVector(std::vector<double> coord);
            std::vector<double> res_;
    };
}
