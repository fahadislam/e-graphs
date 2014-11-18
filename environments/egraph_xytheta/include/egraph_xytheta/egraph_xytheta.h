#ifndef EGRAPH_XYTHETA_H
#define EGRAPH_XYTHETA_H

#include<egraphs/egraph.h>
#include<egraphs/egraphable.h>
#include<egraphs/egraph_2d_grid_heuristic.h>
#include<egraphs/egraph_discretize.h>
#include<sbpl/headers.h>
#include<egraph_vis/egraph_visualizer.h>

class EGraphXYTheta: public EnvironmentNAVXYTHETALAT, public EGraphable<std::vector<int> >, public EGraphMarkerMaker, public EGraphDiscretize{
  public:
    EGraphXYTheta();
    bool collisionCheckPose(int x, int y, int theta, int& cost);
    bool snap(const std::vector<double>& from, const std::vector<double>& to, int& id, int& cost);
    virtual bool getCoord(int id, std::vector<double>& coord);
    virtual int getStateID(const std::vector<double>& coord);
    virtual bool isGoal(int id);
    void projectToHeuristicSpace(const std::vector<double>& coord, std::vector<int>& dp) const;
    void projectGoalToHeuristicSpace(std::vector<int>& dp) const;
    void contToDisc(const std::vector<double>& c, std::vector<int>& d);
    void discToCont(const std::vector<int>& d, std::vector<double>& c);
    virtual bool isValidEdge(const std::vector<double>& coord, const std::vector<double>& coord2, bool& change_cost, int& cost);
    virtual bool isValidVertex(const std::vector<double>& coord);
    visualization_msgs::MarkerArray stateToVisualizationMarker(std::vector<double> coord);
    visualization_msgs::MarkerArray stateToDetailedVisualizationMarker(std::vector<double> coord);
    visualization_msgs::MarkerArray edgeToVisualizationMarker(std::vector<double> coord, std::vector<double> coord2);

    bool InitializeEnv(int width, int height,
        const unsigned char* mapdata,
        double startx, double starty, double starttheta,
        double goalx, double goaly, double goaltheta,
        double goaltol_x, double goaltol_y, double goaltol_theta,
        const std::vector<sbpl_2Dpt_t> & perimeterptsV,
        double cellsize_m, double nominalvel_mpersecs, double timetoturn45degsinplace_secs,
        unsigned char obsthresh,  const char* sMotPrimFile);

  private:
    std::vector<std::vector<sbpl_2Dcell_t> > footprints;
};

#endif
