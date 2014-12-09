#include<egraph_xytheta/egraph_xytheta.h>

using namespace std;

EGraphXYTheta::EGraphXYTheta(){
}


bool EGraphXYTheta::InitializeEnv(int width, int height,
                             const unsigned char* mapdata,
                             double startx, double starty, double starttheta,
                             double goalx, double goaly, double goaltheta,
                             double goaltol_x, double goaltol_y, double goaltol_theta,
                             const vector<sbpl_2Dpt_t> & perimeterptsV,
                             double cellsize_m, double nominalvel_mpersecs, double timetoturn45degsinplace_secs,
                             unsigned char obsthresh,  const char* sMotPrimFile){

  bool ret = EnvironmentNAVXYTHETALATTICE::InitializeEnv(width, height, mapdata, startx, starty, starttheta,
                                              goalx, goaly, goaltheta, goaltol_x, goaltol_y, goaltol_theta,
                                              perimeterptsV, cellsize_m, nominalvel_mpersecs, 
                                              timetoturn45degsinplace_secs, obsthresh, sMotPrimFile);

  if(ret){
    footprints.resize(EnvNAVXYTHETALATCfg.NumThetaDirs);
    for(int i=0; i<EnvNAVXYTHETALATCfg.NumThetaDirs; i++){
      sbpl_xy_theta_pt_t pt;
      pt.x = 0;
      pt.y = 0;
      pt.theta = DiscTheta2Cont(i, EnvNAVXYTHETALATCfg.NumThetaDirs);
      get_2d_footprint_cells(EnvNAVXYTHETALATCfg.FootprintPolygon, &(footprints[i]), pt, EnvNAVXYTHETALATCfg.cellsize_m);
    }
  }
  return ret;
}

bool EGraphXYTheta::collisionCheckPose(int x, int y, int theta, int& cost){
  if(!IsValidCell(x, y))
    return false;

  if(EnvNAVXYTHETALATCfg.Grid2D[x][y] >= EnvNAVXYTHETALATCfg.cost_inscribed_thresh)
    return false;

  //check collisions that for the particular footprint orientation along the action
  if(EnvNAVXYTHETALATCfg.FootprintPolygon.size() > 1 && EnvNAVXYTHETALATCfg.Grid2D[x][y] >= EnvNAVXYTHETALATCfg.cost_possibly_circumscribed_thresh){ 
    for(unsigned int i=0; i<footprints[theta].size(); i++){ 
      //check validity
      if(!IsValidCell(x+footprints[theta][i].x, y+footprints[theta][i].y))
        return false;
    }
  }

  cost = EnvNAVXYTHETALATCfg.Grid2D[x][y]; 
  return true;
}

bool EGraphXYTheta::snap(const vector<double>& from, const vector<double>& to, int& id, int& cost){
  int idFrom = getStateID(from);
  int idTo = getStateID(to);
  EnvNAVXYTHETALATHashEntry_t* hashFrom = StateID2CoordTable[idFrom];
  EnvNAVXYTHETALATHashEntry_t* hashTo = StateID2CoordTable[idTo];

  if(hashFrom->X==hashTo->X && hashFrom->Y==hashTo->Y){
    //if it's the same coordinate don't snap
    if(hashFrom->Theta==hashTo->Theta){
      return false;
    }

    int dir;
    int dang = hashTo->Theta - hashFrom->Theta;
    if(fabs(dang) > EnvNAVXYTHETALATCfg.NumThetaDirs/2){
      if(dang > 0)
        dir = -1;
      else
        dir = 1;
    }
    else{
      if(dang > 0)
        dir = 1;
      else
        dir = -1;
    }

    int max_cost = 0;
    for(int theta = hashFrom->Theta; theta!=hashTo->Theta;){
      if(idFrom==131629)
        ROS_INFO("theta=%d, fromTheta=%d, toTheta=%d, dir=%d",theta,hashFrom->Theta,hashTo->Theta,dir);
      int temp_cost;
      if(!collisionCheckPose(hashFrom->X,hashFrom->Y,theta,temp_cost))
        return false;
      if(temp_cost > max_cost)
        max_cost = temp_cost;

      theta = theta+dir;
      if(theta == EnvNAVXYTHETALATCfg.NumThetaDirs)
        theta = 0;
      if(theta == -1)
        theta = EnvNAVXYTHETALATCfg.NumThetaDirs - 1;
    }

    double angular_distance = fabs(computeMinUnsignedAngleDiff(DiscTheta2Cont(hashTo->Theta, EnvNAVXYTHETALATCfg.NumThetaDirs),
          DiscTheta2Cont(hashFrom->Theta, EnvNAVXYTHETALATCfg.NumThetaDirs)));
    double angular_time = angular_distance/((PI_CONST/4.0)/EnvNAVXYTHETALATCfg.timetoturn45degsinplace_secs);
    cost = (int)(ceil(NAVXYTHETALAT_COSTMULT_MTOMM*angular_time)) * (max_cost+1);
    id = idTo;
    return true;
  }
  return false;
}

//requires a getCoord function which takes a state id (the ids the environment uses) and returns a vector with the coordinate so we can look for shortcuts in the e-graph
//-we will never call getCoord on the goal (because it is possible we don't know what the goal state looks like)
bool EGraphXYTheta::getCoord(int id, vector<double>& coord){
  EnvNAVXYTHETALATHashEntry_t* hashEntry = StateID2CoordTable[id];
  vector<int> d_coord(3,0);
  d_coord[0] = hashEntry->X;
  d_coord[1] = hashEntry->Y;
  d_coord[2] = hashEntry->Theta;
  discToCont(d_coord,coord);
  return true;
}

int EGraphXYTheta::getStateID(const vector<double>& coord){
  vector<int> d_coord;
  contToDisc(coord,d_coord);
  return GetStateFromCoord(d_coord[0],d_coord[1],d_coord[2]);
}

bool EGraphXYTheta::isGoal(int id){
  return id == EnvNAVXYTHETALAT.goalstateid;
}

void EGraphXYTheta::projectToHeuristicSpace(const vector<double>& coord, vector<int>& dp) const{
  int x = CONTXY2DISC(coord[0], EnvNAVXYTHETALATCfg.cellsize_m);
  int y = CONTXY2DISC(coord[1], EnvNAVXYTHETALATCfg.cellsize_m);
  dp.clear();
  dp.push_back(x);
  dp.push_back(y);
  //ROS_INFO("project (%f %f) -> (%d %d)",coord[0],coord[1],dp[0],dp[1]);
}

void EGraphXYTheta::projectGoalToHeuristicSpace(vector<int>& dp) const{
  EnvNAVXYTHETALATHashEntry_t* hashEntry = StateID2CoordTable[EnvNAVXYTHETALAT.goalstateid];
  dp.clear();
  dp.push_back(hashEntry->X);
  dp.push_back(hashEntry->Y);
  //ROS_INFO("project goal -> (%d %d)",dp[0],dp[1]);
}

void EGraphXYTheta::contToDisc(const vector<double>& c, vector<int>& d){
  d.resize(3);
  PoseContToDisc(c[0],c[1],c[2],d[0],d[1],d[2]);
}

void EGraphXYTheta::discToCont(const vector<int>& d, vector<double>& c){
  c.resize(3);
  PoseDiscToCont(d[0],d[1],d[2],c[0],c[1],c[2]);
}

bool EGraphXYTheta::isValidEdge(const vector<double>& coord, const vector<double>& coord2, bool& change_cost, int& cost){
  int id1 = getStateID(coord);
  int id2 = getStateID(coord2);

  vector<int> children;
  vector<int> costs;

  GetSuccs(id1, &children, &costs, NULL);
  for(unsigned int i=0; i<children.size(); i++){
    if(children[i]==id2){
      change_cost = true;
      cost = costs[i];
      return true;
    }
  }
  GetSuccs(id2, &children, &costs, NULL);
  for(unsigned int i=0; i<children.size(); i++){
    if(children[i]==id1){
      change_cost = true;
      cost = costs[i];
      return true;
    }
  }

  int id;
  if(snap(coord, coord2, id, cost)){
    change_cost = true;
    return true;
  }

  //TODO: and demonstrations?

  change_cost = false;
  return false;
}

bool EGraphXYTheta::isValidVertex(const vector<double>& coord){
  vector<int> d_coord;
  contToDisc(coord,d_coord);
  int temp_cost;
  return collisionCheckPose(d_coord[0],d_coord[1],d_coord[2],temp_cost);
}

visualization_msgs::MarkerArray EGraphXYTheta::stateToVisualizationMarker(vector<double> coord){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.pose.position.x = coord[0];
  marker.pose.position.y = coord[1];
  marker.pose.position.z = 0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  visualization_msgs::MarkerArray ma;
  ma.markers.push_back(marker);
  return ma;
}

visualization_msgs::MarkerArray EGraphXYTheta::stateToDetailedVisualizationMarker(vector<double> coord){
  return stateToVisualizationMarker(coord);
}

visualization_msgs::MarkerArray EGraphXYTheta::edgeToVisualizationMarker(vector<double> coord, vector<double> coord2){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.scale.x = 0.01;
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  geometry_msgs::Point p;
  p.x = coord[0];
  p.y = coord[1];
  p.z = 0;
  marker.points.push_back(p);
  p.x = coord2[0];
  p.y = coord2[1];
  marker.points.push_back(p);

  visualization_msgs::MarkerArray ma;
  ma.markers.push_back(marker);
  return ma;
}

