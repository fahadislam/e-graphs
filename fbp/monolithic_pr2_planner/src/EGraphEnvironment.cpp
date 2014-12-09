#include <monolithic_pr2_planner/EGraphEnvironment.h>

using namespace std;
using namespace monolithic_pr2_planner;

EGraphEnvironment::EGraphEnvironment(ros::NodeHandle nh):Environment(nh){ 
  res_.push_back(ContObjectState::getXYZResolution());
  res_.push_back(ContObjectState::getXYZResolution());
  res_.push_back(ContObjectState::getXYZResolution());
  res_.push_back(ContObjectState::getRPYResolution());
  res_.push_back(ContObjectState::getRPYResolution());
  res_.push_back(ContObjectState::getRPYResolution());
  res_.push_back(ContArmState::getFreeAngleResolution());
  res_.push_back(ContArmState::getFreeAngleResolution());
  res_.push_back(ContBaseState::getXYZResolution());
  res_.push_back(ContBaseState::getXYZResolution());
  res_.push_back(ContBaseState::getXYZResolution());
  res_.push_back(ContBaseState::getThetaResolution());
}

bool EGraphEnvironment::snap(const vector<double>& from_vec, const vector<double>& to_vec, int& id, int& cost){
    vector<double> from_v = from_vec;
    vector<double> to_v = to_vec;
    // sigh. ghetto hack.
    from_v[GraphStateElement::R_FA] = from_v[from_v.size()-2];
    from_v[GraphStateElement::L_FA] = from_v[from_v.size()-1];

    to_v[GraphStateElement::R_FA] = to_v[to_v.size()-2];
    to_v[GraphStateElement::L_FA] = to_v[to_v.size()-1];

    GraphState from_state(from_v);
    GraphStatePtr to_state = boost::make_shared<GraphState>(to_v);

    // TODO hack to get left arm to something proper
    vector<double> left_arm_start = DefaultLeftAngles;
    LeftContArmState l_arm(left_arm_start);
    RobotState tmp = from_state.robot_pose();
    tmp.left_arm(l_arm);
    from_state.robot_pose(tmp);

    tmp = to_state->robot_pose();
    tmp.left_arm(l_arm);
    to_state->robot_pose(tmp);

    if (from_state == *to_state){
        return false;
    } else {
        ROS_WARN("Got a snap!");
        //std::cin.get();
        vector<RobotState> interpolated_states;
        RobotState::workspaceInterpolate(from_state.robot_pose(), 
                                         to_state->robot_pose(), 
                                         &interpolated_states);
        from_state.robot_pose().visualize();
        for (auto& state : interpolated_states){
            if (!m_cspace_mgr->isValid(state)){
                ROS_INFO("snap failed");
                return false;
            }
        }
    }
    // the sbpl secret sauce
    cost = 1;
    GraphStatePtr temp_state = boost::make_shared<GraphState>(to_vec);
    id = m_hash_mgr->getStateID(temp_state);
    //ROS_INFO("found a valid snap at %d", id);
    return true;
}

// returns things in base frame object state
bool EGraphEnvironment::getCoord(int id, vector<double>& coord){
    GraphStatePtr state = m_hash_mgr->getGraphState(id);
    coord = state->getContCoords();

    // ghetto hack to save the continuous arm angle
    coord.push_back(state->robot_pose().right_arm().getUpperArmRollAngle());
    coord.push_back(state->robot_pose().left_arm().getUpperArmRollAngle());
    return true;
}

/*
bool EGraphEnvironment::getGoalCoord(const vector<double>& parent_coord, vector<double>& goal){
    GraphStatePtr parent = boost::make_shared<GraphState>(parent_coord);
    for (auto mprim : m_mprims.getMotionPrims()){
        GraphStatePtr successor;
        TransitionData t_data;
        if (mprim->apply(*parent, successor, t_data)){
            if (m_goal->isSatisfiedBy(successor)){
                DiscObjectState goal_obj = successor->getObjectStateRelMap();
                vector<int> int_goal = goal_obj.getCoords();
                for (auto value : int_goal){
                    goal.push_back(value);
                }
                ROS_INFO("goal is ");
                printVector(goal);
                return true;
            }
        }
    }
    return false;
}
*/

void EGraphEnvironment::projectGoalToHeuristicSpace(vector<double>& h_coord) const{
    GraphStatePtr state = m_complete_goal;
    vector<double> coord = state->getContCoords();

    // ghetto hack to save the continuous arm angle
    coord.push_back(state->robot_pose().right_arm().getUpperArmRollAngle());
    coord.push_back(state->robot_pose().left_arm().getUpperArmRollAngle());

    projectToHeuristicSpace(coord, h_coord);
}

bool EGraphEnvironment::isGoal(int id){
  if(id == m_complete_goal->id())
    return true;

  GraphStatePtr state = m_hash_mgr->getGraphState(id);
  vector<int> coord = state->getCoords();
  vector<int> goal_coord = m_complete_goal->getCoords();

  for(unsigned int i=0; i<coord.size(); i++){
    if(i==GraphStateElement::R_FA || i==GraphStateElement::L_FA)
      continue;
    if(coord[i]!=goal_coord[i])
      return false;
  }
  ROS_WARN("Reached goal without free angle!");
  std::cin.get();
  return true;
}

int EGraphEnvironment::getStateID(const vector<double>& coord){
    GraphStatePtr state = boost::make_shared<GraphState>(coord);
    state->printToDebug(HASH_LOG);
    m_hash_mgr->save(state);//if the state didn't exist, this adds it to the HashManager
    return m_hash_mgr->getStateID(state);
}

bool EGraphEnvironment::isValidEdge(const vector<double>& start, const vector<double>& end, bool& change_cost, int& cost){
    GraphStatePtr start_state = boost::make_shared<GraphState>(start);
    GraphStatePtr end_state = boost::make_shared<GraphState>(end);
    vector<RobotState> interpolated_states;
    RobotState::workspaceInterpolate(start_state->robot_pose(), 
                                     end_state->robot_pose(), 
                                     &interpolated_states);
    for (auto& state : interpolated_states){
        if (!m_cspace_mgr->isValid(state)){
            return false;
        }
    }
    change_cost = false;
    //cost = 1000;
    return true;
}

bool EGraphEnvironment::isValidVertex(const vector<double>& coord){
  GraphStatePtr state = boost::make_shared<GraphState>(coord);
  RobotState rs = state->robot_pose();
  if(!m_cspace_mgr->isValid(rs)){
    return false;
  }
  return true;
}

// the obj xyzrpy in coord is in base link frame!
void EGraphEnvironment::projectToHeuristicSpace(const std::vector<double>& coord, std::vector<double>& h_coord) const{
  //arm xyz
  //right free angle
  //base center xy
  //base xy heading (point 1m out along heading from center of robot)
  //torso z
  h_coord.resize(9);
  
  h_coord[0] = coord[GraphStateElement::OBJ_X];
  h_coord[1] = coord[GraphStateElement::OBJ_Y];
  h_coord[2] = coord[GraphStateElement::OBJ_Z];
  h_coord[3] = coord[GraphStateElement::R_FA];
  h_coord[4] = coord[GraphStateElement::BASE_X];
  h_coord[5] = coord[GraphStateElement::BASE_Y];
  h_coord[6] = cos(coord[GraphStateElement::BASE_THETA]);
  h_coord[7] = sin(coord[GraphStateElement::BASE_THETA]);
  h_coord[8] = coord[GraphStateElement::BASE_Z];
}

void EGraphEnvironment::discToCont(const vector<int>& d, vector<double>& c){
  c.resize(d.size());
  for(unsigned int i=0; i<d.size(); i++)
    c[i] = d[i]*res_[i];
}

double signedRound(double r) {
  return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);
}

void EGraphEnvironment::contToDisc(const vector<double>& c, vector<int>& d){
  d.resize(c.size());
  for(unsigned int i=0; i<c.size(); i++)
    d[i] = signedRound( c[i]/res_[i] );
}

void EGraphEnvironment::printVector(vector<double> coord){
    for (auto value : coord){
        printf("%f ", value);
    }
    printf("\n");
}

visualization_msgs::MarkerArray EGraphEnvironment::stateToVisualizationMarker(vector<double> coord){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.type = visualization_msgs::Marker::SPHERE;

    GraphState state(coord);
    ContObjectState obj = state.getObjectStateRelMapFromState();

    marker.pose.position.x = obj.x();
    marker.pose.position.y = obj.y();
    marker.pose.position.z = obj.z();
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

visualization_msgs::MarkerArray EGraphEnvironment::stateToDetailedVisualizationMarker(vector<double> coord){
    GraphState state(coord);
    BodyPose pose = state.robot_pose().base_state().getBodyPose();
    vector<double> r_arm = state.robot_pose().right_arm().getAngles();
    vector<double> l_arm = state.robot_pose().left_arm().getAngles();
    visualization_msgs::MarkerArray ma = pviz_.getRobotMarkerMsg(r_arm, l_arm, pose, 250, "detailed state", 0);
    for(unsigned int i=0; i<ma.markers.size(); i++)
        ma.markers[i].header.frame_id = "/map";
    return ma;
}

visualization_msgs::MarkerArray EGraphEnvironment::edgeToVisualizationMarker(vector<double> coord, vector<double> coord2){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.scale.x = 0.01;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    geometry_msgs::Point p;

    GraphState state(coord);
    ContObjectState obj = state.getObjectStateRelMapFromState();

    p.x = obj.x();
    p.y = obj.y();
    p.z = obj.z();
    marker.points.push_back(p);

    GraphState state2(coord2);
    ContObjectState obj2 = state2.getObjectStateRelMapFromState();
    p.x = obj2.x();
    p.y = obj2.y();
    p.z = obj2.z();
    marker.points.push_back(p);

    visualization_msgs::MarkerArray ma;
    ma.markers.push_back(marker);
    return ma;
}
