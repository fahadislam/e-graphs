#include <monolithic_pr2_planner/Environment.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/DiscBaseState.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/MotionPrimitives/ArmAdaptiveMotionPrimitive.h>
#include <monolithic_pr2_planner/MotionPrimitives/FullBodyAdaptiveMotionPrimitive.h>
#include <monolithic_pr2_planner/Visualizer.h>
#include <monolithic_pr2_planner/Constants.h>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <assert.h>

using namespace monolithic_pr2_planner;
using namespace boost;

// stateid2mapping pointer inherited from sbpl interface. needed for planner.
Environment::Environment(ros::NodeHandle nh) : 
    m_hash_mgr(new HashManager(&StateID2IndexMapping)), 
    m_nodehandle(nh), m_mprims(m_goal), 
    m_heur_mgr(new HeuristicMgr()) {
    m_param_catalog.fetch(nh);
    configurePlanningDomain();
    m_using_lazy = true;
}

/**
 * @brief Resets the environment.
 * @details Intended to be used between calls to subsequent planning
 * requests.
 */
void Environment::reset(){
    m_heur_mgr->reset();
    // m_heur_mgr->setCollisionSpaceMgr(m_cspace_mgr);
    m_hash_mgr.reset(new HashManager(&StateID2IndexMapping));
    m_edges.clear();

    // Fetch params again, in case they're being modified between calls.
    // m_param_catalog.fetch(m_nodehandle);
}

// does set up required whenever a search request comes in. this includes
// setting the planning type, and setting start and goal
bool Environment::configureRequest(SearchRequestParamsPtr search_request_params,
                                   int& start_id, int& goal_id){
  ROS_ERROR("huh %f %f", search_request_params->base_goal.x(), search_request_params->base_goal.y());
    SearchRequestPtr search_request = SearchRequestPtr(new SearchRequest(search_request_params));
    configureQuerySpecificParams(search_request);
    //if (!setStartGoal(search_request, start_id, goal_id)){
    if (!setCompleteStartGoal(search_request, start_id, goal_id)){
        return false;
    }
    return true;
}

int Environment::GetGoalHeuristic(int stateID){
    return GetGoalHeuristic(stateID, 0);
}

int Environment::GetGoalHeuristic(int stateID, int goal_id){
    int heuristic_id = 0;
    // For now, return the max of all the heuristics
    // This vector is of size NUM_MHA_BASE_HEUR + 2
    // Eg, if NUM_MHA_BASE_HEUR is 2, that means there are 2 additional base
    // heuristics. So, the values will be endEff, Base, Base1, Base2
    GraphStatePtr successor = m_hash_mgr->getGraphState(stateID);
    std::unique_ptr<stringintmap> values;
    m_heur_mgr->getGoalHeuristic(successor, values);
    //std::vector<int> values = m_heur_mgr->getGoalHeuristic(m_hash_mgr->getGraphState(stateID));
    switch (heuristic_id) {
      case 0:  // Anchor
        return std::max((*values).at("admissible_endeff"), (*values).at("admissible_base"));
      case 1:  // ARA Heur 
        return std::max((*values).at("admissible_endeff"), (*values).at("admissible_base"));
      case 2:  // Base1, Base2 heur
        return static_cast<int>(0.5f*(*values).at("base_with_rot_0") + 0.5f*(*values).at("endeff_rot_goal"));
      case 3:
        return static_cast<int>(0.5f*(*values).at("base_with_rot_door") + 0.5f*(*values).at("endeff_rot_vert"));
    }
    return std::max((*values).at("admissible_endeff"), (*values).at("admissible_base"));
}

/*
 * Evaluates the edge. Assumes that the edge has already been generated and we
 * know the motion primitive used
 */
int Environment::GetTrueCost(int parentID, int childID){
    GraphStatePtr child = m_hash_mgr->getGraphState(childID);
    TransitionData t_data;

    vector<MotionPrimitivePtr> small_mprims;
    if (m_edges.find(Edge(parentID, childID)) == m_edges.end()){
        ROS_ERROR("transition hasn't been found between %d and %d??", parentID, childID);
        assert(false);
    }
    small_mprims.push_back(m_edges[Edge(parentID, childID)]);
    PathPostProcessor postprocessor(m_hash_mgr, m_cspace_mgr);

    ROS_DEBUG_NAMED(SEARCH_LOG, "evaluating edge (%d %d)", parentID, childID);
    GraphStatePtr source_state = m_hash_mgr->getGraphState(parentID);
    GraphStatePtr real_next_successor = m_hash_mgr->getGraphState(childID);
    GraphStatePtr successor;
    MotionPrimitivePtr mprim = m_edges.at(Edge(parentID, childID));
    if (!mprim->apply(*source_state, successor, t_data)){
        return -1;
    }
    //mprim->printEndCoord();
    //source_state->printToInfo(SEARCH_LOG);
    //successor->printToInfo(SEARCH_LOG);
    successor->id(m_hash_mgr->getStateID(successor));

    // right before this point, the successor's graph state does not match the
    // stored robot state (because we modified the graph state without calling
    // ik and all that). this call updates the stored robot pose.
    real_next_successor->robot_pose(successor->robot_pose());

    bool matchesEndID = successor->id() == childID;
    assert(matchesEndID);

    bool valid_successor = (m_cspace_mgr->isValidSuccessor(*successor, t_data) && 
                            m_cspace_mgr->isValidTransitionStates(t_data));
    if (!valid_successor){
        return -1;
    }
    return t_data.cost();
}

void Environment::GetLazySuccsWithUniqueIds(int sourceStateID, vector<int>* succIDs, 
                           vector<int>* costs, std::vector<bool>* isTrueCost){

    if (!m_using_lazy){
      GetSuccs(sourceStateID, succIDs, costs);
      isTrueCost->clear();
      isTrueCost->resize(succIDs->size(), true);
      return;
    }

    vector<MotionPrimitivePtr> all_mprims = m_mprims.getMotionPrims();
    ROS_DEBUG_NAMED(SEARCH_LOG, "==================Expanding state %d==================", 
                    sourceStateID);
    succIDs->clear();
    succIDs->reserve(all_mprims.size());
    costs->clear();
    costs->reserve(all_mprims.size());

    GraphStatePtr source_state = m_hash_mgr->getGraphState(sourceStateID);
    ROS_DEBUG_NAMED(SEARCH_LOG, "Source state is:");
    source_state->robot_pose().printToDebug(SEARCH_LOG);
    if(m_param_catalog.m_visualization_params.expansions){
        source_state->robot_pose().visualize();
        usleep(10000);
    }

    /*
    for (auto mprim : all_mprims){
      mprim->printEndCoord();
      printf("cost %d\n",mprim->cost());
    }
    */

    for (auto mprim : all_mprims){
        //ROS_DEBUG_NAMED(SEARCH_LOG, "Applying motion:");
        //mprim->printEndCoord();
        GraphStatePtr successor;
        TransitionData t_data;

        if (mprim->motion_type() == MPrim_Types::ARM){
            successor.reset(new GraphState(*source_state));
            successor->lazyApplyMPrim(mprim->getEndCoord());
            //ROS_INFO("source/successor");
            //mprim->printEndCoord();
            //source_state->printToInfo(MPRIM_LOG);
            //successor->printToInfo(MPRIM_LOG);
            //ROS_INFO("done");
        } else {
            if (!mprim->apply(*source_state, successor, t_data)){
                //ROS_DEBUG_NAMED(MPRIM_LOG, "couldn't apply mprim");
                continue;
            }
        }
        m_hash_mgr->save(successor);
        Edge key; 
        succIDs->push_back(successor->id());
        key = Edge(sourceStateID, successor->id());
        m_edges.insert(map<Edge, MotionPrimitivePtr>::value_type(key, mprim));
        costs->push_back(mprim->cost());
        isTrueCost->push_back(false);
    }
}

void Environment::GetSuccs(int sourceStateID, vector<int>* succIDs, vector<int>* costs){

  ROS_DEBUG_NAMED(SEARCH_LOG,
      "==================Expanding state %d==================",
      sourceStateID);
  succIDs->clear();
  succIDs->reserve(m_mprims.getMotionPrims().size());
  costs->clear();
  costs->reserve(m_mprims.getMotionPrims().size());

  GraphStatePtr source_state = m_hash_mgr->getGraphState(sourceStateID);
  ROS_DEBUG_NAMED(SEARCH_LOG, "Source state is:");
  source_state->robot_pose().printToDebug(SEARCH_LOG);
  if (m_param_catalog.m_visualization_params.expansions) {
    RobotState expansion_pose = source_state->robot_pose();
    expansion_pose.visualize();
    usleep(5000);
  }
  for (auto mprim : m_mprims.getMotionPrims()) {
    ROS_DEBUG_NAMED(SEARCH_LOG, "Applying motion:");
    // mprim->printEndCoord();
    GraphStatePtr successor;
    TransitionData t_data;
    if (!mprim->apply(*source_state, successor, t_data)) {
      ROS_DEBUG_NAMED(MPRIM_LOG, "couldn't apply mprim");
      continue;
    }

    if (m_cspace_mgr->isValidSuccessor(*successor,t_data) &&
        m_cspace_mgr->isValidTransitionStates(t_data)){
      ROS_DEBUG_NAMED(SEARCH_LOG, "Source state is:");
      source_state->printToDebug(SEARCH_LOG);
      m_hash_mgr->save(successor);
      ROS_DEBUG_NAMED(MPRIM_LOG, "successor state with id %d is:",
          successor->id());
      successor->printToDebug(MPRIM_LOG);

      succIDs->push_back(successor->id());
      costs->push_back(mprim->cost());
      ROS_DEBUG_NAMED(SEARCH_LOG, "motion succeeded with cost %d", mprim->cost());
    } else {
      //successor->robot_pose().visualize();
      ROS_DEBUG_NAMED(SEARCH_LOG, "successor failed collision checking");
    }
  }
}


// sets the start and goal in the planner. Also informs the adaptive motion
// primitives about the goal state
bool Environment::setStartGoal(SearchRequestPtr search_request,
                               int& start_id, int& goal_id){
    RobotState start_pose(search_request->m_params->base_start, 
                         search_request->m_params->right_arm_start,
                         search_request->m_params->left_arm_start);
    ContObjectState obj_state = start_pose.getObjectStateRelMap();
    obj_state.printToInfo(SEARCH_LOG);

    m_edges.clear();

    if (!search_request->isValid(m_cspace_mgr)){
        obj_state.printToInfo(SEARCH_LOG);
        start_pose.visualize();
        //m_cspace_mgr->visualizeAttachedObject(start_pose);
        //usleep(1000);
        return false;
    }

    GraphStatePtr start_graph_state = make_shared<GraphState>(start_pose);
    m_hash_mgr->save(start_graph_state);
    start_id = start_graph_state->id();
    assert(*(m_hash_mgr->getGraphState(start_graph_state->id())) == *start_graph_state);

    ROS_INFO_NAMED(SEARCH_LOG, "Start state set to:");
    start_pose.printToInfo(SEARCH_LOG);
    obj_state.printToInfo(SEARCH_LOG);
    // start_pose.visualize();
    m_goal = search_request->createGoalState();

    if (m_hash_mgr->size() < 2){
        goal_id = saveFakeGoalState(start_graph_state);
    } else {
        goal_id = 1;
    }

    ROS_INFO_NAMED(SEARCH_LOG, "Goal state created:");
    ContObjectState c_goal = m_goal->getObjectState();
    c_goal.printToInfo(SEARCH_LOG);
    m_goal->visualize();

    // This informs the adaptive motions about the goal.
    ArmAdaptiveMotionPrimitive::goal(*m_goal);
    BaseAdaptiveMotionPrimitive::goal(*m_goal);

    // informs the heuristic about the goal
    m_heur_mgr->setGoal(*m_goal);

    return true;
}

// sets the start and goal in the planner. Also informs the adaptive motion
// primitives about the goal state
bool Environment::setCompleteStartGoal(SearchRequestPtr search_request,
                                       int& start_id, int& goal_id){
    RobotState start_pose(search_request->m_params->base_start, 
                         search_request->m_params->right_arm_start,
                         search_request->m_params->left_arm_start);
    ContObjectState start_obj_state = start_pose.getObjectStateRelMap();
    start_obj_state.printToInfo(SEARCH_LOG);

  ROS_ERROR("wha %f %f", search_request->m_params->base_goal.x(), search_request->m_params->base_goal.y());
    RobotState goal_pose(search_request->m_params->base_goal, 
                         search_request->m_params->right_arm_goal,
                         search_request->m_params->left_arm_goal);
    ContObjectState goal_obj_state = goal_pose.getObjectStateRelMap();
    goal_obj_state.printToInfo(SEARCH_LOG);

    m_edges.clear();

    start_pose.visualize(85,"start");
    goal_pose.visualize(0,"goal");
    if (!search_request->isValid(m_cspace_mgr)){
        return false;
    }

    GraphStatePtr start_graph_state = make_shared<GraphState>(start_pose);
    m_hash_mgr->save(start_graph_state);
    start_id = start_graph_state->id();
    assert(*(m_hash_mgr->getGraphState(start_graph_state->id())) == *start_graph_state);

    ROS_INFO_NAMED(SEARCH_LOG, "Start state set to:");
    start_pose.printToInfo(SEARCH_LOG);
    start_obj_state.printToInfo(SEARCH_LOG);
    // start_pose.visualize();


    GraphStatePtr goal_graph_state = make_shared<GraphState>(goal_pose);
    m_hash_mgr->save(goal_graph_state);
    goal_id = goal_graph_state->id();
    assert(*(m_hash_mgr->getGraphState(goal_graph_state->id())) == *goal_graph_state);

    ROS_INFO_NAMED(SEARCH_LOG, "Goal state set to:");
    goal_pose.printToInfo(SEARCH_LOG);
    goal_obj_state.printToInfo(SEARCH_LOG);
    // goal_pose.visualize();

    m_complete_goal = goal_graph_state;
    m_goal = search_request->createGoalState();//just so reconstruct doesn't crash
    ArmAdaptiveMotionPrimitive::goal(*m_goal);
    GraphState* goalptr = m_complete_goal.get();
    FullBodyAdaptiveMotionPrimitive::goal(goalptr);
    //m_goal 

    //m_goal = search_request->createGoalState();

    //if (m_hash_mgr->size() < 2){
        //goal_id = saveFakeGoalState(start_graph_state);
    //} else {
        //goal_id = 1;
    //}

    //ROS_INFO_NAMED(SEARCH_LOG, "Goal state created:");
    //ContObjectState c_goal = m_goal->getObjectState();
    //c_goal.printToInfo(SEARCH_LOG);
    //m_goal->visualize();

    // This informs the adaptive motions about the goal.
    //ArmAdaptiveMotionPrimitive::goal(*m_goal);
    //BaseAdaptiveMotionPrimitive::goal(*m_goal);

    // informs the heuristic about the goal
    //m_heur_mgr->setGoal(*m_goal);

    return true;
}

// a hack to reserve a goal id in the hash so that no real graph state is ever
// saved as the goal state id
int Environment::saveFakeGoalState(const GraphStatePtr& start_graph_state){
    GraphStatePtr fake_goal = make_shared<GraphState>(*start_graph_state);
    RobotState fake_robot_state = fake_goal->robot_pose();
    DiscBaseState fake_base = fake_robot_state.base_state();
    fake_base.x(0); fake_base.y(0); fake_base.z(0);
    fake_robot_state.base_state(fake_base);
    fake_goal->robot_pose(fake_robot_state);
    m_hash_mgr->save(fake_goal);
    int goal_id = fake_goal->id();
    return goal_id;
}

// this sets up the environment for things that are query independent.
void Environment::configurePlanningDomain(){
    // used for collision space and discretizing plain xyz into grid world 
    OccupancyGridUser::init(m_param_catalog.m_occupancy_grid_params,
                        m_param_catalog.m_robot_resolution_params);
    

    // used for discretization of robot movements
    ContArmState::setRobotResolutionParams(m_param_catalog.m_robot_resolution_params);

#ifdef USE_IKFAST_SOLVER
    ROS_DEBUG_NAMED(CONFIG_LOG, "Using IKFast");
#endif
#ifdef USE_KDL_SOLVER
    ROS_DEBUG_NAMED(CONFIG_LOG, "Using KDL");
#endif

    // Initialize the heuristics. The (optional) parameter defines the cost multiplier.
    //m_heur_mgr->initializeHeuristics();

    // used for arm kinematics
    LeftContArmState::initArmModel(m_param_catalog.m_left_arm_params);
    RightContArmState::initArmModel(m_param_catalog.m_right_arm_params);

    // collision space mgr needs arm models in order to do collision checking
    // have to do this funny thing  of initializing an object because of static
    // variable + inheritance (see ContArmState for details)
    LeftContArmState l_arm;
    RightContArmState r_arm;
    m_cspace_mgr = make_shared<CollisionSpaceMgr>(r_arm.getArmModel(),
                                                  l_arm.getArmModel());
    m_heur_mgr->setCollisionSpaceMgr(m_cspace_mgr);
    // load up motion primitives
    m_mprims.loadMPrims(m_param_catalog.m_motion_primitive_params);

    // load up static pviz instance for visualizations. 
    Visualizer::createPVizInstance();
    Visualizer::setReferenceFrame(std::string("/map"));
}

// sets parameters for query specific things
void Environment::configureQuerySpecificParams(SearchRequestPtr search_request){
    // sets the location of the object in the frame of the wrist
    // have to do this funny thing  of initializing an object because of static
    // variable + inheritance (see ContArmState for details)
    LeftContArmState l_arm;
    RightContArmState r_arm;
    l_arm.setObjectOffset(search_request->m_params->left_arm_object);
    r_arm.setObjectOffset(search_request->m_params->right_arm_object);

    // this specifically sets which arm RobotState will generate IK solutions
    // for
    RobotState::setPlanningMode(search_request->m_params->planning_mode);
    m_mprims.loadMPrimSet(search_request->m_params->planning_mode);
}

/*! \brief Given the solution path containing state IDs, reconstruct the
 * actual corresponding robot states. This also makes the path smooth in between
 * each state id because we add in the intermediate states given by the
 * transition data.
 */
vector<FullBodyState> Environment::reconstructPath(vector<int> soln_path){
    PathPostProcessor postprocessor(m_hash_mgr, m_cspace_mgr);
    vector<FullBodyState> final_path = postprocessor.reconstructPath(soln_path, 
                                                                     *m_goal, 
                                                                     m_mprims.getMotionPrims(),
                                                                     m_edges);
    if(m_param_catalog.m_visualization_params.final_path){
        ROS_INFO("visualizing final path");
        postprocessor.visualizeFinalPath(final_path);
    }
    return final_path;
}
