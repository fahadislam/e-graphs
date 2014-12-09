#include <monolithic_pr2_planner/PathPostProcessor.h>
#include <monolithic_pr2_planner/Constants.h>
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/Visualizer.h>


using namespace monolithic_pr2_planner;
using namespace std;

PathPostProcessor::PathPostProcessor(HashManagerPtr hash_mgr, CSpaceMgrPtr cspace_mgr):
m_cspace_mgr(cspace_mgr), m_hash_mgr(hash_mgr)
{
}


/*! \brief Given the solution path containing state IDs, reconstruct the
 * actual corresponding robot states. This also makes the path smooth in between
 * each state id because we add in the intermediate states given by the
 * transition data.
 */
vector<FullBodyState> PathPostProcessor::reconstructPath(vector<int> soln_path,
                                                         GoalState& goal_state,
                                                         vector<MotionPrimitivePtr> mprims,
                                                         std::map<Edge, MotionPrimitivePtr>& cache){
    vector<TransitionData> transition_states;
    for (size_t i=0; i < soln_path.size()-1; i++){
        int start_id = soln_path[i];
        int end_id = soln_path[i+1];
        GraphStatePtr source_state = m_hash_mgr->getGraphState(start_id);
        GraphStatePtr successor = m_hash_mgr->getGraphState(end_id);
        TransitionData t_data;
        Edge edge(start_id, end_id);
        bool mprim_in_cache = (cache.find(edge) != cache.end());
        if (!mprim_in_cache){
            vector<RobotState> states;
            states.push_back(source_state->robot_pose());
            states.push_back(successor->robot_pose());
            t_data.interm_robot_steps(states);
            //ROS_WARN("couldn't find transition in final path. Going to assume it's a snap!");
        } else {
            MotionPrimitivePtr mprim = cache.at(edge);
            if (!mprim->apply(*source_state, successor, t_data)){
                ROS_ERROR("couldn't apply mprim during reconstruction?? "
                          "this should be possible because it's part of the solution path");
                assert(false);
            }
        }
        transition_states.push_back(t_data);
    }
    bool use_shortcut = false;

    if (!use_shortcut){
        vector<FullBodyState> unshortcut_path = getFinalPath(soln_path, 
                                                             transition_states,
                                                             goal_state);
        return unshortcut_path;
    } else {
        std::vector<FullBodyState> final_path = shortcutPath(soln_path,
            transition_states, goal_state);
        return final_path;
    }
}

std::vector<FullBodyState> PathPostProcessor::shortcutPath(const vector<int>&
    state_ids, const vector<TransitionData>& transition_states, GoalState& goal_state){
    ROS_DEBUG_NAMED(HEUR_LOG, "Original request : States : %lu, transition data : %lu",
        state_ids.size(), transition_states.size());
    std::vector<FullBodyState> final_path;
    size_t i = 0;
    size_t j = 1;
    // Shove in the first full body state.
    // final_path.push_back(path[0]);
    
    { // throw in the first point
        GraphStatePtr source_state = m_hash_mgr->getGraphState(state_ids[0]);
        final_path.push_back(createFBState(source_state->robot_pose()));
    }

    std::vector<FullBodyState> interp_states;
    while(j < state_ids.size()){
        assert(i<j);
        ROS_DEBUG_NAMED(SEARCH_LOG, "Shortcutting : %lu %lu; size of final_path %lu", i, j,
            final_path.size());
        GraphStatePtr source_state = m_hash_mgr->getGraphState(state_ids[i]);
        GraphStatePtr end_state = m_hash_mgr->getGraphState(state_ids[j]);
        RobotState first_pose = source_state->robot_pose();
        RobotState second_pose = end_state->robot_pose();
        std::vector<FullBodyState> interp_states_prev(interp_states.begin(),
            interp_states.end());
        interp_states.clear();
        bool interpolate = stateInterpolate(first_pose, second_pose, &interp_states);
        ROS_DEBUG_NAMED(SEARCH_LOG, "Interpolated states size: %d; Interp prev size: %d",
            static_cast<int>(interp_states.size()),
            static_cast<int>(interp_states_prev.size()));
        if(interpolate) {
            bool no_collision = true;
            for (size_t k = 0; k < interp_states.size(); ++k)
            {
                if(!m_cspace_mgr->isValidContState(interp_states[k].left_arm,interp_states[k].right_arm,
                    interp_states[k].base)) {
                    no_collision = false;
                }
            }
            if(no_collision){
                j++;
            }
            else{
                ROS_DEBUG_NAMED(SEARCH_LOG, "Collision here; %lu %lu", i, j);
                // Check if it is a consecutive state
                if(i == (j - 1)){
                    ROS_DEBUG_NAMED(SEARCH_LOG, "Already, i (%lu) is j (%lu) - 1",
                        i, j);
                    int motion_type = transition_states[i].motion_type();
                    bool isInterpBaseMotion = (motion_type == MPrim_Types::BASE || 
                                               motion_type == MPrim_Types::BASE_ADAPTIVE);
                    for (size_t t=0; t < transition_states[i].interm_robot_steps().size(); t++){
                        RobotState robot = transition_states[i].interm_robot_steps()[t];
                        FullBodyState state = createFBState(robot);
                        if(isInterpBaseMotion){
                            ContBaseState cont_base = transition_states[i].interm_robot_steps()[t].base_state();
                            std::vector<double> base = cont_base.getCoords();
                            state.base = base;
                        }
                        final_path.push_back(state);
                    }
                    ++i;
                    ++j;
                } else {
                    // Shove things in.
                    // [) format. We'll shove the last point in the end or in the
                    // next cycle
                    final_path.insert(final_path.end(), interp_states_prev.begin(),
                        interp_states_prev.end() - 1);
                    i = j - 1;
                    // first_pose.visualize();
                    // second_pose.visualize();
                    // std::cin.get();
                }
            }
        }
        else{
            ROS_DEBUG_NAMED(SEARCH_LOG ,"The interpolation function failed for %lu %lu; Using transition data instead.", i, j);
            // Insert till whatever worked till now.
            if( i != j -1){
                final_path.insert(final_path.end(), interp_states_prev.begin(),
                            interp_states_prev.end() - 1);
                i = j - 1;
            } else {
                // Move i and j forward.
                int motion_type = transition_states[i].motion_type();
                bool isInterpBaseMotion = (motion_type == MPrim_Types::BASE || 
                                           motion_type == MPrim_Types::BASE_ADAPTIVE);
                for (size_t t=0; t < transition_states[i].interm_robot_steps().size(); t++){
                    RobotState robot = transition_states[i].interm_robot_steps()[t];
                    FullBodyState state = createFBState(robot);
                    if(isInterpBaseMotion){
                        ContBaseState cont_base = transition_states[i].interm_robot_steps()[t].base_state();
                        std::vector<double> base = cont_base.getCoords();
                        state.base = base;
                    }
                    final_path.push_back(state);
                }
                ++i;
                ++j;
            }
        }
    }
    if(i != state_ids.size() - 1){
        final_path.insert(final_path.end(), interp_states.begin(),
            interp_states.end());
    }
    ROS_DEBUG_NAMED(HEUR_LOG, "Size of shortcutPath: %d",
        static_cast<int>(final_path.size()));
    { // throw in the last point
        GraphStatePtr soln_state = goal_state.getSolnState();
        final_path.push_back(createFBState(soln_state->robot_pose()));
    }
    return final_path;
}
void PathPostProcessor::visualizeFinalPath(vector<FullBodyState> path){
    for (auto& state : path){
        vector<double> l_arm, r_arm, base;
        l_arm = state.left_arm;
        r_arm = state.right_arm;
        base = state.base;
        BodyPose bp;
        bp.x = base[0];
        bp.y = base[1];
        bp.z = base[2];
        bp.theta = base[3];
        Visualizer::pviz->visualizeRobot(r_arm, l_arm, bp, 150, "robot", 0);
        usleep(5000);
        // std::cin.get();
    }
}

/*! \brief Given a start and end state id, find the motion primitive with the
 * least cost that gets us from start to end. Return that information with a
 * TransitionData object.
 */

FullBodyState PathPostProcessor::createFBState(const RobotState& robot){
    vector<double> l_arm, r_arm, base;
    vector<double> obj(6,0);
    robot.right_arm().getAngles(&r_arm);
    robot.left_arm().getAngles(&l_arm);
    ContBaseState c_base = robot.base_state();
    base = c_base.getCoords();
    FullBodyState state;
    state.left_arm = l_arm;
    state.right_arm = r_arm;
    state.base = base;
    ContObjectState obj_state = robot.getObjectStateRelMap();
    obj[0] = obj_state.x();
    obj[1] = obj_state.y();
    obj[2] = obj_state.z();
    obj[3] = obj_state.roll();
    obj[4] = obj_state.pitch();
    obj[5] = obj_state.yaw();
    state.obj = obj;
    return state;
}

RobotState PathPostProcessor::createRobotState(const FullBodyState& fb_state){
    ContBaseState cbase_state = createContBaseState(fb_state);
    LeftContArmState l_arm(fb_state.left_arm);
    RightContArmState r_arm(fb_state.right_arm);
    RobotState state(cbase_state, r_arm, l_arm);
    return state;
}

/*! \brief Retrieve the final path, with intermediate states and all.  There's
 * one more state_id than transition state.  */
std::vector<FullBodyState> PathPostProcessor::getFinalPath(const vector<int>& state_ids,
                                 const vector<TransitionData>& transition_states,
                                 GoalState& goal_state){
    vector<FullBodyState> fb_states;
    for (size_t i=0; i < transition_states.size(); i++){
        // throw in the first point
        GraphStatePtr source_state = m_hash_mgr->getGraphState(state_ids[i]);
        fb_states.push_back(createFBState(source_state->robot_pose()));
        
        int motion_type = transition_states[i].motion_type();
        bool isInterpBaseMotion = (motion_type == MPrim_Types::BASE || 
                                   motion_type == MPrim_Types::BASE_ADAPTIVE);
        for (size_t j=0; j < transition_states[i].interm_robot_steps().size(); j++){
            RobotState robot = transition_states[i].interm_robot_steps()[j];
            FullBodyState state = createFBState(robot);
            if (isInterpBaseMotion){
                ContBaseState cont_base = transition_states[i].interm_robot_steps()[j].base_state();
                std::vector<double> base = cont_base.getCoords();
                state.base = base;
            } 
            fb_states.push_back(state);
        }
    }
    return fb_states;
}


/*! \brief Given two robot states, we interpolate all steps in between them. The
 * delta step size is based on the RPY resolution of the object pose and the XYZ
 * resolution of the base. This returns a vector of RobotStates, which are
 * discretized to the grid (meaning if the arms move a lot, but the base barely
 * moves, the base discrete values will likely stay the same). This determines
 * the number of interpolation steps based on which part of the robot moves more
 * (arms or base).
 * TODO need to test this a lot more
 */
bool PathPostProcessor::stateInterpolate(const RobotState& start, const RobotState& end, vector<FullBodyState>* interp_steps){

    // Need this relbody - not relmap
    ContObjectState start_obj = start.getObjectStateRelBody();
    ContObjectState end_obj = end.getObjectStateRelBody();

    ContBaseState start_base = start.base_state();
    ContBaseState end_base = end.base_state();

    int num_interp_steps = RobotState::numInterpSteps(start, end);
    vector<ContObjectState> interp_obj_steps;
    vector<ContBaseState> interp_base_steps;
    //ROS_DEBUG_NAMED(MPRIM_LOG, "start obj");
    //start_obj.printToDebug(MPRIM_LOG);
    //ROS_DEBUG_NAMED(MPRIM_LOG, "end obj");
    //end_obj.printToDebug(MPRIM_LOG);
    interp_obj_steps = ContObjectState::interpolate(start_obj, end_obj, 
                                                    num_interp_steps);
    interp_base_steps = ContBaseState::interpolate(start_base, end_base, 
                                                   num_interp_steps);
    assert(interp_obj_steps.size() == interp_base_steps.size());
    ROS_DEBUG_NAMED(MPRIM_LOG, "size of returned interp %lu", interp_obj_steps.size());

    // should at least return the same start and end poses
    if (num_interp_steps < 2){
        assert(interp_obj_steps.size() == 2);
    } else {
        assert(interp_obj_steps.size() == static_cast<size_t>(num_interp_steps));
    }

    // Returns all the full body states in between. It's continuous.
    for (size_t i=0; i < interp_obj_steps.size(); i++){
        //interp_obj_steps[i].printToDebug(MPRIM_LOG);
        RobotState seed(interp_base_steps[i], start.right_arm(), start.left_arm());
        RobotPosePtr new_robot_state;
        if (!RobotState::computeRobotPose(interp_obj_steps[i], seed, new_robot_state)){
            return false;
        }
        FullBodyState fb_state = createFBState(*new_robot_state);
        std::vector<double> base = interp_base_steps[i].getCoords();
        fb_state.base = base;
        interp_steps->push_back(fb_state);
    }
    return true;
}

ContBaseState PathPostProcessor::createContBaseState(const FullBodyState& state){
    ContBaseState c_base_state;
    c_base_state.x(state.base[BodyDOF::X]);
    c_base_state.y(state.base[BodyDOF::Y]);
    c_base_state.z(state.base[BodyDOF::Z]);
    c_base_state.theta(state.base[BodyDOF::THETA]);
    return c_base_state;
}
