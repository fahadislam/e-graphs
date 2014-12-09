#include <monolithic_pr2_planner/MotionPrimitives/FullBodyAdaptiveMotionPrimitive.h>
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <monolithic_pr2_planner/Visualizer.h>
#include <cmath>

using namespace monolithic_pr2_planner;
using namespace std;

GraphState* FullBodyAdaptiveMotionPrimitive::m_goal;

// TODO refactor this
bool FullBodyAdaptiveMotionPrimitive::apply(const GraphState& source_state,
                                  GraphStatePtr& successor,
                                  TransitionData& t_data){
    DiscObjectState goal = m_goal->getObjectStateRelMap();
    // TODO parameterize this distance?
    if (dist(source_state.getObjectStateRelMap(), goal) > 10){
        return false;
    }

    successor = boost::make_shared<GraphState>(*m_goal);

    ROS_ERROR("Full Body AMP!");
    vector<int> c = successor->getCoords();
    for(unsigned int i=0; i<c.size(); i++)
      printf("%d ",c[i]);
    printf("\n");
    //std::cin.get();


    t_data.motion_type(motion_type());
    // TODO compute real cost
    t_data.cost(cost());
    computeIntermSteps(source_state, *successor, t_data);

    return true;
}


void FullBodyAdaptiveMotionPrimitive::computeIntermSteps(const GraphState& source_state, 
                        const GraphState& successor, 
                        TransitionData& t_data){
    std::vector<RobotState> interp_steps;
    RobotState::workspaceInterpolate(source_state.robot_pose(), 
                                     successor.robot_pose(),
                                     &interp_steps);

    ROS_DEBUG_NAMED(MPRIM_LOG, "interpolation for arm AMP");
    for (auto robot_state: interp_steps){
        robot_state.printToDebug(MPRIM_LOG);
    }
    t_data.interm_robot_steps(interp_steps);
    // fill in the cont base steps to be the same throughout; this is an arm
    // only motion
    ContBaseState c_base = source_state.robot_pose().base_state();
    std::vector<ContBaseState> cont_base_states(interp_steps.size(), c_base);
    t_data.cont_base_interm_steps(cont_base_states);

}

void FullBodyAdaptiveMotionPrimitive::print() const {
    ROS_DEBUG_NAMED(MPRIM_LOG, 
                    "FullBodyAdaptiveMotionPrimitive cost %d", cost());
}

void FullBodyAdaptiveMotionPrimitive::computeCost(const MotionPrimitiveParams& params){
    m_cost = 1;
}
