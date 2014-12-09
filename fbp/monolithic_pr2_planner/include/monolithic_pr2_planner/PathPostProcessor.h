#pragma once
#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitive.h>
#include <monolithic_pr2_planner/HashManager.h>
#include <monolithic_pr2_planner/CollisionSpaceMgr.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <vector>
#include <map>

namespace monolithic_pr2_planner {
    struct FullBodyState {
        std::vector<double> base;
        std::vector<double> left_arm;
        std::vector<double> right_arm;
        std::vector<double> obj;
    };
    typedef std::pair<int, int> Edge;
    class PathPostProcessor {
        public:
            PathPostProcessor(HashManagerPtr hash_mgr, CSpaceMgrPtr cspace_mgr);
            std::vector<FullBodyState> reconstructPath(std::vector<int> state_ids,
                                                       GoalState& goal_state,
                                                       std::vector<MotionPrimitivePtr> mprims,
                                                       std::map<Edge, MotionPrimitivePtr>& cache);
            static void visualizeFinalPath(std::vector<FullBodyState> path);
            bool stateInterpolate(const RobotState& start, const RobotState& end,
                                             std::vector<FullBodyState>* interp_steps);
        private:
            std::vector<FullBodyState> getFinalPath(const vector<int>& state_ids,
                                            const vector<TransitionData>& transition_states,
                                            GoalState& goal_state);
            std::vector<FullBodyState> shortcutPath(const vector<int>& state_ids, const vector<TransitionData>& transition_states,
                                            GoalState& goal_state);
            FullBodyState createFBState(const RobotState& robot);
            RobotState createRobotState(const FullBodyState& fb_state);
            ContBaseState createContBaseState(const FullBodyState& state);
            CSpaceMgrPtr m_cspace_mgr;
            HashManagerPtr m_hash_mgr;
    };
}
