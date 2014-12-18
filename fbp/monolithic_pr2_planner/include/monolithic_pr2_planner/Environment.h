#pragma once
#include <ros/ros.h>
#include <sbpl/headers.h>
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <monolithic_pr2_planner/CollisionSpaceMgr.h>
#include <monolithic_pr2_planner/HashManager.h>
#include <monolithic_pr2_planner/SearchRequest.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitivesMgr.h>
#include <monolithic_pr2_planner/Heuristics/HeuristicMgr.h>
#include <monolithic_pr2_planner/PathPostProcessor.h>
#include <boost/functional/hash.hpp>
#include <stdexcept>
#include <vector>
#include <memory>
#include <map>

namespace monolithic_pr2_planner {
    /*! \brief Implements a complete environment used by the SBPL planner.
     * Contains everything from managing state IDs to collision space
     * information.
     */
    typedef std::pair<int, int> Edge;
    class Environment : public DiscreteSpaceInformation {
        public:
            Environment(ros::NodeHandle nh);
            CSpaceMgrPtr getCollisionSpace(){ return m_cspace_mgr; };
            HeuristicMgrPtr getHeuristicMgr(){ return m_heur_mgr; };
            bool configureRequest(SearchRequestParamsPtr search_request_params,
                                  int& start_id, int& goal_id);
            std::vector<FullBodyState> reconstructPath(std::vector<int> state_ids);
            inline void setCollisionSpace(CSpaceMgrPtr cspace_mgr){
                m_cspace_mgr = cspace_mgr;
            }

            virtual void GetSuccs(int sourceStateID, vector<int>* succIDs, vector<int>* costs);
            virtual void GetLazySuccsWithUniqueIds(int g_id, int sourceStateID, vector<int>* succIDs,
                          vector<int>* costs, std::vector<bool>* isTrueCost);
            virtual int GetTrueCost(int parentID, int childID);
            void reset();

        protected:
            bool setStartGoal(SearchRequestPtr search_request,
                              int& start_id, int& goal_id);
            bool setCompleteStartGoal(SearchRequestPtr search_request,
                                       int& start_id, int& goal_id);
            int saveFakeGoalState(const GraphStatePtr& graph_state);
            void configurePlanningDomain();
            void configureQuerySpecificParams(SearchRequestPtr search_request);

            ParameterCatalog m_param_catalog;
            CSpaceMgrPtr m_cspace_mgr;
            HashManagerPtr m_hash_mgr;
            ros::NodeHandle m_nodehandle;
            GoalStatePtr m_goal;
            GraphStatePtr m_complete_goal;
            MotionPrimitivesMgr m_mprims;
            HeuristicMgrPtr m_heur_mgr;
            std::map<Edge, MotionPrimitivePtr> m_edges;

            bool m_using_lazy;

        // SBPL interface stuff
        public:
            bool InitializeEnv(const char* sEnvFile){return false;};
            bool InitializeMDPCfg(MDPConfig *MDPCfg){ return true; };
            int  GetFromToHeuristic(int FromStateID, int ToStateID){ throw std::runtime_error("unimplement");  };
            int  GetGoalHeuristic(int stateID);
            int  GetGoalHeuristic(int stateID, int goal_id);
            int  GetStartHeuristic(int stateID) { throw std::runtime_error("unimplement"); };
            void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV){};
            void SetAllActionsandAllOutcomes(CMDPSTATE* state){};
            void SetAllPreds(CMDPSTATE* state){};
            int  SizeofCreatedEnv(){ return m_hash_mgr->size(); };
            void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL){};
            void PrintEnv_Config(FILE* fOut){};

    };
}
