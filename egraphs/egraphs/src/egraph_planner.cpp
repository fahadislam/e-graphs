/*
 * Copyright (c) 2013, Mike Phillips and Maxim Likhachev
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Mellon University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <egraphs/egraph_planner.h>
#include <algorithm>
#include <numeric>
#include <stdlib.h>
#include <tf/tf.h>

using namespace std;

template <typename HeuristicType>
LazyAEGPlanner<HeuristicType>::LazyAEGPlanner(DiscreteSpaceInformation* environment, bool bSearchForward,
                               EGraphManagerPtr egraph_mgr, int num_isls, EGraphVisualizer* egraph_vis) :
  params(0.0), egraph_mgr_(egraph_mgr){ //, goal_state(NULL) {
  //bforwardsearch = bSearchForward;
  if(!bSearchForward)
    ROS_WARN("backward search not supported. setting to run forward search.");
  bforwardsearch = true;
  environment_ = environment;
  replan_number = 0;

  num_islands = num_isls;
  heaps.resize(num_islands);
  incons.resize(num_islands);
  states.resize(num_islands);  //fadi
  goal_state.resize(num_islands);
  island_state_id.resize(num_islands);
  island_state.resize(num_islands);
  egraph_vis_ = egraph_vis;

  //goal_state_id = -1;
  start_state_id = -1;
  evaluated_snaps = 0;

  ros::NodeHandle nh;
  marker_pub_expansion = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);  //fadi
}

template <typename HeuristicType>
LazyAEGState* LazyAEGPlanner<HeuristicType>::GetState(int g_id, int id){  //fadi
  //if this stateID is out of bounds of our state vector then grow the list
  if(id >= int(states[g_id].size())){
    for(int i=states[g_id].size(); i<=id; i++)
      states[g_id].push_back(NULL);
  }
  //if we have never seen this state then create one
  if(states[g_id][id]==NULL){
    states[g_id][id] = new LazyAEGState();
    states[g_id][id]->id = id;
    states[g_id][id]->replan_number = -1;
  }
  //initialize the state if it hasn't been for this call to replan
  LazyAEGState* s = states[g_id][id];
  if(s->replan_number != replan_number){
    s->g = INFINITECOST;
    s->v = INFINITECOST;
    s->iteration_closed = -1;
    s->replan_number = replan_number;
    s->best_parent = NULL;
    s->expanded_best_parent = NULL;
    s->best_edge_type = EdgeType::NONE;
    s->expanded_best_edge_type = EdgeType::NONE;
    s->snap_midpoint = -1;
    s->expanded_snap_midpoint = -1;
    s->heapindex = 0;
    s->in_incons = false;
    s->isTrueCost = true;
    //clear the lazy list
    while(!s->lazyList.empty())
      s->lazyList.pop();

    //compute heuristics
    if(bforwardsearch){
      clock_t h_t0 = clock();
      s->h = egraph_mgr_->getHeuristic(s->id);
      clock_t h_t1 = clock();
      heuristicClock += h_t1-h_t0;
    } else {
      printf("backwards search not implemented!");
      assert(false);
      s->h = environment_->GetStartHeuristic(s->id);
    }
  }
  return s;
}

template <typename HeuristicType>
void LazyAEGPlanner<HeuristicType>::ExpandState(int g_id, LazyAEGState* parent){
  bool print = false; //parent->id == 285566;
  if(print)
    printf("expand %d\n",parent->id);
  vector<int> children;
  vector<int> costs;
  vector<bool> isTrueCost;

  //visualizing state
  vector<double> coord(3);
  egraph_mgr_->egraph_env_->getCoord(parent->id, coord);

  // printf("angle %f\n", coord[2]);
  tf::Quaternion quat_state(0,0,coord[2]);

  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.id = rand()%10000;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.pose.position.x = coord[0];
  marker.pose.position.y = coord[1];
  marker.pose.position.z = 0;
  marker.pose.orientation.x = quat_state.x();
  marker.pose.orientation.y = quat_state.y();
  marker.pose.orientation.z = quat_state.z();
  marker.pose.orientation.w = quat_state.w();
  marker.scale.x = 0.15;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;
  if (g_id == 0){ //todo
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  }
  else if (g_id == 1){
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  }
  else if (g_id == 2){
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  }


  marker.color.a = 1.0;

  ma.markers.push_back(marker);
  // visualization_msgs::MarkerArray ma;
  // ma.markers.push_back(marker);
  marker_pub_expansion.publish(ma);
  getchar();



  clock_t getSucc_t0 = clock();
  if(bforwardsearch)
    environment_->GetLazySuccsWithUniqueIds(parent->id, &children, &costs, &isTrueCost);
  else
    environment_->GetLazyPredsWithUniqueIds(parent->id, &children, &costs, &isTrueCost);
  clock_t getSucc_t1 = clock();
  succsClock += getSucc_t1-getSucc_t0;

  vector<EdgeType> edgeTypes(children.size(),EdgeType::NORMAL);

  vector<int> snap_midpoints(children.size(),-1);
  if (params.use_egraph){
    //egraph_mgr_->clearSnapSuccessorsCache();

    //egraph_mgr_->getSnapSuccessors(parent->id, &children, &costs, &isTrueCost, &edgeTypes);
    clock_t shortcut_t0 = clock();
    egraph_mgr_->getDirectShortcutSuccessors(parent->id, &children, &costs, &isTrueCost, &edgeTypes);

    snap_midpoints.resize(children.size(),-1);
    egraph_mgr_->getSnapShortcuts(parent->id, &children, &costs, &isTrueCost, &edgeTypes, &snap_midpoints);
    if(edgeTypes.size()>0 && edgeTypes.back()==EdgeType::SNAP_DIRECT_SHORTCUT)
      assert(snap_midpoints.back()>=0);
    clock_t shortcut_t1 = clock();
    shortcutClock += shortcut_t1 - shortcut_t0;

    //egraph_mgr_->getComboSnapShortcutSuccessors(parent->id, &children, &costs, &isTrueCost, &edgeTypes);
    // getComboSnapShortcutSuccessors needs the output of getSnapSuccessors, so
    // getSnapSuccessors caches its output inside egraph_mgr_. we must make sure
    // to clean it up before the next iteration
    //egraph_mgr_->clearSnapSuccessorsCache();
  }

  // make sure our costs are nonzero
  for (auto& cost : costs){
      assert(cost > 0);
  }
  if (print){
      for (auto& id : children){
          ROS_INFO("%d", id);
      }
  }

  //iterate through children of the parent
  for(int i=0; i<(int)children.size(); i++){
    //printf("  succ %d\n",children[i]);
    LazyAEGState* child = GetState(g_id,children[i]);  //fadi
    insertLazyList(g_id, child, parent, costs[i], isTrueCost[i], edgeTypes[i], snap_midpoints[i]);
  }
}

//assumptions:
//state is at the front of the open list
//it's minimum f-value is an underestimate (the edge cost from the parent is a guess and needs to be evaluated properly)
//it hasn't been expanded yet this iteration
template <typename HeuristicType>
void LazyAEGPlanner<HeuristicType>::EvaluateState(int g_id, LazyAEGState* state){
  bool print = false; //state->id == 285566;
  if(print)
    printf("evaluate %d (from %d)\n",state->id, state->best_parent->id);
  LazyAEGState* parent = state->best_parent;
  EdgeType edgeType = state->best_edge_type;
  int snap_midpoint = state->snap_midpoint;

  getNextLazyElement(g_id, state);
  //printf("state_ptr=%p\n",state);
  //printf("state_id=%d\n",state->id);
  //printf("parent_ptr=%p\n",parent);
  //printf("parent_id=%d\n",parent->id);

  /* Mike replacing victor's code
  Edge snap_edge(parent->id, state->id);
  int trueCost;
  if (egraph_mgr_->snaps_.find(snap_edge) != egraph_mgr_->snaps_.end()){
    trueCost = egraph_mgr_->getSnapTrueCost(parent->id, state->id);
  } else {
    trueCost = environment_->GetTrueCost(parent->id, state->id);
  }
  */

  int trueCost;
  if(edgeType == EdgeType::SNAP)
    trueCost = egraph_mgr_->getSnapTrueCost(parent->id, state->id);
  else if(edgeType == EdgeType::NORMAL){
    clock_t getSucc_t0 = clock();
    trueCost = environment_->GetTrueCost(parent->id, state->id);
    clock_t getSucc_t1 = clock();
    succsClock += getSucc_t1-getSucc_t0;
  }
  else if(edgeType == EdgeType::SNAP_DIRECT_SHORTCUT){
    clock_t shortcut_t0 = clock();
    assert(snap_midpoint >= 0);
    trueCost = egraph_mgr_->getSnapShortcutTrueCost(parent->id, snap_midpoint, state->id);
    clock_t shortcut_t1 = clock();
    shortcutClock += shortcut_t1 - shortcut_t0;
  }
  else
    assert(false);

  if (print)
      printf("has a true cost of %d\n",trueCost);
  if(trueCost > 0) //if the evaluated true cost is valid (positive), insert it into the lazy list
    insertLazyList(g_id, state,parent,trueCost,true,edgeType,snap_midpoint);
}

//this should only be used with EvaluateState since it is assuming state hasn't been expanded yet (only evaluated)
template <typename HeuristicType>
void LazyAEGPlanner<HeuristicType>::getNextLazyElement(int g_id, LazyAEGState* state){
  if(state->lazyList.empty()){
    state->g = INFINITECOST;
    state->best_parent = NULL;
    state->best_edge_type = EdgeType::NONE;
    state->snap_midpoint = -1;
    state->isTrueCost = true;
    return;
  }
  LazyAEGListElement elem = state->lazyList.top();
  state->lazyList.pop();
  state->g = elem.parent->v + elem.edgeCost;
  state->best_parent = elem.parent;
  state->best_edge_type = elem.edgeType;
  state->snap_midpoint = elem.snap_midpoint;
  state->isTrueCost = elem.isTrueCost;
  //the new value is cheapest and if the value is also true then we want to throw out all the other options
  if(state->isTrueCost){
    while(!state->lazyList.empty())
      state->lazyList.pop();
  }
  putStateInHeap(g_id, state);
}

template <typename HeuristicType>
void LazyAEGPlanner<HeuristicType>::insertLazyList(int g_id, LazyAEGState* state, LazyAEGState* parent, int edgeCost, bool isTrueCost, EdgeType edgeType, int snap_midpoint){
  bool print = false; //state->id == 285566 || parent->id == 285566;
  if(state->v <= parent->v + edgeCost)
    return;
  else if(state->g <= parent->v + edgeCost){
    //if the best g-value we have is true and better, then the value we had dominates this one and we don't need it
    if(state->isTrueCost)
      return;
    //insert this guy into the lazy list
    LazyAEGListElement elem(parent,edgeCost,isTrueCost,edgeType,snap_midpoint);
    state->lazyList.push(elem);
  }
  else{//the new guy is the cheapest so far
    //should we save what was the previous best?
    if(!isTrueCost && //the better guy's cost is not for sure
       //state->g < INFINITECOST && //we actually have a previous best (we actually don't need this line because of the next one)
       state->g < state->v){ //we're not saving something we already expanded (and is stored in v and expanded_best_parent)
      //we save it by putting it in the lazy list
      LazyAEGListElement elem(state->best_parent, state->g - state->best_parent->v, state->isTrueCost, state->best_edge_type, state->snap_midpoint);
      state->lazyList.push(elem);
      //printf("save the previous best\n");
    }

    //the new guy is the cheapest
    state->g = parent->v + edgeCost;
    state->best_parent = parent;
    state->best_edge_type = edgeType;
    state->snap_midpoint = snap_midpoint;
    state->isTrueCost = isTrueCost;

    //the new value is cheapest and if the value is also true then we want to throw out all the other options
    if(isTrueCost){
      //printf("clear the lazy list\n");
      while(!state->lazyList.empty())
        state->lazyList.pop();
    }

    //this function puts the state into the heap (or updates the position) if we haven't expanded
    //if we have expanded, it will put the state in the incons list (if we haven't already)
    putStateInHeap(g_id, state);
  }
  if(print)
    printf("state->id=%d state->g=%d state->h=%d, parent->v=%d edgeCost=%d isTrueCost=%d\n",state->id,state->g,state->h,parent->v,edgeCost,isTrueCost);
}

template <typename HeuristicType>
void LazyAEGPlanner<HeuristicType>::putStateInHeap(int g_id, LazyAEGState* state){
  updateGoal(g_id, state);
  bool print = false; //state->id == 285566;
  //we only allow one expansion per search iteration
  //so insert into heap if not closed yet
  if(state->iteration_closed != search_iteration){
    CKey key;
    key.key[0] = state->g + int(eps * state->h);
    if(print)
      printf("put state in open with f %lu\n", key.key[0]);
    //if the state is already in the heap, just update its priority
    if(state->heapindex != 0)
      heaps[g_id].updateheap(state,key);
    else //otherwise add it to the heap
      heaps[g_id].insertheap(state,key);
  }
  //if the state has already been expanded once for this iteration
  //then add it to the incons list so we can keep track of states
  //that we know we have better costs for
  else if(!state->in_incons){
    if(print)
      printf("put state in incons\n");
    incons[g_id].push_back(state);
    state->in_incons = true;
  }
}

template <typename HeuristicType>
void LazyAEGPlanner<HeuristicType>::updateGoal(int g_id, LazyAEGState* state){
  if(egraph_mgr_->egraph_env_->isGoal(state->id) && state->isTrueCost && state->g < goal_state[g_id].g){
    //ROS_INFO("updating the goal state");
    goal_state[g_id].id = state->id;
    goal_state[g_id].g = state->g;
    goal_state[g_id].best_parent = state->best_parent;
    goal_state[g_id].best_edge_type = state->best_edge_type;
    goal_state[g_id].snap_midpoint = state->snap_midpoint;
    goal_state[g_id].isTrueCost = true;
  }
}

//returns 1 if the solution is found, 0 if the solution does not exist and 2 if it ran out of time
template <typename HeuristicType>
int LazyAEGPlanner<HeuristicType>::ImprovePath(){

  //expand states until done
  int expands = 0;
  vector<CKey> min_key(num_islands);
  vector<bool> search_reached(num_islands, false);
  for (int g_id=0; g_id < num_islands; g_id++)
    min_key[g_id] = heaps[g_id].getminkeyheap();  //fadi


  while((!heaps[0].emptyheap() &&    //todo
        min_key[0].key[0] < INFINITECOST &&
        (goal_state[0].g > min_key[0].key[0] || !goal_state[0].isTrueCost) &&
        !outOfTime())
        ||
        (!heaps[1].emptyheap() &&    //todo
        min_key[1].key[0] < INFINITECOST &&
        (goal_state[1].g > min_key[1].key[0] || !goal_state[1].isTrueCost) &&
        !outOfTime())
        ||
        (!heaps[2].emptyheap() &&    //todo
        min_key[2].key[0] < INFINITECOST &&
        (goal_state[2].g > min_key[2].key[0] || !goal_state[2].isTrueCost) &&
        !outOfTime())){


    //get the state

    for (int g_id=0; g_id < num_islands; g_id++){

      if (search_reached[g_id] == true)  //todo
        continue;

      if (!(!heaps[g_id].emptyheap() &&    //todo
        min_key[g_id].key[0] < INFINITECOST &&
        (goal_state[g_id].g > min_key[g_id].key[0] || !goal_state[g_id].isTrueCost) &&
        !outOfTime())){
        printf("SEARCH %d FOUND PATH\n", g_id);
        search_reached[g_id] = true;
        for (int i=0; i < num_islands; i++)  //fadi
          if(goal_state[i].g < goal_state[i].v){
            goal_state[i].expanded_best_parent = goal_state[i].best_parent;
            goal_state[i].expanded_best_edge_type = goal_state[i].best_edge_type;
            goal_state[i].expanded_snap_midpoint = goal_state[i].snap_midpoint;
            goal_state[i].v = goal_state[i].g;
          }
        int PathCost;
        vector<vector<int>> pathIds;
        pathIds = GetSearchPath(g_id, PathCost);
        egraph_vis_->visualize();
        egraph_mgr_->setGoal();
        continue;
      }


      LazyAEGState* state = (LazyAEGState*)heaps[g_id].deleteminheap();

      if(state->v == state->g){
        printf("ERROR: consistent state is being expanded\n");
        printf("id=%d v=%d g=%d isTrueCost=%d lazyListSize=%lu\n",
                state->id,state->v,state->g,state->isTrueCost,state->lazyList.size());
        std::cin.get();
      }

      if(state->isTrueCost){
        if (state->best_parent){
            /* Mike replacing victor's code
            Edge edge(state->best_parent->id, state->id);
            if (egraph_mgr_->snaps_.find(edge) != egraph_mgr_->snaps_.end()){
              evaluated_snaps++;
            }
            */
            if(state->best_edge_type == EdgeType::SNAP)
              evaluated_snaps++;
        }

        //mark the state as expanded
        state->v = state->g;
        state->expanded_best_parent = state->best_parent;
        state->expanded_best_edge_type = state->best_edge_type;
        state->expanded_snap_midpoint = state->snap_midpoint;
        state->iteration_closed = search_iteration;
        //expand the state
        expands++;
        ExpandState(g_id, state);
        if(expands%10000 == 0)
          printf("expands so far=%u\n", expands);
      }
      else //otherwise the state needs to be evaluated for its true cost
        EvaluateState(g_id, state);

      //get the min key for the next iteration
      min_key[g_id] = heaps[g_id].getminkeyheap(); //todo
    }
  }

  search_expands += expands;

  // todo
  if(goal_state[0].g == INFINITECOST && (heaps[0].emptyheap() || min_key[0].key[0] >= INFINITECOST))
    return 0;//solution does not exists
  if(!heaps[0].emptyheap() && goal_state[0].g > min_key[0].key[0])
    return 2; //search exited because it ran out of time
  printf("search exited with a solution for eps=%.2f\n", eps*params.epsE);
  for (int g_id=0; g_id < num_islands; g_id++)  //fadi
    if(goal_state[g_id].g < goal_state[g_id].v){
      goal_state[g_id].expanded_best_parent = goal_state[g_id].best_parent;
      goal_state[g_id].expanded_best_edge_type = goal_state[g_id].best_edge_type;
      goal_state[g_id].expanded_snap_midpoint = goal_state[g_id].snap_midpoint;
      goal_state[g_id].v = goal_state[g_id].g;
    }

  while(1);
  return 1;
}

template <typename HeuristicType>
bool LazyAEGPlanner<HeuristicType>::reconstructSuccs(LazyAEGState* state,
                                               LazyAEGState*& next_state,
                                               vector<int>* wholePathIds,
                                               vector<int>* costs){
    //ROS_INFO("reconstruct with standard edge start");
    vector<int> SuccIDV;
    vector<int> CostV;
    vector<bool> isTrueCost;
    if(bforwardsearch)
        environment_->GetLazySuccsWithUniqueIds(state->expanded_best_parent->id, &SuccIDV, &CostV, &isTrueCost);
    else
        environment_->GetLazyPredsWithUniqueIds(state->expanded_best_parent->id, &SuccIDV, &CostV, &isTrueCost);
    int actioncost = INFINITECOST;
    //ROS_INFO("reconstruct with standard edge %d\n",state->expanded_best_parent->id);
    for(unsigned int i=0; i<SuccIDV.size(); i++){
        //printf("  succ %d\n",SuccIDV[i]);
        if(SuccIDV[i] == state->id && CostV[i]<actioncost)
            actioncost = CostV[i];
    }
    if(actioncost == INFINITECOST){
        return false;
    } else {
        // remember we're starting from the goal and working backwards, so we
        // want to store the parents
        //ROS_INFO("good...");
        costs->push_back(actioncost);
        next_state = state->expanded_best_parent;
        wholePathIds->push_back(next_state->id);
        return true;
    }
}

template <typename HeuristicType>
vector<vector<int>> LazyAEGPlanner<HeuristicType>::GetSearchPath(int g_id, int& solcost){
    clock_t reconstruct_t0 = clock();

    bool print = false;
    vector<vector<int>> wholePathIds(num_islands);
    vector<vector<int>> costs(num_islands);
    LazyAEGState* state;
    LazyAEGState* final_state;

    // for (int g_id=0;g_id<num_islands;g_id++){
      if(bforwardsearch){
            state = &goal_state[g_id];
            final_state = island_state[g_id];    //fadi
        } else {
            state = island_state[g_id];
            final_state = &goal_state[g_id];
        }

        wholePathIds[g_id].push_back(state->id);
        solcost = 0;
        int shortcut_count = 0;

        while(state->id != final_state->id){
            if(state->expanded_best_parent == NULL){
                printf("a state along the path has no parent!\n");
                assert(false);
            }
            if(state->v == INFINITECOST){
                printf("a state along the path has an infinite g-value!\n");
                printf("inf state = %d\n",state->id);
                assert(false);
            }

            LazyAEGState* next_state;
            if(state->expanded_best_edge_type == EdgeType::SNAP){
              assert(egraph_mgr_->reconstructSnap(state, next_state, &wholePathIds[g_id], &costs[g_id]));
              if(print)
                ROS_INFO("snap edge %d %d %d", costs[g_id].back(), state->id, wholePathIds[g_id].back());
            }
            else if(state->expanded_best_edge_type == EdgeType::NORMAL){
              bool ret = reconstructSuccs(state, next_state, &wholePathIds[g_id], &costs[g_id]);
              assert(ret);
              if(print)
                ROS_INFO("normal edge %d %d %d", costs[g_id].back(), state->id, wholePathIds[g_id].back());
            }
            else if(state->expanded_best_edge_type == EdgeType::DIRECT_SHORTCUT){
              int sc_cost;
              assert(egraph_mgr_->reconstructDirectShortcuts(state, next_state, &wholePathIds[g_id], &costs[g_id], shortcut_count, sc_cost));
              if(print)
                ROS_INFO("shortcut edge %d %d %d", sc_cost, state->id, wholePathIds[g_id].back());
            }
            else if(state->expanded_best_edge_type == EdgeType::SNAP_DIRECT_SHORTCUT){
              int totalCost;
              assert(egraph_mgr_->reconstructSnapShortcut(state, next_state, &wholePathIds[g_id], &costs[g_id], totalCost));
              if(print)
                ROS_INFO("snap shortcut edge %d %d %d", totalCost, state->id, wholePathIds[g_id].back());
            }
            else
              assert(false);
            assert(next_state == state->expanded_best_parent);
            assert(wholePathIds[g_id].back() == state->expanded_best_parent->id);
            state = next_state;
        }
        //if we searched forward then the path reconstruction
        //worked backward from the goal, so we have to reverse the path
        if(bforwardsearch){
            std::reverse(wholePathIds[g_id].begin(), wholePathIds[g_id].end());
            std::reverse(costs[g_id].begin(), costs[g_id].end());
        }
        solcost = std::accumulate(costs[g_id].begin(), costs[g_id].end(), 0);

        clock_t reconstruct_t1 = clock();
        reconstructTime = double(reconstruct_t1 - reconstruct_t0)/CLOCKS_PER_SEC;

        /*
        // if we're using lazy evaluation, we always want to feedback the path
        // regardless if it's valid
        if (params.feedback_path || params.use_lazy_validation){
            egraph_mgr_->storeLastPath(wholePathIds, costs);
        }
        if (params.use_lazy_validation){
            egraph_mgr_->feedbackLastPath();
        }
        */

        clock_t feedback_t0 = clock();
        if (params.feedback_path){
            egraph_mgr_->storeLastPath(wholePathIds[g_id], costs[g_id]);
            egraph_mgr_->feedbackLastPath();
        }
        clock_t feedback_t1 = clock();
        feedbackPathTime = double(feedback_t1-feedback_t0)/CLOCKS_PER_SEC;
        //egraph_mgr_->printVector(wholePathIds);

    // }
    return wholePathIds;

}

template <typename HeuristicType>
bool LazyAEGPlanner<HeuristicType>::outOfTime(){
  //if the user has sent an interrupt signal we stop
  if(interruptFlag)
    return true;
  //if we are supposed to run until the first solution, then we are never out of time
  if(params.return_first_solution)
    return false;
  double time_used = double(clock() - TimeStarted)/CLOCKS_PER_SEC;
  if(time_used >= params.max_time)
    printf("out of max time\n");
  if(use_repair_time && eps_satisfied != INFINITECOST && time_used >= params.repair_time)
    printf("used all repair time...\n");
  //we are out of time if:
         //we used up the max time limit OR
         //we found some solution and used up the minimum time limit
  return time_used >= params.max_time ||
         (use_repair_time && eps_satisfied != INFINITECOST && time_used >= params.repair_time);
}

template <typename HeuristicType>
void LazyAEGPlanner<HeuristicType>::initializeSearch(){
  //it's a new search, so increment replan_number and reset the search_iteration
  replan_number++;
  search_iteration = 0;
  search_expands = 0;
  totalExpands = 0;
  succsClock = 0;
  shortcutClock = 0;
  heuristicClock = 0;
  reconstructTime = 0;
  feedbackPathTime = 0;

  //clear open list, incons list, and stats list

  for (int g_id = 0; g_id < num_islands; ++g_id)
  {
    heaps[g_id].makeemptyheap();
    incons[g_id].clear();
  }

  stats.clear();

  //initialize epsilon variable
  eps = params.initial_eps;
  eps_satisfied = INFINITECOST;
  //call get state to initialize the start and goal states
  //double t0 = ros::Time::now().toSec();
  //goal_state = GetState(goal_state_id);
  //if (!goal_state){
    //goal_state = GetState(goal_state_id);
  //} else {
    for (int g_id = 0; g_id < num_islands; ++g_id){
      goal_state[g_id].g = INFINITECOST;
      goal_state[g_id].v = INFINITECOST;
      goal_state[g_id].iteration_closed = -1;
      goal_state[g_id].replan_number = replan_number;
      goal_state[g_id].best_parent = NULL;
      goal_state[g_id].expanded_best_parent = NULL;
      goal_state[g_id].best_edge_type = EdgeType::NONE;
      goal_state[g_id].expanded_best_edge_type = EdgeType::NONE;
      goal_state[g_id].snap_midpoint = -1;
      goal_state[g_id].expanded_snap_midpoint = -1;
      goal_state[g_id].heapindex = 0;
      goal_state[g_id].in_incons = false;
      goal_state[g_id].isTrueCost = true;
    }


  //}
  for (int g_id=0; g_id < num_islands; g_id++)
    {island_state[g_id] = GetState(g_id, island_state_id[g_id]);
        goal_state[g_id].h = 0;}

  // needed to add this because GetState calls the heuristic function on the
  // goal state before the heuristic has been initialized.


  //put start state in the heap

  //fadi

  for (int g_id=0; g_id < num_islands; g_id++){
    island_state[g_id]->g = 0;
    ROS_INFO("start state heuristic is %d", island_state[g_id]->h);
    assert(island_state[g_id]->h >= 0);
    CKey key;
    key.key[0] = eps*island_state[g_id]->h;
    heaps[g_id].insertheap(island_state[g_id], key);
  }
  //ensure heuristics are up-to-date
  //environment_->EnsureHeuristicsUpdated((bforwardsearch==true));
  //printf("computing heuristic took %f sec\n", ros::Time::now().toSec()-t0);
}

template <typename HeuristicType>
bool LazyAEGPlanner<HeuristicType>::Search(vector<vector<int>>& pathIds, int& PathCost){
  CKey key;
  TimeStarted = clock();

  initializeSearch();

  //the main loop of ARA*
  while(eps_satisfied > params.final_eps && !outOfTime()){

    //run weighted A*
    clock_t before_time = clock();
    int before_expands = search_expands;
    //ImprovePath returns:
    //1 if the solution is found
    //0 if the solution does not exist
    //2 if it ran out of time
    int ret = ImprovePath();
    if(ret == 1) //solution found for this iteration
      eps_satisfied = eps;
    int delta_expands = search_expands - before_expands;
    double delta_time = double(clock()-before_time)/CLOCKS_PER_SEC;

    //print the bound, expands, and time for that iteration
    printf("bound=%f expands=%d cost=%d time=%.2f\n",
        eps_satisfied*params.epsE, delta_expands, goal_state[0].g, delta_time);

    //update stats
    totalExpands += delta_expands;
    PlannerStats tempStat;
    tempStat.eps = eps_satisfied;
    tempStat.expands = delta_expands;
    tempStat.time = delta_time;
    tempStat.cost = goal_state[0].g;
    stats.push_back(tempStat);

    //no solution exists
    if(ret == 0){
      printf("Solution does not exist\n");
      return false;
    }

    //if we're just supposed to find the first solution
    //or if we ran out of time, we're done
    if(params.return_first_solution || ret == 2)
      break;

    prepareNextSearchIteration();
  }

  if(goal_state[0].g == INFINITECOST){
    printf("could not find a solution (ran out of time)\n");
    return false;
  }
  if(eps_satisfied == INFINITECOST)
    printf("WARNING: a solution was found but we don't have quality bound for it!\n");

  printf("solution found\n");
  // pathIds = GetSearchPath(PathCost);

  return true;
}

template <typename HeuristicType>
void LazyAEGPlanner<HeuristicType>::prepareNextSearchIteration(){
  //decrease epsilon
  eps -= params.dec_eps;
  if(eps < params.final_eps)
    eps = params.final_eps;

  //dump the inconsistent states into the open list
  CKey key;
  for (int g_id = 0; g_id < num_islands; ++g_id)
  {
    while(!incons[g_id].empty()){
      LazyAEGState* s = incons[g_id].back();
      incons[g_id].pop_back();
      s->in_incons = false;
      key.key[0] = s->g + int(eps * s->h);
      heaps[g_id].insertheap(s,key);
    }

    //recompute priorities for states in OPEN and reorder it
    for (int i=1; i<=heaps[g_id].currentsize; ++i){
      LazyAEGState* state = (LazyAEGState*)heaps[g_id].heap[i].heapstate;
      heaps[g_id].heap[i].key.key[0] = state->g + int(eps * state->h);
    }
    heaps[g_id].makeheap();
  }
  search_iteration++;
}


//-----------------------------Interface function-----------------------------------------------------

template <typename HeuristicType>
void LazyAEGPlanner<HeuristicType>::interrupt(){
  interruptFlag = true;
}

template <typename HeuristicType>
int LazyAEGPlanner<HeuristicType>::replan(vector<vector<int>>* solution_stateIDs_V, EGraphReplanParams p){
  int solcost;
  return replan(solution_stateIDs_V, p, &solcost);
}

template <typename HeuristicType>
int LazyAEGPlanner<HeuristicType>::replan(int start, vector<vector<int>>* solution_stateIDs_V, EGraphReplanParams p, int* solcost){
  set_start(start); //todo
  //set_goal(goal);
  return replan(solution_stateIDs_V, p, solcost);
}

template <typename HeuristicType>
int LazyAEGPlanner<HeuristicType>::replan(vector<vector<int>>* solution_stateIDs_V, EGraphReplanParams p, int* solcost){
  clock_t replan_t0 = clock();
  printf("planner: replan called\n");
  params = p;
  use_repair_time = params.repair_time >= 0;
  interruptFlag = false;

  egraph_mgr_->setEpsE(p.epsE);
  set_goal();

  if(start_state_id < 0){
    printf("ERROR searching: no start state set\n");
    return 0;
  }
  if (egraph_mgr_->egraph_env_->isGoal(start_state_id)){
    ROS_WARN("start is goal! nothing interesting returned");
    return true;
  }

  //plan
  vector<vector<int>> pathIds;
  int PathCost;
  bool solnFound = Search(pathIds, PathCost);

  //copy the solution
  *solution_stateIDs_V = pathIds;
  *solcost = PathCost;

  start_state_id = -1;
  //goal_state_id = -1;

  clock_t replan_t1 = clock();
  totalPlanTime = double(replan_t1-replan_t0)/CLOCKS_PER_SEC;

  printf("\n---------------------------------------------------------------\n");
  if(solnFound)
    printf("Solution found!\n");
  else
    printf("Solution not found...\n");
  printf("total time=%.2f total time without counting adding new path=%.2f\n",
          totalPlanTime, totalPlanTime-feedbackPathTime);
  printf("total expands=%d solution cost=%d\n",
          totalExpands, goal_state[0].g);
  printf("time breakdown: heuristic set goal  = %.2f\n", heuristicSetGoalTime);
  printf("                heuristic           = %.2f\n", double(heuristicClock)/CLOCKS_PER_SEC);
  printf("                generate successors = %.2f\n", double(succsClock)/CLOCKS_PER_SEC);
  printf("                shortcuts           = %.2f\n", double(shortcutClock)/CLOCKS_PER_SEC);
  printf("                path reconstruction = %.2f\n", reconstructTime);
  printf("                feedback path       = %.2f\n", feedbackPathTime);
  printf("---------------------------------------------------------------\n\n");

  stat_map_["solution_found"] = solnFound;
  stat_map_["solution_bound"] = params.epsE*params.initial_eps;
  stat_map_["total_time"] = totalPlanTime;
  stat_map_["total_time_without_feedback"] = totalPlanTime-feedbackPathTime;
  stat_map_["expands"] = totalExpands;
  stat_map_["solution_cost"] = goal_state[0].g;
  stat_map_["heuristic_set_goal_time"] = heuristicSetGoalTime;
  stat_map_["heuristic_time"] = double(heuristicClock)/CLOCKS_PER_SEC;
  stat_map_["generate_successors_time"] = double(succsClock)/CLOCKS_PER_SEC;
  stat_map_["shortcuts_time"] = double(shortcutClock)/CLOCKS_PER_SEC;
  stat_map_["path_reconstruction_time"] = reconstructTime;
  stat_map_["feedback_path_time"] = feedbackPathTime;

  return (int)solnFound;
}

template <typename HeuristicType>
int LazyAEGPlanner<HeuristicType>::set_goal(){
  /*
  printf("planner: setting goal to %d\n", id);
  if(bforwardsearch)
    goal_state_id = id;
  else
    start_state_id = id;
  */
  if (!params.use_lazy_validation){
      ROS_INFO("fully validating egraph");
      egraph_mgr_->validateEGraph();
  }
  clock_t t0 = clock();
  egraph_mgr_->setGoal();
  clock_t t1 = clock();
  heuristicSetGoalTime = double(t1-t0)/CLOCKS_PER_SEC;

  if (!params.use_lazy_validation){
      egraph_mgr_->initEGraph();
  }
  return 1;
}

template <typename HeuristicType>
int LazyAEGPlanner<HeuristicType>::set_start(int id){
  //printf("planner: setting start to %d\n", id);
  //if(bforwardsearch)
  // for (int g_id=0; g_id < num_islands; g_id++)
  //  island_state_id[g_id] = id;


  start_state_id = id;    //todo
  //else
    //goal_state_id = id;
  return 1;
}

template <typename HeuristicType>
int LazyAEGPlanner<HeuristicType>::set_islands(int g_id, int id){
  island_state_id[g_id] = id;
  return 1;
}


template <typename HeuristicType>
void LazyAEGPlanner<HeuristicType>::feedback_last_path(){
    egraph_mgr_->feedbackLastPath();
    printf("validitycheck time=%.3f feedbacktime %.3f errorcheck time=%.3f\n",
            egraph_mgr_->getStats().egraph_validity_check_time,
            egraph_mgr_->getStats().feedback_time,
            egraph_mgr_->getStats().error_check_time);
}


//---------------------------------------------------------------------------------------------------------


template <typename HeuristicType>
void LazyAEGPlanner<HeuristicType>::get_search_stats(vector<PlannerStats>* s){
  s->clear();
  s->reserve(stats.size());
  for(unsigned int i=0; i<stats.size(); i++){
    s->push_back(stats[i]);
  }
}

