#include <monolithic_pr2_planner_node/EnvInterfaces.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>
#include <monolithic_pr2_planner/SearchRequest.h>
#include <monolithic_pr2_planner/Constants.h>
#include <kdl/frames.hpp>
#include <boost/filesystem.hpp>
#include <geometry_msgs/Pose.h>
#include <leatherman/utils.h>
#include <LinearMath/btVector3.h>
#include <egraphs/egraphManager.h>

using namespace monolithic_pr2_planner_node;
using namespace monolithic_pr2_planner;
using namespace boost;
using namespace std;
using namespace KDL;

// constructor automatically launches the collision space interface, which only
// loads it up with a pointer to the collision space mgr. it doesn't bind to any
// topic.

EnvInterfaces::EnvInterfaces(boost::shared_ptr<monolithic_pr2_planner::EGraphEnvironment> env, ros::NodeHandle nh) : 
    m_nodehandle(nh),
    m_env(env), 
    m_collision_space_interface(new CollisionSpaceInterface(env->getCollisionSpace(), 
                                                            env->getHeuristicMgr(),
                                                            NULL)),
    m_generator(new StartGoalGenerator(env->getCollisionSpace())) {

    m_collision_space_interface->mutex = &mutex;

    getParams();
    RobotState::setPlanningMode(PlanningModes::RIGHT_ARM_MOBILE);

    string egraph_filename;
    m_nodehandle.param<string>("egraph_filename", egraph_filename, "");
    if(egraph_filename.empty())
      egraph_ = new EGraph(m_env.get(),12, 2);
    else
      egraph_ = new EGraph(m_env.get(), egraph_filename);

    //heur_ = new EGraph3dGridHeuristic(*(m_env.get()),10.0/0.02,6.0/0.02,2.0/0.02,20);
    /*
    double arm_xyz_weight = 40.0/0.02 * 0.1;
    double arm_rpy_weight = 40.0/2.0/(5.625*M_PI/180.0) * 0.1;
    double arm_fa_weight = 40.0/5.0/(2.0*M_PI/180.0) * 0.01;//artificially downweight to optimize this last
    double base_xy_weight = 40.0/0.02;
    double base_z_weight = 3000.0/0.02 * 0.001;
    double base_theta_weight = 251.0/(M_PI/8.0); 
    vector<double> heuristic_weights = {arm_xyz_weight, arm_xyz_weight, arm_xyz_weight,
                                        arm_rpy_weight, arm_rpy_weight, arm_rpy_weight,
                                        arm_fa_weight, arm_fa_weight,
                                        base_xy_weight, base_xy_weight,
                                        base_z_weight, base_theta_weight};
    vector<bool> continuous = {false, false, false, true, true, true, false, false, false, false, false, true};
    vector<double> snap_dist = {0.40, 0.40, 0.40, 
      2.0*M_PI, 2.0*M_PI, 2.0*M_PI, 
      2.0*M_PI, 2.0*M_PI,
      0.04, 0.04, 0.31, 3.0*M_PI/16.0};
    heur_ = new EGraphEuclideanHeuristic(*(m_env.get()),heuristic_weights,continuous);
      */

    vector<double> snap_dist = {0.40, 0.40, 0.40, 
      2.0*M_PI, 0.04, 0.04, 0.04, 0.04, 0.31};
    heur_ = new EGraphEuclideanHeuristic(*(m_env.get()));
    heur_->setSnapDist(snap_dist);
    EGraphManager<vector<double> >* egraph_mgr = new EGraphManager<vector<double> >(egraph_, m_env.get(), heur_);
    bool forward_search = true;
    m_ara_planner.reset(new LazyAEGPlanner<vector<double> >(m_env.get(), forward_search, egraph_mgr));
    m_egraph_vis.reset(new EGraphVisualizer(egraph_, m_env.get()));

    interrupt_sub_ = nh.subscribe("/sbpl_planning/interrupt", 1, &EnvInterfaces::interruptPlannerCallback,this);
    m_costmap_pub = m_nodehandle.advertise<nav_msgs::OccupancyGrid>("costmap_pub", 1);
    m_costmap_publisher.reset(new costmap_2d::Costmap2DPublisher(m_nodehandle,1,"/map"));

    m_egraph_vis->visualize();
}

void EnvInterfaces::interruptPlannerCallback(std_msgs::EmptyConstPtr){
  ROS_WARN("Planner interrupt received!");
  m_ara_planner->interrupt();
}

void EnvInterfaces::startOMPLPlanners(){
    m_rrt.reset(new OMPLPR2Planner(m_env->getCollisionSpace(), RRT));
    m_prm.reset(new OMPLPR2Planner(m_env->getCollisionSpace(), PRM_P));
    m_rrtstar.reset(new OMPLPR2Planner(m_env->getCollisionSpace(), RRTSTAR));
    m_rrtstar_first_sol.reset(new OMPLPR2Planner(m_env->getCollisionSpace(), 
                                                 RRTSTARFIRSTSOL));
}

/*! \brief grabs parameters from param server
 */
void EnvInterfaces::getParams(){
    m_nodehandle.param<string>("reference_frame", m_params.ref_frame, 
                                    string("map"));
}

/*! \brief advertise the planPath callback
 */
void EnvInterfaces::bindPlanPathToEnv(string service_name){
    m_plan_service = m_nodehandle.advertiseService(service_name, 
                                                   &EnvInterfaces::planPathCallback,
                                                   this);
}

void EnvInterfaces::bindWriteExperimentToEnv(string service_name){
  m_write_experiments_service = m_nodehandle.advertiseService(service_name,
      &EnvInterfaces::GenerateExperimentFile,
      this);
}

void EnvInterfaces::weirdEGraphInit(std::vector<string>& names, std::vector<double>& min,
                                    std::vector<double>& max, std::vector<double>& res){
    names.push_back("obj_x");
    names.push_back("obj_y");
    names.push_back("obj_z");
    names.push_back("obj_roll");
    names.push_back("obj_pitch");
    names.push_back("obj_yaw");
    names.push_back("free_angle_right");
    names.push_back("free_angle_left");
    names.push_back("base_x");
    names.push_back("base_y");
    names.push_back("spine_z");
    names.push_back("base_theta");

    min.push_back(0.0);
    min.push_back(0.0);
    min.push_back(0.0);
    min.push_back(0.0);
    min.push_back(0.0);
    min.push_back(0.0);
    min.push_back(0.0);
    min.push_back(0.0);
    min.push_back(0.0);
    min.push_back(0.0);
    min.push_back(0.0);
    min.push_back(0.0);

    max.push_back(2.4);
    max.push_back(2.4);
    max.push_back(2.0);
    max.push_back(2*M_PI);
    max.push_back(2*M_PI);
    max.push_back(2*M_PI);
    max.push_back(2*M_PI);
    max.push_back(2*M_PI);
    max.push_back(9.0);
    max.push_back(5.0);
    max.push_back(2.0);
    max.push_back(2*M_PI);

    res.push_back(0.02);
    res.push_back(0.02);
    res.push_back(0.02);
    res.push_back(5.625*M_PI/180);
    res.push_back(5.625*M_PI/180);
    res.push_back(5.625*M_PI/180);
    res.push_back(2.000*M_PI/180);
    res.push_back(2.000*M_PI/180);
    res.push_back(0.02);
    res.push_back(0.02);
    res.push_back(0.02);
    res.push_back(2*M_PI/16);
}

//making experiments:
////launch file brings up stlToOctomap with rosparams that tell it to randomize environment
////addTableObstacles
////randomizeTableObstacles - if this is false we have to provide the next one (when true we have to write configuration to file)
////pathToTableObstacleParamFile (this file has num_surfaces, num_obs, and seed)
////
////service call is made to this function with the number of trials to make
////this guy makes random trials with checks for rrt-connect feasibility
////it writes the start/goal pairs to file (in the yaml format)
////
////////////
////
////running experiments:
////stlToOctomap is launched with rosparams (and table config file) to re-create the env
////runTests is run with the yaml file
bool EnvInterfaces::GenerateExperimentFile(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  ROS_INFO("generating trials!");
  vector<pair<RobotState, RobotState> > start_goal_pairs;
  RobotState::setPlanningMode(PlanningModes::RIGHT_ARM_MOBILE);
  int number_of_trials = 100;
  m_generator->initializeRegions();//reads from ros params set by stlToOctomap
  m_generator->generateUniformPairs(number_of_trials, start_goal_pairs);

  int test_num = 0;
  FILE* fout = fopen("fbp_tests.yaml","w");
  fprintf(fout, "experiments:\n\n");
  for (auto& start_goal : start_goal_pairs){
    geometry_msgs::Quaternion start_obj_q;
    leatherman::rpyToQuatMsg(start_goal.first.getObjectStateRelMap().roll(),
        start_goal.first.getObjectStateRelMap().pitch(),
        start_goal.first.getObjectStateRelMap().yaw(),
        start_obj_q);
    geometry_msgs::Quaternion goal_obj_q;
    leatherman::rpyToQuatMsg(start_goal.second.getObjectStateRelMap().roll(),
        start_goal.second.getObjectStateRelMap().pitch(),
        start_goal.second.getObjectStateRelMap().yaw(),
        goal_obj_q);

    fprintf(fout,"  - test: test_%d\n", test_num);
    fprintf(fout,"    start:\n");
    fprintf(fout,"      object_xyz_wxyz: %f %f %f %f %f %f %f\n",
        start_goal.first.getObjectStateRelMap().x(),
        start_goal.first.getObjectStateRelMap().y(),
        start_goal.first.getObjectStateRelMap().z(),
        start_obj_q.w, start_obj_q.x, start_obj_q.y, start_obj_q.z);
    fprintf(fout,"      base_xyzyaw: %f %f %f %f\n",
        start_goal.first.getContBaseState().x(),
        start_goal.first.getContBaseState().y(),
        start_goal.first.getContBaseState().z(),
        start_goal.first.getContBaseState().theta());
    fprintf(fout,"      rarm: %f %f %f %f %f %f %f\n",
        start_goal.first.right_arm().getShoulderPanAngle(),
        start_goal.first.right_arm().getShoulderLiftAngle(),
        start_goal.first.right_arm().getUpperArmRollAngle(),
        start_goal.first.right_arm().getElbowFlexAngle(),
        start_goal.first.right_arm().getForearmRollAngle(),
        start_goal.first.right_arm().getWristFlexAngle(),
        start_goal.first.right_arm().getWristRollAngle());
    fprintf(fout,"      larm: %f %f %f %f %f %f %f\n",
        start_goal.first.left_arm().getShoulderPanAngle(),
        start_goal.first.left_arm().getShoulderLiftAngle(),
        start_goal.first.left_arm().getUpperArmRollAngle(),
        start_goal.first.left_arm().getElbowFlexAngle(),
        start_goal.first.left_arm().getForearmRollAngle(),
        start_goal.first.left_arm().getWristFlexAngle(),
        start_goal.first.left_arm().getWristRollAngle());
    fprintf(fout,"    goal:\n");
    fprintf(fout,"      object_xyz_wxyz: %f %f %f %f %f %f %f\n",
        start_goal.second.getObjectStateRelMap().x(),
        start_goal.second.getObjectStateRelMap().y(),
        start_goal.second.getObjectStateRelMap().z(),
        start_obj_q.w, start_obj_q.x, start_obj_q.y, start_obj_q.z);
    fprintf(fout,"      base_xyzyaw: %f %f %f %f\n",
        start_goal.second.getContBaseState().x(),
        start_goal.second.getContBaseState().y(),
        start_goal.second.getContBaseState().z(),
        start_goal.second.getContBaseState().theta());
    fprintf(fout,"      rarm: %f %f %f %f %f %f %f\n",
        start_goal.second.right_arm().getShoulderPanAngle(),
        start_goal.second.right_arm().getShoulderLiftAngle(),
        start_goal.second.right_arm().getUpperArmRollAngle(),
        start_goal.second.right_arm().getElbowFlexAngle(),
        start_goal.second.right_arm().getForearmRollAngle(),
        start_goal.second.right_arm().getWristFlexAngle(),
        start_goal.second.right_arm().getWristRollAngle());
    fprintf(fout,"      larm: %f %f %f %f %f %f %f\n",
        start_goal.second.left_arm().getShoulderPanAngle(),
        start_goal.second.left_arm().getShoulderLiftAngle(),
        start_goal.second.left_arm().getUpperArmRollAngle(),
        start_goal.second.left_arm().getElbowFlexAngle(),
        start_goal.second.left_arm().getForearmRollAngle(),
        start_goal.second.left_arm().getWristFlexAngle(),
        start_goal.second.left_arm().getWristRollAngle());
    fprintf(fout,"\n");
    test_num++;
  }
  fclose(fout);
  return true;
}

/*! \brief takes in a service call request and runs the planner, spits back
 * statistics and a plan.
 */
bool EnvInterfaces::planPathCallback(GetMobileArmPlan::Request &req, 
                                     GetMobileArmPlan::Response &res)
{
    boost::unique_lock<boost::mutex> lock(mutex);

    SearchRequestParamsPtr search_request = make_shared<SearchRequestParams>();
    search_request->initial_epsilon = req.initial_eps;
    search_request->final_epsilon = req.final_eps;
    search_request->decrement_epsilon = req.dec_eps;
    search_request->obj_goal= req.goal;
    search_request->base_start = req.body_start;
    search_request->left_arm_start = LeftContArmState(req.larm_start);
    search_request->right_arm_start = RightContArmState(req.rarm_start);
    search_request->base_goal = req.body_goal;
    search_request->left_arm_goal = LeftContArmState(req.larm_goal);
    search_request->right_arm_goal = RightContArmState(req.rarm_goal);

    KDL::Frame rarm_offset, larm_offset;
    rarm_offset.p.x(req.rarm_object.pose.position.x);
    rarm_offset.p.y(req.rarm_object.pose.position.y);
    rarm_offset.p.z(req.rarm_object.pose.position.z);
    larm_offset.p.x(req.larm_object.pose.position.x);
    larm_offset.p.y(req.larm_object.pose.position.y);
    larm_offset.p.z(req.larm_object.pose.position.z);

    rarm_offset.M = Rotation::Quaternion(req.rarm_object.pose.orientation.x, 
                                         req.rarm_object.pose.orientation.y, 
                                         req.rarm_object.pose.orientation.z, 
                                         req.rarm_object.pose.orientation.w);
    larm_offset.M = Rotation::Quaternion(req.larm_object.pose.orientation.x, 
                                         req.larm_object.pose.orientation.y, 
                                         req.larm_object.pose.orientation.z, 
                                         req.larm_object.pose.orientation.w);
    search_request->left_arm_object = larm_offset;
    search_request->right_arm_object = rarm_offset;
    search_request->xyz_tolerance = req.xyz_tolerance;
    search_request->roll_tolerance = req.roll_tolerance;
    search_request->pitch_tolerance = req.pitch_tolerance;
    search_request->yaw_tolerance = req.yaw_tolerance;
    search_request->planning_mode = req.planning_mode;

    res.stats_field_names.resize(18);
    res.stats.resize(18);
    int start_id, goal_id;
    vector<double> stats;
    vector<string> stat_names;
    static int counter = 0;
    vector<FullBodyState> states;

    std::string planner_prefix = "eg_";

    //m_env->reset();

    double total_planning_time = clock();
    double t0 = ros::Time::now().toSec();
    bool retVal = m_env->configureRequest(search_request, start_id, goal_id);
    if(!retVal){
      ROS_ERROR("Unable to configure request for Trial ID: %d", counter);

      total_planning_time = -1;
      int soln_cost = -1;
      packageStats(stat_names, stats, soln_cost, 0, total_planning_time);
      for(unsigned int i=0; i<stats.size(); i++)
        stats[i] = -1;
      m_stats_writer.writeSBPL(stats, states, counter, planner_prefix);
      res.stats_field_names = stat_names;
      res.stats = stats;
      return true;
    }
    ROS_INFO("configuring time (plus validity checking) is %f %f", ros::Time::now().toSec() - t0, 
                (clock()-total_planning_time)/static_cast<double>(CLOCKS_PER_SEC));

    if(req.use_ompl){
      ROS_INFO("rrt init");
      RobotState::setPlanningMode(PlanningModes::RIGHT_ARM_MOBILE);
      m_rrt.reset(new OMPLPR2Planner(m_env->getCollisionSpace(), RRT));
      ROS_INFO("rrt check request");
      if (!m_rrt->checkRequest(*search_request))
        ROS_WARN("bad start goal for ompl");
      ROS_INFO("rrt plan");
      m_rrt->setPlanningTime(req.allocated_planning_time);
      double t0 = ros::Time::now().toSec();
      bool found_path = m_rrt->planPathCallback(*search_request, counter, m_stats_writer);
      double t1 = ros::Time::now().toSec();
      ROS_INFO("rrt done planning");

      res.stats_field_names.clear();
      res.stats_field_names.push_back("total_plan_time");
      res.stats.clear();
      if(found_path)
        res.stats.push_back(t1-t0);
      else
        res.stats.push_back(-1.0);

      sleep(5);
      return true;
    }
    else{
      m_ara_planner->set_start(start_id);
      m_ara_planner->force_planning_from_scratch();
      vector<int> soln;
      int soln_cost;

      EGraphReplanParams params(req.allocated_planning_time);
      params.initial_eps = req.initial_eps;
      params.dec_eps = req.dec_eps;
      params.final_eps = req.final_eps;
      params.epsE = req.egraph_eps;
      params.dec_epsE = req.final_egraph_eps;
      params.final_epsE = req.dec_egraph_eps;
      params.return_first_solution = false;
      params.use_egraph = req.use_egraph;
      params.feedback_path = req.feedback_path;

      double t2 = ros::Time::now().toSec();
      bool isPlanFound = m_ara_planner->replan(&soln, params, &soln_cost);
      ROS_INFO("planning time is %f", ros::Time::now().toSec() - t2);

      map<string,double> statsMap = m_ara_planner->getStats();
      for(map<string,double>::iterator it=statsMap.begin(); it!=statsMap.end(); it++){
        res.stat_names.push_back(it->first);
        res.stat_values.push_back(it->second);
      }

      double t3 = ros::Time::now().toSec();
      m_ara_planner->feedback_last_path();
      ROS_INFO("feedback time is %f", ros::Time::now().toSec() - t3);

      if (isPlanFound){
        ROS_INFO("Plan found. Moving on to reconstruction.");
        double t4 = ros::Time::now().toSec();
        states =  m_env->reconstructPath(soln);
        ROS_INFO("env reconsturction time is %f", ros::Time::now().toSec() - t4);
        total_planning_time = clock() - total_planning_time;
        ROS_INFO("total time %f %f", ros::Time::now().toSec()-t0, 
            total_planning_time/static_cast<double>(CLOCKS_PER_SEC));
        vector<string> stat_names;
        vector<double> stats;
        packageStats(stat_names, stats, soln_cost, states.size(),
            total_planning_time);
        m_stats_writer.writeSBPL(stats, states, counter, planner_prefix);
        res.stats_field_names = stat_names;
        res.stats = stats;

        if(req.save_egraph)
          egraph_->save("fbp_egraph.eg");
      } 
      else {
        ROS_INFO("No plan found!");
        packageStats(stat_names, stats, soln_cost, states.size(), total_planning_time);
        res.stats_field_names = stat_names;
        res.stats = stats;
        ROS_INFO("No plan found in %s!", planner_prefix.c_str());
      }
      m_egraph_vis->visualize();
      return true;
    }
}

/*! \brief fills in two vectors with stat names and values. needs a couple
 * parameters that can't be retrieved from the planner member variable.
 */
void EnvInterfaces::packageStats(vector<string>& stat_names, 
                                 vector<double>& stats,
                                 int solution_cost,
                                 size_t solution_size,
                                 double total_planning_time)
{
    
    stat_names.resize(4);
    stats.resize(4);
    stat_names[0] = "total plan time";
    stat_names[1] = "expansions";
    stat_names[2] = "solution cost";
    stat_names[3] = "path length";

    vector<PlannerStats> planner_stats;
    m_ara_planner->get_search_stats(&planner_stats);
    if(planner_stats.empty()){
      stats[0] = -1;
      stats[1] = -1;
      stats[2] = -1;
      stats[3] = -1;
    }
    else{
      stats[0] = planner_stats[0].time; //total_planning_time/static_cast<double>(CLOCKS_PER_SEC);
      stats[1] = planner_stats[0].expands;
      stats[2] = static_cast<double>(planner_stats[0].cost);//static_cast<double>(solution_cost);
      stats[3] = static_cast<double>(solution_size);
    }
}


bool EnvInterfaces::bindCollisionSpaceToTopic(string topic_name){
    m_collision_space_interface->bindCollisionSpaceToTopic(topic_name, 
                                                          m_tf, 
                                                          m_params.ref_frame);
    return true;
}

void EnvInterfaces::bindNavMapToTopic(string topic){
    m_nav_map = m_nodehandle.subscribe(topic, 1, &EnvInterfaces::loadNavMap, this);
}

/*! \brief given a single dimensional vector of numbers that represents a map,
 * crop it to the appropriate size. new_origin_x and y are for which coordinate
 * you want to be the origin. probably is going to be 0,0. width and height are
 * the dimensions that you want the new map to be starting from 0,0. in meters.
 */
void EnvInterfaces::crop2DMap(const nav_msgs::MapMetaData& map_info, 
                              const std::vector<unsigned char>& v,
                              double new_origin_x, double new_origin_y,
                              double width, double height){
    vector<vector<unsigned char> > tmp_map(map_info.height);
    for (unsigned int i=0; i < map_info.height; i++){
        for (unsigned int j=0; j < map_info.width; j++){
            tmp_map[i].push_back(v[i*map_info.width+j]);
        }
    }

    double res = map_info.resolution;
    int new_origin_x_idx = (new_origin_x-map_info.origin.position.x)/res;
    int new_origin_y_idx = (new_origin_y-map_info.origin.position.y)/res;
    int new_width = static_cast<int>((width/res) + 1 + 0.5);
    int new_height = static_cast<int>((height/res) + 1 + 0.5);
    ROS_DEBUG_NAMED(HEUR_LOG, "new origin: %d %d, width and height: %d %d",
                              new_origin_x_idx, new_origin_y_idx, new_width, 
                              new_height);
    ROS_DEBUG_NAMED(HEUR_LOG, "size of map %lu %lu", tmp_map.size(), 
                                                     tmp_map[0].size());

    vector<vector<unsigned char> > new_map(new_height);
    int row_count = 0;
    for (int i=new_origin_y_idx; i < new_origin_y_idx + new_height; i++){
        for (int j=new_origin_x_idx; j < new_origin_x_idx + new_width; j++){
            new_map[row_count].push_back(tmp_map[i][j]);
        }
        row_count++;
    }
    m_final_map.clear();
    m_cropped_map.clear();
    // m_final_map.resize(new_width * new_height);
    for (size_t i=0; i < new_map.size(); i++){
        for (size_t j=0; j < new_map[i].size(); j++){
            m_final_map.push_back(static_cast<signed char>(double(new_map[i][j])/255.0*100.0));
            m_cropped_map.push_back(new_map[i][j]);
        }
    }
    ROS_DEBUG_NAMED(HEUR_LOG, "size of final map: %lu", m_final_map.size());
}

void EnvInterfaces::loadNavMap(const nav_msgs::OccupancyGridPtr& map){
    boost::unique_lock<boost::mutex> lock(mutex);
    ROS_DEBUG_NAMED(CONFIG_LOG, "received navmap of size %u %u, resolution %f",
                    map->info.width, map->info.height, map->info.resolution);
    ROS_DEBUG_NAMED(CONFIG_LOG, "origin is at %f %f", map->info.origin.position.x,
                                                      map->info.origin.position.y);

    // look up the values from the occup grid parameters
    // This stuff is in cells.
    int dimX, dimY, dimZ;
    m_collision_space_interface->getOccupancyGridSize(dimX, dimY,
        dimZ);
    // This costmap_ros object listens to the map topic as defined
    // in the costmap_2d.yaml file.
    m_costmap_ros.reset(new costmap_2d::Costmap2DROS("costmap_2d", m_tf));

    // Get the underlying costmap in the cost_map object.
    // Publish for visualization. Publishing is done for the entire (uncropped) costmap.
    costmap_2d::Costmap2D cost_map;
    m_costmap_ros->getCostmapCopy(cost_map);

    // Normalize and convert to array.
    for (unsigned int j = 0; j < cost_map.getSizeInCellsY(); ++j)
    {
        for (unsigned int i = 0; i < cost_map.getSizeInCellsX(); ++i)
        {
            // Row major. X is row wise, Y is column wise.
            int c = cost_map.getCost(i,j);

            // Set unknowns to free space (we're dealing with static maps for
            // now)
            if (c == costmap_2d::NO_INFORMATION) {
                c = costmap_2d::FREE_SPACE;
            }
            // c = (c==costmap_2d::NO_INFORMATION)?costmap_2d::FREE_SPACE:c;

            // Re-set the cost.
            cost_map.setCost(i,j,c);
        }
    }

    // Re-inflate because we modified the unknown cells to be free space.
    cost_map.reinflateWindow(dimX*map->info.resolution/2, dimY*map->info.resolution/2, dimX*map->info.resolution, dimY*map->info.resolution);

    std::vector<unsigned char> uncropped_map;
    for (unsigned int j = 0; j < cost_map.getSizeInCellsY(); ++j)
    {
        for (unsigned int i = 0; i < cost_map.getSizeInCellsX(); ++i)
        {
            // Normalize the values from 0 to 100. Not absolutely needed, but
            // makes life easier when dealing with the heuristic later.
            //uncropped_map.push_back(static_cast<double>(cost_map.getCost(i,j))/costmap_2d::NO_INFORMATION*100);
            uncropped_map.push_back(cost_map.getCost(i,j));
        }
    }
    m_costmap_publisher->updateCostmapData(cost_map, m_costmap_ros->getRobotFootprint());

    // Publish the full costmap
    m_costmap_publisher->publishCostmap();
    m_costmap_publisher->publishFootprint();
    
    // std::vector<signed char> final_map;

    // TODO: Check if this is the right thing to do : Take the resolution from
    // the map for the occupancy grid's values.
    double width = dimX*map->info.resolution;
    double height = dimY*map->info.resolution;
    
    crop2DMap(map->info, uncropped_map, 0, 0, width, height);
    
    // Don't want to publish this.
    nav_msgs::OccupancyGrid costmap_pub;
    costmap_pub.header.frame_id = "/map";
    costmap_pub.header.stamp = ros::Time::now();
    costmap_pub.info.map_load_time = ros::Time::now();
    costmap_pub.info.resolution = map->info.resolution;
    // done in the crop function too.
    costmap_pub.info.width = (width/map->info.resolution+1 + 0.5);
    costmap_pub.info.height = (height/map->info.resolution+1 + 0.5);
    costmap_pub.info.origin.position.x = 0;
    costmap_pub.info.origin.position.y = 0;
    costmap_pub.data = m_final_map;

    // Publish the cropped version of the costmap
    m_costmap_pub.publish(costmap_pub);

    m_collision_space_interface->update2DHeuristicMaps(m_cropped_map);
    ROS_INFO("received 2D map!");
}
