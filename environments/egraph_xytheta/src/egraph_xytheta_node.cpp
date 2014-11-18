#include <egraph_xytheta/egraph_xytheta_node.h>
#include <nav_msgs/Path.h>

using namespace std;

EGraphXYThetaNode::EGraphXYThetaNode(costmap_2d::Costmap2DROS* costmap_ros) {
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;

  private_nh.param("primitive_filename",primitive_filename_,string(""));
  double nominalvel_mpersecs, timetoturn45degsinplace_secs;
  private_nh.param("nominalvel_mpersecs", nominalvel_mpersecs, 0.4);
  private_nh.param("timetoturn45degsinplace_secs", timetoturn45degsinplace_secs, 0.6);

  int lethal_obstacle;
  private_nh.param("lethal_obstacle",lethal_obstacle,20);
  lethal_obstacle_ = (unsigned char) lethal_obstacle;
  inscribed_inflated_obstacle_ = lethal_obstacle_-1;
  sbpl_cost_multiplier_ = (unsigned char) (costmap_2d::INSCRIBED_INFLATED_OBSTACLE/inscribed_inflated_obstacle_ + 1);

  costmap_ros_ = costmap_ros;
  costmap_ros_->clearRobotFootprint();
  costmap_ros_->getCostmapCopy(cost_map_);

  std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();

  env_ = new EGraphXYTheta();

  if(!env_->SetEnvParameter("cost_inscribed_thresh",costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))){
    ROS_ERROR("Failed to set cost_inscribed_thresh parameter");
    exit(1);
  }
  if(!env_->SetEnvParameter("cost_possibly_circumscribed_thresh", costMapCostToSBPLCost(cost_map_.getCircumscribedCost()))){
    ROS_ERROR("Failed to set cost_possibly_circumscribed_thresh parameter");
    exit(1);
  }
  int obst_cost_thresh = costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE);
  vector<sbpl_2Dpt_t> perimeterptsV;
  perimeterptsV.reserve(footprint.size());
  for (size_t ii(0); ii < footprint.size(); ++ii) {
    sbpl_2Dpt_t pt;
    pt.x = footprint[ii].x;
    pt.y = footprint[ii].y;
    perimeterptsV.push_back(pt);
  }

  bool ret;
  try{
    ret = env_->InitializeEnv(costmap_ros_->getSizeInCellsX(), // width
        costmap_ros_->getSizeInCellsY(), // height
        0, // mapdata
        0, 0, 0, // start (x, y, theta, t)
        0, 0, 0, // goal (x, y, theta)
        0, 0, 0, //goal tolerance
        perimeterptsV, costmap_ros_->getResolution(), nominalvel_mpersecs,
        timetoturn45degsinplace_secs, obst_cost_thresh,
        primitive_filename_.c_str());
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception!");
    ret = false;
  }
  if(!ret){
    ROS_ERROR("SBPL initialization failed!");
    exit(1);
  }
  for (ssize_t ix(0); ix < costmap_ros_->getSizeInCellsX(); ++ix)
    for (ssize_t iy(0); iy < costmap_ros_->getSizeInCellsY(); ++iy)
      env_->UpdateCost(ix, iy, costMapCostToSBPLCost(cost_map_.getCost(ix,iy)));

  string egraph_filename;
  private_nh.param<string>("egraph_filename", egraph_filename, "");
  if(egraph_filename.empty())
    egraph_ = new EGraph(env_,3, 0);
  else
    egraph_ = new EGraph(env_,egraph_filename);

  heur_ = new EGraph2dGridHeuristic(*env_, costmap_ros_->getSizeInCellsX(), costmap_ros_->getSizeInCellsY(), NAVXYTHETALAT_COSTMULT_MTOMM/nominalvel_mpersecs);
  egraph_mgr_ = new EGraphManager<vector<int> >(egraph_, env_, heur_);
  planner_ = new LazyAEGPlanner<vector<int> >(env_, true, egraph_mgr_);
  egraph_vis_ = new EGraphVisualizer(egraph_, env_);

  egraph_vis_->visualize();

  interrupt_sub_ = nh.subscribe("/sbpl_planning/interrupt", 1, &EGraphXYThetaNode::interruptPlannerCallback,this);
  plan_pub_ = nh.advertise<nav_msgs::Path>("plan", 1);
  plan_service_ = nh.advertiseService("/sbpl_planning/plan_path",&EGraphXYThetaNode::makePlan,this);
}

void EGraphXYThetaNode::interruptPlannerCallback(std_msgs::EmptyConstPtr){
  ROS_WARN("Planner interrupt received!");
  planner_->interrupt();
}

//Taken from Sachin's sbpl_cart_planner
//This rescales the costmap according to a rosparam which sets the obstacle cost
unsigned char EGraphXYThetaNode::costMapCostToSBPLCost(unsigned char newcost){
  if(newcost == costmap_2d::LETHAL_OBSTACLE)
    return lethal_obstacle_;
  else if(newcost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    return inscribed_inflated_obstacle_;
  else if(newcost == 0 || newcost == costmap_2d::NO_INFORMATION)
    return 0;
  else
    return (unsigned char) (newcost/sbpl_cost_multiplier_ + 0.5);
}

bool EGraphXYThetaNode::makePlan(egraph_xytheta::GetXYThetaPlan::Request& req, egraph_xytheta::GetXYThetaPlan::Response& res){

  ROS_DEBUG("[sbpl_lattice_planner] getting fresh copy of costmap");
  costmap_ros_->clearRobotFootprint();
  ROS_DEBUG("[sbpl_lattice_planner] robot footprint cleared");

  costmap_ros_->getCostmapCopy(cost_map_);

  try{
    int ret = env_->SetStart(req.start_x - cost_map_.getOriginX(), req.start_y - cost_map_.getOriginY(), req.start_theta);
    if(ret < 0 || planner_->set_start(ret) == 0){
      ROS_ERROR("ERROR: failed to set start state\n");
      return false;
    }
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while setting the start state");
    return false;
  }

  try{
    int ret = env_->SetGoal(req.goal_x - cost_map_.getOriginX(), req.goal_y - cost_map_.getOriginY(), req.goal_theta);
    if(ret < 0 || planner_->set_goal(ret) == 0){
      ROS_ERROR("ERROR: failed to set goal state\n");
      return false;
    }
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while setting the goal state");
    return false;
  }

  vector<vector<bool> > heur_grid(cost_map_.getSizeInCellsX(), vector<bool>(cost_map_.getSizeInCellsY(), false));
  for(unsigned int ix = 0; ix < cost_map_.getSizeInCellsX(); ix++){
    for(unsigned int iy = 0; iy < cost_map_.getSizeInCellsY(); iy++){
      unsigned char c = costMapCostToSBPLCost(cost_map_.getCost(ix,iy));
      env_->UpdateCost(ix, iy, c);
      if(c >= inscribed_inflated_obstacle_)
        heur_grid[ix][iy] = true;
    }
  }
  heur_->setGrid(heur_grid);

  EGraphReplanParams params(5.0);
  params.initial_eps = req.initial_eps;
  params.dec_eps = req.dec_eps;
  params.final_eps = req.final_eps;
  params.epsE = req.egraph_eps;
  params.dec_epsE = req.dec_egraph_eps;
  params.final_epsE = req.final_egraph_eps;
  params.return_first_solution = false;
  params.use_egraph = req.use_egraph;
  params.feedback_path = req.feedback_path;

  vector<int> solution_stateIDs;
  bool ret = planner_->replan(&solution_stateIDs, params);

  map<string,double> stats = planner_->getStats();
  for(map<string,double>::iterator it=stats.begin(); it!=stats.end(); it++){
    res.stat_names.push_back(it->first);
    res.stat_values.push_back(it->second);
  }
  if(!ret)
    return false;

  if(req.save_egraph)
    egraph_->save("xytheta_egraph.eg");

  ros::Time plan_time = ros::Time::now();

  //create a message for the plan 
  nav_msgs::Path gui_path;
  gui_path.poses.resize(solution_stateIDs.size());
  gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
  gui_path.header.stamp = plan_time;
  for(unsigned int i=0; i<solution_stateIDs.size(); i++){
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = plan_time;
    pose.header.frame_id = costmap_ros_->getGlobalFrameID();

    vector<double> coord(3,0);
    env_->getCoord(solution_stateIDs[i],coord);

    pose.pose.position.x = coord[0] + cost_map_.getOriginX();
    pose.pose.position.y = coord[1] + cost_map_.getOriginY();
    pose.pose.position.z = 0;

    tf::Quaternion temp;
    temp.setEulerZYX(coord[2],0,0);
    pose.pose.orientation.x = temp.getX();
    pose.pose.orientation.y = temp.getY();
    pose.pose.orientation.z = temp.getZ();
    pose.pose.orientation.w = temp.getW();

    res.path.push_back(pose);

    gui_path.poses[i].pose.position.x = pose.pose.position.x;
    gui_path.poses[i].pose.position.y = pose.pose.position.y;
    gui_path.poses[i].pose.position.z = pose.pose.position.z;
  }
  plan_pub_.publish(gui_path);

  egraph_vis_->visualize();

  return true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "egraph_xytheta_node");

  tf::TransformListener tf_;
  costmap_2d::Costmap2DROS* costmap = new costmap_2d::Costmap2DROS("global_costmap", tf_);
  costmap->pause();

  EGraphXYThetaNode xytheta(costmap);

  //ros::spin();
  ros::MultiThreadedSpinner spinner(2);//need 2 threads to catch a the interrupt
  spinner.spin();
}

