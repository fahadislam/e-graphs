/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *********************************************************************/

#include<egraph_arm/planner_node.h>

using namespace sbpl_arm_planner;

void EGraphArmPlannerNode::fillConstraint(geometry_msgs::Pose pose, std::string frame_id, arm_navigation_msgs::Constraints &goals)
{
  goals.position_constraints.resize(1);
  goals.orientation_constraints.resize(1);
  goals.position_constraints[0].header.frame_id = frame_id;
  goals.position_constraints[0].position = pose.position;
  goals.orientation_constraints[0].orientation = pose.orientation;

  geometry_msgs::Pose p;
  p.position = goals.position_constraints[0].position;
  p.orientation = goals.orientation_constraints[0].orientation;
  leatherman::printPoseMsg(p, "Goal");

  goals.position_constraints[0].constraint_region_shape.dimensions.resize(3, 0.015);
  goals.orientation_constraints[0].absolute_roll_tolerance = 0.05;
  goals.orientation_constraints[0].absolute_pitch_tolerance = 0.05;
  goals.orientation_constraints[0].absolute_yaw_tolerance = 0.05;
  ROS_INFO("Done packing the goal constraints message.");
}

arm_navigation_msgs::CollisionObject EGraphArmPlannerNode::getCollisionCube(geometry_msgs::Pose pose, std::vector<double> &dims, std::string frame_id, std::string id)
{
  arm_navigation_msgs::CollisionObject object;
  object.id = id;
  object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  object.header.frame_id = frame_id;
  object.header.stamp = ros::Time::now();

  arm_navigation_msgs::Shape box_object;
  box_object.type = arm_navigation_msgs::Shape::BOX;
  box_object.dimensions.resize(3);
  box_object.dimensions[0] = dims[0];
  box_object.dimensions[1] = dims[1];
  box_object.dimensions[2] = dims[2];

  object.shapes.push_back(box_object);
  object.poses.push_back(pose);
  return object;
}

std::vector<arm_navigation_msgs::CollisionObject> EGraphArmPlannerNode::getCollisionCubes(std::vector<std::vector<double> > &objects, std::vector<std::string> &object_ids, std::string frame_id)
{
  std::vector<arm_navigation_msgs::CollisionObject> objs;
  std::vector<double> dims(3,0);
  geometry_msgs::Pose pose;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;

  if(object_ids.size() != objects.size())
  {
    ROS_INFO("object id list is not same length as object list. exiting.");
    return objs;
  }

  for(size_t i = 0; i < objects.size(); i++)
  {
    pose.position.x = objects[i][0];
    pose.position.y = objects[i][1];
    pose.position.z = objects[i][2];
    dims[0] = objects[i][3];
    dims[1] = objects[i][4];
    dims[2] = objects[i][5];

    objs.push_back(getCollisionCube(pose, dims, frame_id, object_ids.at(i)));
  }
  return objs;
}

std::vector<arm_navigation_msgs::CollisionObject> EGraphArmPlannerNode::getCollisionObjects(std::string filename, std::string frame_id)
{
  char sTemp[1024];
  int num_obs = 0;
  std::vector<std::string> object_ids;
  std::vector<std::vector<double> > objects;
  std::vector<arm_navigation_msgs::CollisionObject> objs;

  char* file = new char[filename.length()+1];
  filename.copy(file, filename.length(),0);
  file[filename.length()] = '\0';
  FILE* fCfg = fopen(file, "r");

  if(fCfg == NULL)
  {
    ROS_INFO("ERROR: unable to open objects file. Exiting.\n");
    return objs;
  }

  // get number of objects
  if(fscanf(fCfg,"%s",sTemp) < 1)
    printf("Parsed string has length < 1.\n");

  num_obs = atoi(sTemp);

  ROS_INFO("%i objects in file",num_obs);

  //get {x y z dimx dimy dimz} for each object
  objects.resize(num_obs);
  object_ids.clear();
  vector<int> color(4,0);
  for (int i=0; i < num_obs; ++i)
  {
    if(fscanf(fCfg,"%s",sTemp) < 1)
      printf("Parsed string has length < 1.\n");
    object_ids.push_back(sTemp);

    objects[i].resize(6);
    for(int j=0; j < 6; ++j)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        printf("Parsed string has length < 1.\n");
      if(!feof(fCfg) && strlen(sTemp) != 0)
        objects[i][j] = atof(sTemp);
    }
    for(int j=0; j < 4; ++j){
      if(fscanf(fCfg,"%s",sTemp) < 1)
        printf("Parsed string has length < 1.\n");
      if(!feof(fCfg) && strlen(sTemp) != 0)
        color[j] = atoi(sTemp);
    }

    //apply environment x,y,z,yaw

    double env_offset_x, env_offset_y, env_offset_z, env_offset_yaw;
    ph.param("env_offset_x", env_offset_x, 0.0);
    ph.param("env_offset_y", env_offset_y, 0.0);
    ph.param("env_offset_z", env_offset_z, 0.0);
    ph.param("env_offset_yaw", env_offset_yaw, 0.0);
    //ROS_INFO("tranlate environment by (%f %f %f) and rotate by %f",env_offset_x, env_offset_y, env_offset_z, env_offset_yaw);

    double theta = 0;
    if(fabs(angles::shortest_angular_distance(env_offset_yaw,0)) < 0.1)
      theta = 0;
    else if(fabs(angles::shortest_angular_distance(env_offset_yaw,M_PI/2.0)) < 0.1){
      theta = M_PI/2.0;
      std::swap(objects[i][3],objects[i][4]);
    }
    else if(fabs(angles::shortest_angular_distance(env_offset_yaw,M_PI)) < 0.1)
      theta = M_PI;
    else if(fabs(angles::shortest_angular_distance(env_offset_yaw,3.0*M_PI/2.0)) < 0.1){
      theta = 3.0*M_PI/2.0;
      std::swap(objects[i][3],objects[i][4]);
    }
    else{
      ROS_ERROR("Can only rotate environments by increments of 90 degrees!");
      assert(false);
    }
    //ROS_INFO("theta = %f",theta);

    double x1 = objects[i][0];
    double y1 = objects[i][1];
    //ROS_INFO("old x1=%f y1=%f",x1,y1);

    double new_x1 = cos(theta)*x1 - sin(theta)*y1;
    double new_y1 = sin(theta)*x1 + cos(theta)*y1;
    //ROS_INFO("new x1=%f y1=%f",new_x1,new_y1);

    objects[i][0] = new_x1 + env_offset_x;
    objects[i][1] = new_y1 + env_offset_y;
    objects[i][2] +=  env_offset_z;
    //ROS_INFO("pt=(%f %f) dim=(%f %f)",objects[i][0],objects[i][1],objects[i][3],objects[i][4]);
  }

  return getCollisionCubes(objects, object_ids, frame_id);
}

bool EGraphArmPlannerNode::getInitialConfiguration(ros::NodeHandle &nh, arm_navigation_msgs::RobotState &state, std::vector<double> angles, std::vector<string> names)
{
  XmlRpc::XmlRpcValue xlist;

  state.joint_state.name.clear();
  state.joint_state.position.clear();
  state.multi_dof_joint_state.frame_ids.clear();
  state.multi_dof_joint_state.child_frame_ids.clear();
  state.multi_dof_joint_state.poses.clear();

  for(unsigned int i = 0; i < angles.size(); ++i){
    state.joint_state.name.push_back(names[i]);
    state.joint_state.position.push_back(angles[i]);
  }

  double torso_z;
  ph.param("torso_z", torso_z, 0.0);
  
  //multi_dof_joint_state
  if(nh.hasParam("initial_configuration/multi_dof_joint_state"))
  {
    nh.getParam("initial_configuration/multi_dof_joint_state", xlist);

    if(xlist.getType() != XmlRpc::XmlRpcValue::TypeArray)
      ROS_WARN("initial_configuration/multi_dof_joint_state is not an array.");

    if(xlist.size() != 0)
    {
      geometry_msgs::Pose pose;
      for(int i = 0; i < xlist.size(); ++i)
      {
        state.multi_dof_joint_state.frame_ids.push_back(xlist[i]["frame_id"]);
        state.multi_dof_joint_state.child_frame_ids.push_back(xlist[i]["child_frame_id"]);
        pose.position.x = xlist[i]["x"];
        pose.position.y = xlist[i]["y"];
        pose.position.z = xlist[i]["z"];
        if(state.multi_dof_joint_state.child_frame_ids.back()=="torso_lift_link"){
          pose.position.z += torso_z;
          ROS_INFO("Adjust the multi dof joint state to include torso_z offset (%f)",torso_z);
        }
        leatherman::rpyToQuatMsg(xlist[i]["roll"], xlist[i]["pitch"], xlist[i]["yaw"], pose.orientation);
        state.multi_dof_joint_state.poses.push_back(pose);
      }
    }
  }
  return true;
}

EGraphArmPlannerNode::EGraphArmPlannerNode() : ph("~") {
  sleep(1);
  ros::spinOnce();
  ma_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 500);
  std::string group_name, kinematics_frame, planning_link, chain_tip_link;
  std::string object_filename, action_set_filename;
  ph.param<std::string>("kinematics_frame", kinematics_frame, "");
  ph.param<std::string>("planning_frame", planning_frame, "");
  ph.param<std::string>("planning_link", planning_link, "");
  ph.param<std::string>("chain_tip_link", chain_tip_link, "");
  ph.param<std::string>("group_name", group_name, "");
  ph.param<std::string>("object_filename", object_filename, "");
  ph.param<std::string>("action_set_filename", action_set_filename, "");

  // planning joints
  std::vector<std::string> planning_joints;
  XmlRpc::XmlRpcValue xlist;
  ph.getParam("planning/planning_joints", xlist);
  std::string joint_list = std::string(xlist);
  std::stringstream joint_name_stream(joint_list);
  while(joint_name_stream.good() && !joint_name_stream.eof())
  {
    std::string jname;
    joint_name_stream >> jname;
    if(jname.size() == 0)
      continue;
    planning_joints.push_back(jname);
  }
  start_angles.resize(planning_joints.size(),0);
  if(planning_joints.size() < 7)
    ROS_ERROR("Found %d planning joints on the param server. I usually expect at least 7 joints...", int(planning_joints.size()));

  // robot description
  std::string urdf;
  nh.param<std::string>("robot_description", urdf, " ");

  // distance field
  distance_field::PropagationDistanceField *df = new distance_field::PropagationDistanceField(1.7, 1.9, 1.6, 0.02, -0.70, -1.25, 0.0, 0.2);
  df->reset();

  // robot model
  rm = new sbpl_arm_planner::PR2KDLRobotModel();
  if(!rm->init(urdf, planning_joints))
  {
    ROS_ERROR("Failed to initialize robot model.");
    exit(1);
  }
  rm->setPlanningLink(planning_link);
  //ROS_WARN("Robot Model Information");
  //rm->printRobotModelInformation();
  //ROS_WARN(" ");

  KDL::Frame f;
  double torso_z;
  ph.param("torso_z", torso_z, 0.0);
  f.p.x(-0.05); f.p.y(0.0); f.p.z(0.802+torso_z);
  f.M = KDL::Rotation::Quaternion(0,0,0,1);
  rm->setKinematicsToPlanningTransform(f, planning_frame);

  // collision checker
  grid = new sbpl_arm_planner::OccupancyGrid(df);
  grid->setReferenceFrame(planning_frame);
  cc = new sbpl_arm_planner::SBPLCollisionSpace(grid);

  if(!cc->init(group_name))
    exit(1);
  if(!cc->setPlanningJoints(planning_joints))
    exit(1);
  cc->setJointPosition("torso_lift_joint", torso_z);
  cc->setJointPosition("l_shoulder_pan_joint", 1.7);
  cc->setJointPosition("l_shoulder_lift_joint", 1.2);

  // action set
  as = new sbpl_arm_planner::ActionSet(action_set_filename);

  // planner interface
  planner_ = new sbpl_arm_planner::EGraphSBPLArmPlannerInterface(rm, cc, as, df);

  if(!planner_->init())
    exit(1);

  planner_->setTorsoZ(torso_z);
  
  // collision objects
  scene.reset(new arm_navigation_msgs::PlanningScene);
  if(!object_filename.empty())
    scene->collision_objects = getCollisionObjects(object_filename, planning_frame);

  interrupt_sub_ = nh.subscribe("/sbpl_planning/interrupt", 1, &EGraphArmPlannerNode::interruptPlannerCallback,this);
  plan_service_ = nh.advertiseService("/sbpl_planning/plan_path",&EGraphArmPlannerNode::callPlanner,this);
  add_path_service_ = nh.advertiseService("/sbpl_planning/add_paths",&EGraphArmPlannerNode::addPathsToEGraph,this);
}

bool EGraphArmPlannerNode::addPathsToEGraph(egraph_arm::AddPathsToEGraph::Request& req, egraph_arm::AddPathsToEGraph::Response& res){
  if(req.clearEGraph)
    planner_->egraph_->clearEGraph();
  if(!req.paths.empty()){
    vector<vector<vector<double> > > paths;
    vector<vector<int> > costs;
    paths.resize(req.paths.size());
    costs.resize(req.paths.size());
    for(unsigned int i=0; i<req.paths.size(); i++){
      paths[i].resize(req.paths[i].points.size());
      costs[i].resize(req.paths[i].points.size()-1);
      for(unsigned int j=0; j<req.paths[i].points.size(); j++){
        paths[i][j].resize(req.paths[i].points[j].positions.size());
        int cost = 0;
        for(unsigned int k=0; k<req.paths[i].points[j].positions.size(); k++){
          paths[i][j][k] = req.paths[i].points[j].positions[k];
          if(j>0){
            double dang = fabs(angles::shortest_angular_distance(paths[i][j][k],paths[i][j-1][k]));
            cost += dang/(4*M_PI/180.0) * planner_->prm_->cost_multiplier_;
          }
        }
        if(j>0)
          costs[i][j-1] = cost;
      }
    }
    planner_->egraph_->addPaths(paths,costs);
  }
  return true;
}

void EGraphArmPlannerNode::interruptPlannerCallback(std_msgs::EmptyConstPtr){
  ROS_WARN("Planner interrupt received!");
  planner_->interrupt();
}

bool EGraphArmPlannerNode::callPlanner(egraph_arm::GetArmPlan::Request& req, egraph_arm::GetArmPlan::Response& res){
  // create goal
  arm_navigation_msgs::GetMotionPlan::Request arm_req;
  arm_navigation_msgs::GetMotionPlan::Response arm_res;
  scene->collision_map.header.frame_id = planning_frame;

  // fill goal state
  fillConstraint(req.goal, planning_frame, arm_req.motion_plan_request.goal_constraints);
  arm_req.motion_plan_request.allowed_planning_time.fromSec(5.0);

  // fill start state 
  if(!getInitialConfiguration(ph, scene->robot_state, req.start, req.names)){
    ROS_ERROR("Failed to get initial configuration.");
    return false;
  }
  scene->robot_state.joint_state.header.frame_id = planning_frame;  
  arm_req.motion_plan_request.start_state = scene->robot_state;

  // set planning scene
  //cc->setPlanningScene(*scene);

  EGraphReplanParams params(5.0);
  params.initial_eps = req.initial_eps;
  params.dec_eps = req.dec_eps;
  params.final_eps = req.final_eps;
  params.epsE = req.egraph_eps;
  params.dec_epsE = req.final_egraph_eps;
  params.final_epsE = req.dec_egraph_eps;
  params.return_first_solution = false;
  params.use_egraph = req.use_egraph;
  params.feedback_path = req.feedback_paths;
 
  // plan
  ROS_INFO("Calling solve...");
  if(!planner_->solve(scene, params, arm_req, arm_res)){
    ROS_ERROR("Failed to plan.");
  }
  else
    ma_pub.publish(planner_->getCollisionModelTrajectoryMarker());

  map<string,double> statMap = planner_->getEGraphStats();
  for(map<string,double>::iterator it=statMap.begin(); it!=statMap.end(); it++){
    res.stat_names.push_back(it->first);
    res.stat_values.push_back(it->second);
  }
  res.path = arm_res.trajectory.joint_trajectory;

  if(req.save_egraph)
    planner_->egraph_->save("arm_egraph.eg");

  // visualizations
  ros::spinOnce();
  ma_pub.publish(cc->getVisualization("bounds"));
  //ma_pub.publish(cc->getVisualization("distance_field"));
  ma_pub.publish(planner_->getVisualization("goal"));
  ma_pub.publish(planner_->getVisualization("expansions"));
  ma_pub.publish(cc->getVisualization("collision_objects"));
  //ma_pub.publish(grid->getVisualization("occupied_voxels"));
  ma_pub.publish(cc->getCollisionModelVisualization(start_angles));

  ROS_INFO("Done");
  return true;
}

int main(int argc, char **argv){
  ros::init (argc, argv, "egraph_arm_planner");
  EGraphArmPlannerNode planner;
  //ros::spin();
  ros::MultiThreadedSpinner spinner(2);//need 2 threads to catch a the interrupt
  spinner.spin();
}

