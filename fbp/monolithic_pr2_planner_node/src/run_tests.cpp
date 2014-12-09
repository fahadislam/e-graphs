#include<ros/ros.h>
#include<monolithic_pr2_planner_node/GetMobileArmPlan.h>
#include <monolithic_pr2_planner/Constants.h>
#include <egraphs/egraph_stat_writer.h>

void printUsage(){
  printf("usage: runTests [naive | kd | prm] [fb] path_to_test_file.yaml\n");
}

int main(int argc, char** argv){
  if(argc != 3 && argc != 4){
    printUsage();
    return 1;
  }
  ros::init(argc,argv,"run_tests");
  ros::NodeHandle nh;

  monolithic_pr2_planner_node::GetMobileArmPlan::Request req;
  monolithic_pr2_planner_node::GetMobileArmPlan::Response res;

  bool gotFilename = false;
  bool gotPlannerType = false;
  req.use_ompl = false;
  bool feedback = false;
  char* filename;
  for(int i=1; i<argc; i++){
    if(strcmp(argv[i],"prm")==0){
      req.use_ompl = true;
      gotPlannerType = true;
    }
    else if(strcmp(argv[i],"kd")==0){
      req.use_kd_heuristic = true;
      gotPlannerType = true;
    }
    else if(strcmp(argv[i],"naive")==0){
      req.use_kd_heuristic = false;
      gotPlannerType = true;
    }
    else if(strcmp(argv[i],"fb")==0){
      feedback = true;
    }
    else{
      filename = argv[i];
      gotFilename = true;
    }
  }

  if(!gotFilename || !gotPlannerType){
    printUsage();
    return 1;
  }

  //egraph and planner parameters
  req.egraph_eps = 10.0;
  req.final_egraph_eps = 10.0;
  req.dec_egraph_eps = 1.0;
  req.initial_eps = 2.0;
  req.final_eps = 2.0;
  req.dec_eps = 0.2;
  if(feedback){
    req.feedback_path = true;
    req.save_egraph = true;
  }
  req.use_egraph = true;

  req.rarm_object.pose.position.x = 0;
  req.rarm_object.pose.position.y = 0;
  req.rarm_object.pose.position.z = 0;
  req.rarm_object.pose.orientation.x = 0;
  req.rarm_object.pose.orientation.y = 0;
  req.rarm_object.pose.orientation.z = 0;
  req.rarm_object.pose.orientation.w = 1;
  req.larm_object.pose.position.x = 0;
  req.larm_object.pose.position.y = 0;
  req.larm_object.pose.position.z = 0;
  req.larm_object.pose.orientation.x = 0;
  req.larm_object.pose.orientation.y = 0;
  req.larm_object.pose.orientation.z = 0;
  req.larm_object.pose.orientation.w = 1;

  req.xyz_tolerance = .04;
  req.roll_tolerance = .1;
  req.pitch_tolerance = .1;
  req.yaw_tolerance = .1;
  req.allocated_planning_time = 30;
  req.planning_mode = monolithic_pr2_planner::PlanningModes::RIGHT_ARM_MOBILE;

  req.body_start.resize(4);
  req.rarm_start.resize(7);
  req.larm_start.resize(7);
  req.body_goal.resize(4);
  req.rarm_goal.resize(7);
  req.larm_goal.resize(7);


  ros::service::waitForService("/sbpl_planning/plan_path",10);
  ros::ServiceClient planner = ros::NodeHandle().serviceClient<monolithic_pr2_planner_node::GetMobileArmPlan>("/sbpl_planning/plan_path", true);
  sleep(1);

  FILE* fin = fopen(filename,"r");
  if(!fin){
    printf("file %s does not exist\n", argv[1]);
    return 1;
  }
  fscanf(fin,"experiments:\n\n");

  bool first = true;
  while(1){
    int test_num = 0;
    if(fscanf(fin,"  - test: test_%d\n    start:\n", &test_num) <= 0)
      break;

    if(fscanf(fin,"      object_xyz_wxyz: %lf %lf %lf %lf %lf %lf %lf\n",
              &req.start.pose.position.x,&req.start.pose.position.y,&req.start.pose.position.z,
              &req.start.pose.orientation.w,&req.start.pose.orientation.x,&req.start.pose.orientation.y,&req.start.pose.orientation.z) <= 0)
      break;
    if(fscanf(fin,"      base_xyzyaw: %lf %lf %lf %lf\n",
              &req.body_start[0], &req.body_start[1],
              &req.body_start[2], &req.body_start[3]) <= 0)
      break;
    if(fscanf(fin,"      rarm: %lf %lf %lf %lf %lf %lf %lf\n",
              &req.rarm_start[0],&req.rarm_start[1],
              &req.rarm_start[2],&req.rarm_start[3],
              &req.rarm_start[4],&req.rarm_start[5],
              &req.rarm_start[6]) <= 0)
      break;
    if(fscanf(fin,"      larm: %lf %lf %lf %lf %lf %lf %lf\n    goal:\n",
              &req.larm_start[0],&req.larm_start[1],
              &req.larm_start[2],&req.larm_start[3],
              &req.larm_start[4],&req.larm_start[5],
              &req.larm_start[6]) <= 0)
      break;
    if(fscanf(fin,"      object_xyz_wxyz: %lf %lf %lf %lf %lf %lf %lf\n",
              &req.goal.pose.position.x,&req.goal.pose.position.y,&req.goal.pose.position.z,
              &req.goal.pose.orientation.w,&req.goal.pose.orientation.x,&req.goal.pose.orientation.y,&req.goal.pose.orientation.z) <= 0)
      break;
    if(fscanf(fin,"      base_xyzyaw: %lf %lf %lf %lf\n",
              &req.body_goal[0], &req.body_goal[1],
              &req.body_goal[2], &req.body_goal[3]) <= 0)
      break;
    if(fscanf(fin,"      rarm: %lf %lf %lf %lf %lf %lf %lf\n",
              &req.rarm_goal[0],&req.rarm_goal[1],
              &req.rarm_goal[2],&req.rarm_goal[3],
              &req.rarm_goal[4],&req.rarm_goal[5],
              &req.rarm_goal[6]) <= 0)
      break;
    if(fscanf(fin,"      larm: %lf %lf %lf %lf %lf %lf %lf\n\n",
              &req.larm_goal[0],&req.larm_goal[1],
              &req.larm_goal[2],&req.larm_goal[3],
              &req.larm_goal[4],&req.larm_goal[5],
              &req.larm_goal[6]) <= 0)
      break;

    if(test_num<1)
      continue;

    printf("Running test %d\n",test_num);
    planner.call(req,res);
    EGraphStatWriter::writeStatsToFile("egraph_fbp_stats.csv", first, res.stat_names, res.stat_values);
    first = false;
  }

  return 0;
}

