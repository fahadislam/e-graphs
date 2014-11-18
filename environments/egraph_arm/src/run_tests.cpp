#include<ros/ros.h>
#include<egraph_arm/GetArmPlan.h>
#include<vector>
#include <egraphs/egraph_stat_writer.h>

int main(int argc, char** argv){
  if(argc < 2){
    printf("provide a path to a test file!\n");
    return 1;
  }
  ros::init(argc,argv,"run_tests");
  ros::NodeHandle nh;

  egraph_arm::GetArmPlan::Request req;
  egraph_arm::GetArmPlan::Response res;

  //egraph and planner parameters
  req.egraph_eps = 50.0;
  req.final_egraph_eps = 50.0;
  req.dec_egraph_eps = 1.0;
  req.initial_eps = 2.0;
  req.final_eps = 2.0;
  req.dec_eps = 0.2;
  req.feedback_paths = true;
  req.save_egraph = true;
  req.use_egraph = true;

  req.start.resize(8);
  req.names = {"r_shoulder_pan_joint","r_shoulder_lift_joint",
               "r_upper_arm_roll_joint","r_elbow_flex_joint","r_forearm_roll_joint",
               "r_wrist_flex_joint","r_wrist_roll_joint","torso_lift_link"};

  ros::service::waitForService("/sbpl_planning/plan_path",10);
  ros::ServiceClient planner = ros::NodeHandle().serviceClient<egraph_arm::GetArmPlan>("/sbpl_planning/plan_path", true);
  sleep(1);

  std::vector<double> unused_goal_angles(7,0);

  FILE* fin = fopen(argv[1],"r");
  if(!fin){
    printf("file %s does not exist\n", argv[1]);
    return 1;
  }
  fscanf(fin,"experiments:\n\n");

  bool first = true;
  while(1){
    int test_num = 0;
    if(fscanf(fin,"  - test: test_%d\n", &test_num) <= 0)
      break;
    if(fscanf(fin,"    start_angles: %lf %lf %lf %lf %lf %lf %lf\n",
              &req.start[0],
              &req.start[1],
              &req.start[2],
              &req.start[3],
              &req.start[4],
              &req.start[5],
              &req.start[6]) <= 0)
      break;
    if(fscanf(fin,"    goal_angles: %lf %lf %lf %lf %lf %lf %lf\n",
              &unused_goal_angles[0],
              &unused_goal_angles[1],
              &unused_goal_angles[2],
              &unused_goal_angles[3],
              &unused_goal_angles[4],
              &unused_goal_angles[5],
              &unused_goal_angles[6]) <= 0)
      break;
    if(fscanf(fin,"    torso_z: %lf\n", &req.start[7]) <= 0)
      break;
    if(fscanf(fin,"    goal_xyz_wxyz: %lf %lf %lf %lf %lf %lf %lf\n\n",
              &req.goal.position.x,
              &req.goal.position.y,
              &req.goal.position.z,
              &req.goal.orientation.w,
              &req.goal.orientation.x,
              &req.goal.orientation.y,
              &req.goal.orientation.z) <= 0)
      break;

    planner.call(req,res);

    EGraphStatWriter::writeStatsToFile("egraph_arm_stats.csv", first, res.stat_names, res.stat_values);
    first = false;
  }

  return 0;
}

