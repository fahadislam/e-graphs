#include <egraph_arm/interactArmPlanner.h>
#include <egraphs/egraph_stat_writer.h>

enum MenuItems{PLAN=1,PLAN_AND_FEEDBACK,PLAN_WITH_EGRAPHS,PLAN_WITH_EGRAPHS_AND_FEEDBACK,INTERRUPT,WRITE_TO_FILE};

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "control_planner");

  ControlPlanner cp;

  // start the ROS main loop
  ros::spin();
}

void ControlPlanner::callPlanner(){
  while(ros::ok()){
    boost::unique_lock<boost::mutex> lock(mutex);
    call_planner_cond.wait(lock);
    lock.unlock();
    planner.call(req,res);

    static bool first = true;
    EGraphStatWriter::writeStatsToFile("egraph_arm_stats.csv", first, res.stat_names, res.stat_values);
    first = false;
  }
}

void ControlPlanner::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT){
    if(feedback->menu_entry_id == MenuItems::PLAN ||
       feedback->menu_entry_id == MenuItems::PLAN_AND_FEEDBACK ||
       feedback->menu_entry_id == MenuItems::PLAN_WITH_EGRAPHS ||
       feedback->menu_entry_id == MenuItems::PLAN_WITH_EGRAPHS_AND_FEEDBACK){//plan

      // fill goal state
      visualization_msgs::InteractiveMarker goal_wrist_marker;
      int_marker_server->get("goal_wrist",goal_wrist_marker);
      
      req.goal = goal_wrist_marker.pose;
      vector<double> start = start_angles0;
      start.push_back(torso_z);
      req.start = start;
      req.names = {"r_shoulder_pan_joint","r_shoulder_lift_joint",
                   "r_upper_arm_roll_joint","r_elbow_flex_joint","r_forearm_roll_joint",
                   "r_wrist_flex_joint","r_wrist_roll_joint","torso_lift_link"};

      //egraph and planner parameters
      req.egraph_eps = 50.0;
      req.final_egraph_eps = 50.0;
      req.dec_egraph_eps = 1.0;
      req.initial_eps = 2.0;
      req.final_eps = 2.0;
      req.dec_eps = 0.2;
      if(feedback->menu_entry_id == MenuItems::PLAN_AND_FEEDBACK ||
         feedback->menu_entry_id == MenuItems::PLAN_WITH_EGRAPHS_AND_FEEDBACK){
        printf("plan and feed back path to egraph\n");
        req.feedback_paths = true;
        req.save_egraph = true;
      }
      else{
        printf("plan\n");
        req.feedback_paths = false;
        req.save_egraph = false;
      }
      if(feedback->menu_entry_id == MenuItems::PLAN_WITH_EGRAPHS ||
         feedback->menu_entry_id == MenuItems::PLAN_WITH_EGRAPHS_AND_FEEDBACK)
        req.use_egraph = true;
      else
        req.use_egraph = false;

      call_planner_cond.notify_one();
    }
    else if(feedback->menu_entry_id == MenuItems::INTERRUPT){//interrupt
      printf("interrupt planner\n");
      std_msgs::Empty msg;
      interrupt_pub.publish(msg);
    }
    else if(feedback->menu_entry_id == MenuItems::WRITE_TO_FILE){//interrupt
      printf("write to file\n");
      visualization_msgs::InteractiveMarker goal_wrist_marker;
      int_marker_server->get("goal_wrist",goal_wrist_marker);

      fout = fopen("arm_tests.yaml","a");
      fprintf(fout,"  - test: test_%d\n", test_num);
      fprintf(fout,"    start_angles: %f %f %f %f %f %f %f\n",
              start_angles0[0],
              start_angles0[1],
              start_angles0[2],
              start_angles0[3],
              start_angles0[4],
              start_angles0[5],
              start_angles0[6]);
      fprintf(fout,"    goal_angles: %f %f %f %f %f %f %f\n",
              goal_angles0[0],
              goal_angles0[1],
              goal_angles0[2],
              goal_angles0[3],
              goal_angles0[4],
              goal_angles0[5],
              goal_angles0[6]);
      fprintf(fout,"    torso_z: %f\n", torso_z);
      fprintf(fout,"    goal_xyz_wxyz: %f %f %f %f %f %f %f\n",
              goal_wrist_marker.pose.position.x,
              goal_wrist_marker.pose.position.y,
              goal_wrist_marker.pose.position.z,
              goal_wrist_marker.pose.orientation.w,
              goal_wrist_marker.pose.orientation.x,
              goal_wrist_marker.pose.orientation.y,
              goal_wrist_marker.pose.orientation.z);
      fprintf(fout,"\n");
      fclose(fout);
      test_num++;
    }
    else{
      ROS_ERROR("Invalid menu item");
    }
  }
  else{//movement
    bool is_start = feedback->marker_name == "start_wrist";

    //ik_pose needs to be in the base_footprint frame
    std::vector<double> ik_pose(7, 0);
    ik_pose[0] = feedback->pose.position.x;
    ik_pose[1] = feedback->pose.position.y;
    ik_pose[2] = feedback->pose.position.z;
    ik_pose[3] = feedback->pose.orientation.x;
    ik_pose[4] = feedback->pose.orientation.y;
    ik_pose[5] = feedback->pose.orientation.z;
    ik_pose[6] = feedback->pose.orientation.w;

    visualization_msgs::InteractiveMarker r_gripper_marker;
    string gripper_name( is_start ? "start_wrist" : "goal_wrist" );
    int_marker_server->get(gripper_name, r_gripper_marker);
    std::vector<double> solution(7, 0);
    vector<double>* angles = is_start ? &start_angles0 : &goal_angles0;
    if(kdl_robot_model_.computeIK(ik_pose, *angles, solution)){
      *angles = solution;
      vector<double> base(3,0);
      string ns( is_start ? "start" : "goal" );
      int color = is_start ? 85 : 0;
      pviz.visualizeRobot(*angles, angles1, base, torso_z, color, ns, 0, false);
      r_gripper_marker.controls[0].markers[0].color.r = 0;
      r_gripper_marker.controls[0].markers[0].color.g = 1;
      r_gripper_marker.controls[0].markers[0].color.b = 0;
    }
    else{
      r_gripper_marker.controls[0].markers[0].color.r = 1;
      r_gripper_marker.controls[0].markers[0].color.g = 0;
      r_gripper_marker.controls[0].markers[0].color.b = 0;
    }
    int_marker_server->insert(r_gripper_marker);
    int_marker_server->applyChanges();
  }
}

ControlPlanner::ControlPlanner(){
  fout = fopen("arm_tests.yaml","w");
  fprintf(fout, "experiments:\n\n");
  fclose(fout);
  test_num = 0;

  //initialize joint angles for the start and goal markers
  start_angles0.resize(7);
  start_angles0[0] = -0.002109;
  start_angles0[1] = 0.655300;
  start_angles0[2] = 0.000000;
  start_angles0[3] = -1.517650;
  start_angles0[4] = -3.138816;
  start_angles0[5] = -0.862352;
  start_angles0[6] = 3.139786;

  goal_angles0.resize(7);
  goal_angles0[0] = -0.002109;
  goal_angles0[1] = 0.655300;
  goal_angles0[2] = 0.000000;
  goal_angles0[3] = -1.517650;
  goal_angles0[4] = -3.138816;
  goal_angles0[5] = -0.862352;
  goal_angles0[6] = 3.139786;

  //torso_z = 0.3;
  ros::NodeHandle().param("/planner_node/torso_z", torso_z, 0.0);
  angles1.resize(7);
  angles1[0] = 0.2;
  angles1[1] = 1.4;
  angles1[2] = 1.9;
  angles1[3] = -0.4;
  angles1[4] = -0.1;
  angles1[5] = -1.00;
  angles1[6] = 0.0;

  //make kdl (used for FK and IK)
  string robot_description;
  ros::NodeHandle().param<std::string>("robot_description", robot_description, "");
  std::vector<std::string> planning_joints;
  planning_joints.push_back("r_shoulder_pan_joint");
  planning_joints.push_back("r_shoulder_lift_joint");
  planning_joints.push_back("r_upper_arm_roll_joint");
  planning_joints.push_back("r_elbow_flex_joint");
  planning_joints.push_back("r_forearm_roll_joint");
  planning_joints.push_back("r_wrist_flex_joint");
  planning_joints.push_back("r_wrist_roll_joint");
  if(!kdl_robot_model_.init(robot_description, planning_joints))
    ROS_ERROR("[PR2Sim] Failed to initialize the KDLRobotModel for the PR2.");
  kdl_robot_model_.setPlanningLink("r_gripper_palm_link");

  // Get the pose of the base footprint in the torso lift link frame.
  KDL::Frame base_in_torso_lift_link;
  base_in_torso_lift_link.p.x(0.050);
  base_in_torso_lift_link.p.y(0.0);
  base_in_torso_lift_link.p.z(-0.802 - torso_z);
  base_in_torso_lift_link.M = KDL::Rotation::Quaternion(0.0, 0.0, 0.0, 1.0);

  // Note that all computed poses of the end-effector are in the base footprint frame.
  kdl_robot_model_.setKinematicsToPlanningTransform(base_in_torso_lift_link.Inverse(),"base_footprint");

  int_marker_server = new interactive_markers::InteractiveMarkerServer("robot_marker");

  //make menu
  visualization_msgs::InteractiveMarkerControl menu_control;
  menu_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  menu_control.name = "menu_control";

  {//Make start hand marker
    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "/map";
    int_marker.name = "start_wrist";
    int_marker.description = "";
    int_marker.scale = 0.25;

    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.09;
    marker.scale.y = 0.09;
    marker.scale.z = 0.09;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // create a control which contains the box
    visualization_msgs::InteractiveMarkerControl trans_control;
    trans_control.always_visible = true;
    trans_control.markers.push_back(marker);
    trans_control.orientation.w = 1;
    trans_control.orientation.x = 0;
    trans_control.orientation.y = 1;
    trans_control.orientation.z = 0;
    trans_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(trans_control);

    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    int_marker.controls.push_back(menu_control);

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    int_marker_server->insert(int_marker, boost::bind(&ControlPlanner::processFeedback, this, _1));
  }
  {//Make goal hand marker
    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "/map";
    int_marker.name = "goal_wrist";
    int_marker.description = "";
    int_marker.scale = 0.25;

    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.09;
    marker.scale.y = 0.09;
    marker.scale.z = 0.09;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // create a control which contains the box
    visualization_msgs::InteractiveMarkerControl trans_control;
    trans_control.always_visible = true;
    trans_control.markers.push_back(marker);
    trans_control.orientation.w = 1;
    trans_control.orientation.x = 0;
    trans_control.orientation.y = 1;
    trans_control.orientation.z = 0;
    trans_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(trans_control);

    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    int_marker.controls.push_back(menu_control);

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    int_marker_server->insert(int_marker, boost::bind(&ControlPlanner::processFeedback, this, _1));
  }
  
  menu_handler.insert("Plan", boost::bind(&ControlPlanner::processFeedback, this, _1));
  menu_handler.insert("Plan and feedback", boost::bind(&ControlPlanner::processFeedback, this, _1));
  menu_handler.insert("Plan with E-Graphs", boost::bind(&ControlPlanner::processFeedback, this, _1));
  menu_handler.insert("Plan with E-Graphs and feedback", boost::bind(&ControlPlanner::processFeedback, this, _1));
  menu_handler.insert("Interrupt", boost::bind(&ControlPlanner::processFeedback, this, _1));
  menu_handler.insert("Write to file", boost::bind(&ControlPlanner::processFeedback, this, _1));
  menu_handler.apply(*int_marker_server, "start_wrist");
  menu_handler.apply(*int_marker_server, "goal_wrist");

  // 'commit' changes and send to all clients
  int_marker_server->applyChanges();

  moveMarkerToAngles("start_wrist", start_angles0);
  moveMarkerToAngles("goal_wrist", goal_angles0);

  planner_thread = new boost::thread(boost::bind(&ControlPlanner::callPlanner, this));
  planner = ros::NodeHandle().serviceClient<egraph_arm::GetArmPlan>("/sbpl_planning/plan_path", true);
  interrupt_pub = ros::NodeHandle().advertise<std_msgs::Empty>("/sbpl_planning/interrupt", 1);
}

void ControlPlanner::moveMarkerToAngles(string marker_name, vector<double> angles){
  //snap the interative gripper marker back on the pr2 in the last valid pose
  visualization_msgs::InteractiveMarker gripper_marker;
  int_marker_server->get(marker_name, gripper_marker);
  gripper_marker.controls[0].markers[0].color.r = 0;
  gripper_marker.controls[0].markers[0].color.g = 1;
  gripper_marker.controls[0].markers[0].color.b = 0;

  std::vector<double> fk_pose;
  kdl_robot_model_.computePlanningLinkFK(angles, fk_pose);

  gripper_marker.pose.position.x = fk_pose[0];
  gripper_marker.pose.position.y = fk_pose[1];
  gripper_marker.pose.position.z = fk_pose[2];
  tf::Quaternion q_hand(fk_pose[5],fk_pose[4],fk_pose[3]);
  gripper_marker.pose.orientation.w = q_hand.getW();
  gripper_marker.pose.orientation.x = q_hand.getX();
  gripper_marker.pose.orientation.y = q_hand.getY();
  gripper_marker.pose.orientation.z = q_hand.getZ();

  int_marker_server->insert(gripper_marker);
  int_marker_server->applyChanges();
}

ControlPlanner::~ControlPlanner(){

}


