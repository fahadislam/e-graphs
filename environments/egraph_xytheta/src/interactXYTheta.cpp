#include <egraph_xytheta/interactXYTheta.h>
#include <egraphs/egraph_stat_writer.h>

enum MenuItems{PLAN=1,PLAN_AND_FEEDBACK,PLAN_WITH_EGRAPHS,PLAN_WITH_EGRAPHS_AND_FEEDBACK,INTERRUPT,WRITE_TO_FILE};

void ControlPlanner::callPlanner(){
  while(ros::ok()){
    boost::unique_lock<boost::mutex> lock(mutex);
    call_planner_cond.wait(lock);
    lock.unlock();
    planner.call(req,res);

    static bool first = true;
    EGraphStatWriter::writeStatsToFile("egraph_xytheta_stats.csv", first, res.stat_names, res.stat_values);
    first = false;
  }
}

void ControlPlanner::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT){
    if(feedback->menu_entry_id == MenuItems::PLAN ||
        feedback->menu_entry_id == MenuItems::PLAN_AND_FEEDBACK ||
        feedback->menu_entry_id == MenuItems::PLAN_WITH_EGRAPHS ||
        feedback->menu_entry_id == MenuItems::PLAN_WITH_EGRAPHS_AND_FEEDBACK){//plan

      visualization_msgs::InteractiveMarker start_marker;
      server->get("robot start",start_marker);
      visualization_msgs::InteractiveMarker goal_marker;
      server->get("robot goal",goal_marker);

      req.start_x = start_marker.pose.position.x;
      req.start_y = start_marker.pose.position.y;
      req.start_theta = 2 * atan2(start_marker.pose.orientation.z, start_marker.pose.orientation.w);

      req.goal_x = goal_marker.pose.position.x;
      req.goal_y = goal_marker.pose.position.y;
      req.goal_theta = 2 * atan2(goal_marker.pose.orientation.z, goal_marker.pose.orientation.w);
      printf("(%f %f %f) -> (%f %f %f)\n",req.start_x, req.start_y, req.start_theta, req.goal_x, req.goal_y, req.goal_theta);

      //egraph and planner parameters
      req.egraph_eps = 5.0;
      req.final_egraph_eps = 5.0;
      req.dec_egraph_eps = 1.0;
      req.initial_eps = 2.0;
      req.final_eps = 2.0;
      req.dec_eps = 0.2;
      if(feedback->menu_entry_id == MenuItems::PLAN_AND_FEEDBACK ||
          feedback->menu_entry_id == MenuItems::PLAN_WITH_EGRAPHS_AND_FEEDBACK){
        printf("plan and feed back path to egraph\n");
        req.feedback_path = true;
        req.save_egraph = true;
      }
      else{
        printf("plan\n");
        req.feedback_path = false;
        req.save_egraph = false;
      }
      if(feedback->menu_entry_id == MenuItems::PLAN_WITH_EGRAPHS ||
         feedback->menu_entry_id == MenuItems::PLAN_WITH_EGRAPHS_AND_FEEDBACK)
        req.use_egraph = true;
      else
        req.use_egraph = false;

      call_planner_cond.notify_one();
    }
    else if(feedback->menu_entry_id == MenuItems::INTERRUPT){
      printf("interrupt planner\n");
      std_msgs::Empty msg;
      interrupt_pub.publish(msg);
    }
    else if(feedback->menu_entry_id == MenuItems::WRITE_TO_FILE){
      printf("write to file\n");
      visualization_msgs::InteractiveMarker start_marker;
      server->get("robot start",start_marker);
      visualization_msgs::InteractiveMarker goal_marker;
      server->get("robot goal",goal_marker);

      fout = fopen("xytheta_tests.yaml","a");
      fprintf(fout,"  - test: test_%d\n", test_num);
      fprintf(fout,"    start: %f %f %f\n", start_marker.pose.position.x, start_marker.pose.position.y,
                  2 * atan2(start_marker.pose.orientation.z, start_marker.pose.orientation.w));
      fprintf(fout,"    goal: %f %f %f\n", goal_marker.pose.position.x, goal_marker.pose.position.y,
                  2 * atan2(goal_marker.pose.orientation.z, goal_marker.pose.orientation.w));
      fprintf(fout,"\n");
      fclose(fout);
      test_num++;

    }
    else{
      ROS_ERROR("Invalid menu item");
    }
  }
  else{
    //draw arrow
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = feedback->marker_name;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.02;
    marker.scale.y = 0.04;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    geometry_msgs::Point p;
    p.x = feedback->pose.position.x;
    p.y = feedback->pose.position.y;
    p.z = 0.1;
    marker.points.push_back(p);
    double r = radius_;
    double theta = 2 * atan2(feedback->pose.orientation.z, feedback->pose.orientation.w) - M_PI/4;//not sure why i need this offset...
    p.x += r*cos(theta) - r*sin(theta);
    p.y += r*sin(theta) + r*cos(theta);
    marker.points.push_back(p);
    vis_pub_.publish(marker);

    //collision check start and goal
    visualization_msgs::InteractiveMarker start_marker;
    server->get("robot start",start_marker);
    visualization_msgs::InteractiveMarker goal_marker;
    server->get("robot goal",goal_marker);
    costmap_2d::Costmap2D cost_map;
    planner_costmap_ros_->getCostmapCopy(cost_map);

    double start_x = start_marker.pose.position.x;
    double start_y = start_marker.pose.position.y;
    unsigned int sx,sy;
    bool inBounds = cost_map.worldToMap(start_x, start_y, sx, sy);
    if(!inBounds)
      printf("Start out of bounds!\n");
    else{
      unsigned char cost = cost_map.getCost(sx,sy);
      if(cost >= cost_map.getCircumscribedCost())
        printf("Start in collision\n");
    }

    double goal_x = goal_marker.pose.position.x;
    double goal_y = goal_marker.pose.position.y;
    unsigned int gx,gy;
    inBounds = cost_map.worldToMap(goal_x, goal_y, gx, gy);
    if(!inBounds)
      printf("Goal out of bounds!\n");
    else{
      unsigned char cost = cost_map.getCost(gx,gy);
      if(cost >= cost_map.getCircumscribedCost())
        printf("Goal in collision\n");
    }
  }
}

ControlPlanner::ControlPlanner(){
  fout = fopen("xytheta_tests.yaml","w");
  fprintf(fout, "experiments:\n\n");
  fclose(fout);
  test_num = 0;

  planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
  planner_costmap_ros_->pause();
  std::vector<geometry_msgs::Point> footprint = planner_costmap_ros_->getRobotFootprint();
  double max_radius = 0;
  for (unsigned int i=0; i<footprint.size(); i++){
    double r = sqrt(footprint[i].x*footprint[i].x + footprint[i].y*footprint[i].y);
    if(r > max_radius)
      max_radius = r;
  }
  radius_ = max_radius;

  server = new interactive_markers::InteractiveMarkerServer("robot_marker");

  {
    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "/map";
    int_marker.name = "robot start";
    int_marker.description = "robot start";

    // create a grey box marker
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.scale.x = max_radius*2;
    marker.scale.y = max_radius*2;
    marker.scale.z = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // create a control which contains the box
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(marker);
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(control);

    visualization_msgs::InteractiveMarkerControl control2;
    control2.orientation.w = 1;
    control2.orientation.x = 0;
    control2.orientation.y = 1;
    control2.orientation.z = 0;
    control2.name = "rotate_z";
    control2.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control2);

    visualization_msgs::InteractiveMarkerControl menu_control;
    menu_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
    menu_control.name = "menu_control";
    int_marker.controls.push_back(menu_control);

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    server->insert(int_marker, boost::bind(&ControlPlanner::processFeedback, this, _1));
  }
  {
    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "/map";
    int_marker.name = "robot goal";
    int_marker.description = "robot goal";

    // create a grey box marker
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.scale.x = max_radius*2;
    marker.scale.y = max_radius*2;
    marker.scale.z = 0.1;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // create a control which contains the box
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(marker);
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(control);

    visualization_msgs::InteractiveMarkerControl control2;
    control2.orientation.w = 1;
    control2.orientation.x = 0;
    control2.orientation.y = 1;
    control2.orientation.z = 0;
    control2.name = "rotate_z";
    control2.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control2);

    visualization_msgs::InteractiveMarkerControl menu_control;
    menu_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
    menu_control.name = "menu_control";
    int_marker.controls.push_back(menu_control);

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    server->insert(int_marker, boost::bind(&ControlPlanner::processFeedback, this, _1));
  }

  menu_handler.insert("Plan", boost::bind(&ControlPlanner::processFeedback, this, _1));
  menu_handler.insert("Plan and feedback", boost::bind(&ControlPlanner::processFeedback, this, _1));
  menu_handler.insert("Plan with E-Graphs", boost::bind(&ControlPlanner::processFeedback, this, _1));
  menu_handler.insert("Plan with E-Graphs and feedback", boost::bind(&ControlPlanner::processFeedback, this, _1));
  menu_handler.insert("Interrupt", boost::bind(&ControlPlanner::processFeedback, this, _1));
  menu_handler.insert("Write to file", boost::bind(&ControlPlanner::processFeedback, this, _1));
  menu_handler.apply(*server, "robot start");
  menu_handler.apply(*server, "robot goal");

  // 'commit' changes and send to all clients
  server->applyChanges();

  planner_thread = new boost::thread(boost::bind(&ControlPlanner::callPlanner, this));
  planner = ros::NodeHandle().serviceClient<egraph_xytheta::GetXYThetaPlan>("/sbpl_planning/plan_path", true);
  interrupt_pub = ros::NodeHandle().advertise<std_msgs::Empty>("/sbpl_planning/interrupt", 1);
  vis_pub_ = ros::NodeHandle().advertise<visualization_msgs::Marker>("visualization_marker",1);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "interact_xy_planner");

  ControlPlanner cp;

  ros::spin();
}

