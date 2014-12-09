#include <ros/ros.h>
#include <kdl/frames.hpp>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <monolithic_pr2_planner_node/GetMobileArmPlan.h>
#include <monolithic_pr2_planner/Constants.h>
#include <angles/angles.h>
#include <boost/date_time/posix_time/posix_time.hpp>

monolithic_pr2_planner_node::GetMobileArmPlan generate_request(double base_x,
                                                               double base_y,
                                                               double base_theta,
                                                               double goal_x,
                                                               double goal_y,
                                                               double goal_z,
                                                               double goal_roll,
                                                               double goal_pitch,
                                                               double goal_yaw,
                                                               double init_eps
                                                               ){


    monolithic_pr2_planner_node::GetMobileArmPlan srv;
    std::vector<double> right_arm_start(7,0), left_arm_start(7,0), body_start(4,0);
    right_arm_start[0] = -0.034127;
    right_arm_start[1] = 1.09261;
    right_arm_start[2] = 0.000000;
    right_arm_start[3] = -1.614009;
    right_arm_start[4] = 2.987015;
    right_arm_start[5] = -1.413143;
    right_arm_start[6] = 2.889659;

    left_arm_start[0] = 0.038946;
    left_arm_start[1] = 1.214670;
    left_arm_start[2] = 1.396356;
    left_arm_start[3] = -1.197227;
    left_arm_start[4] = -4.616317;
    left_arm_start[5] = -0.988727;
    left_arm_start[6] = 1.175568;

    body_start[0] = base_x;
    body_start[1] = base_y;
    body_start[2] = 0.260000;
    body_start[3] = base_theta; 

    srv.request.rarm_start = right_arm_start;
    srv.request.larm_start = left_arm_start;
    srv.request.body_start = body_start;

    // goal orientation in RPY
    KDL::Rotation rot = KDL::Rotation::RPY(goal_roll, goal_pitch, goal_yaw);
    double qx, qy, qz, qw;
    rot.GetQuaternion(qx, qy, qz, qw);

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = goal_x;
    pose.pose.position.y = goal_y;
    pose.pose.position.z = goal_z;

    pose.pose.orientation.x = qx;
    pose.pose.orientation.y = qy;
    pose.pose.orientation.z = qz;
    pose.pose.orientation.w = qw;

    geometry_msgs::PoseStamped rarm_offset;
    rarm_offset.pose.position.x = 0;
    rarm_offset.pose.position.y = 0;
    rarm_offset.pose.orientation.z = 0;
    rarm_offset.pose.orientation.x = 0;
    rarm_offset.pose.orientation.y = 0;
    rarm_offset.pose.orientation.z = 0;
    rarm_offset.pose.orientation.w = 1;

    geometry_msgs::PoseStamped larm_offset;
    larm_offset.pose.position.x = 0;
    larm_offset.pose.position.y = 0;
    larm_offset.pose.orientation.z = 0;
    larm_offset.pose.orientation.x = 0;
    larm_offset.pose.orientation.y = 0;
    larm_offset.pose.orientation.z = 0;
    larm_offset.pose.orientation.w = 1;
    srv.request.rarm_object = rarm_offset;
    srv.request.larm_object = larm_offset;

    srv.request.goal = pose;
    srv.request.initial_eps = init_eps;
    srv.request.final_eps = init_eps;
    srv.request.dec_eps = .1;
    srv.request.egraph_eps = 10;
    srv.request.use_egraph = true;
    srv.request.feedback_path = true;
    srv.request.xyz_tolerance = .04;
    srv.request.roll_tolerance = .1;
    srv.request.pitch_tolerance = .1;
    srv.request.yaw_tolerance = .1;

    srv.request.allocated_planning_time = 30;

    srv.request.planning_mode = monolithic_pr2_planner::PlanningModes::RIGHT_ARM_MOBILE;
    return srv;
}

bool make_request(monolithic_pr2_planner_node::GetMobileArmPlan srv){
    static ros::NodeHandle n;
    static ros::ServiceClient client = n.serviceClient<monolithic_pr2_planner_node::GetMobileArmPlan>("/sbpl_planning/plan_path");
    ROS_INFO("Sending request at : %s",
        boost::posix_time::to_simple_string(boost::posix_time::microsec_clock::local_time()).c_str());
    if (client.call(srv))
    {
        ROS_INFO("called service");
        for (size_t i=0; i < srv.response.stats_field_names.size(); i++){
            ROS_INFO("%s: %f", srv.response.stats_field_names[i].c_str(),
                               srv.response.stats[i]);
        }
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return false;
    }
    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "testPlanningRequest");
    //make_request(generate_request(1,1,-M_PI/2, 8.0, 5, 1, 0, 0, M_PI/2, 10));
    make_request(generate_request(5,1,-M_PI/2, 5, 4, 1, 0, 0, M_PI/2, 100));
    //make_request(generate_request(4.8,1,-M_PI/2, 4.2, 4, 1, 0, 0, M_PI/2, 2));
    //make_request(generate_request(5.2,1,-M_PI/2, 6, 4, 1, 0, 0, M_PI/2, 2));

    //make_request(generate_request(4.7,1,-M_PI/2, 3, 4, 1, 0, 0, M_PI/2, 2));
    
    //make_request(generate_request(5.5,1,-M_PI/2, 5, 4, 1, 0, 0, M_PI/2, 10));
    //make_request(generate_request(4.4,1,-M_PI/2, 4.2, 4, 1, 0, 0, M_PI/2, 2));
    make_request(generate_request(5.9,1,-M_PI/2, 6, 4, 1, 0, 0, M_PI/2, 2));
    //make_request(generate_request(4.1,1,-M_PI/2, 3, 4, 1, 0, 0, M_PI/2, 2));
    return 0;
}
