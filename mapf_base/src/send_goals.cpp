#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"
#include "mapf_msgs/GlobalPlan.h"
#include "mapf_msgs/SinglePlan.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"


#include <sstream>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void sub_callback(const mapf_msgs::GlobalPlan &msg_plan) {
    ROS_INFO( "Get New plan..");
    ROS_INFO( "MakeSpan: %d", msg_plan.makespan);
    ROS_INFO( "AgentNum: 2");
    // mapf_msgs/GlobalPlan
    // Create goals
    // Create action client
    MoveBaseClient ac_0("/rb_0/move_base", true);
    while(!ac_0.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the rb_0/move_base/goal action server to come up");
    }

    MoveBaseClient ac_1("/rb_1/move_base", true);
    while(!ac_1.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the rb_1/move_base/goal action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal_0, goal_1;
    
    
    for (int i = 0; i < msg_plan.makespan; i++) {
        goal_0.target_pose.header.frame_id = "map";
        goal_0.target_pose.header.stamp = ros::Time::now();
        goal_0.target_pose.pose = msg_plan.global_plan[0].plan.poses[i].pose;

        goal_1.target_pose.header.frame_id = "map";
        goal_1.target_pose.header.stamp = ros::Time::now();
        goal_1.target_pose.pose = msg_plan.global_plan[1].plan.poses[i].pose;

        
        // Send goal 0
        ROS_INFO("===== Sending Goal 0 ===== makespan %d", i);
        ac_0.sendGoal(goal_0);
        if (ac_0.waitForResult() == true) {
            ROS_INFO("===== Goal 0 Arrived ===== makespan %d", i);
        }
        
        // Send goal 1
        ROS_INFO("===== Sending Goal 1 ===== makespan %d", i);
        ac_1.sendGoal(goal_1);
        if (ac_1.waitForResult() == true) {
            ROS_INFO("===== Goal 1 Arrived ===== makespan %d", i);
        }
        
    }
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "send_goal_client");

    ros::NodeHandle nh;
    ros::Rate rate(1);
    ros::Duration timesleep(5);

    // publish init mapf goal for each agent
    ros::Publisher pub_g0 = nh.advertise<geometry_msgs::PoseStamped>("/mapf_base/rb_0/goal", 1000);
    geometry_msgs::PoseStamped goal_0;
    goal_0.header.frame_id = "map";
    goal_0.pose.position.x = 4.0;
    goal_0.pose.position.y = 0.0;
    goal_0.pose.orientation.w = 1;
    ros::Publisher pub_g1 = nh.advertise<geometry_msgs::PoseStamped>("/mapf_base/rb_1/goal", 1000);
    geometry_msgs::PoseStamped goal_1;
    goal_1.header.frame_id = "map";
    goal_1.pose.position.x = 8.0;
    goal_1.pose.position.y = 0.0;
    goal_1.pose.orientation.w = 1;

    while (pub_g0.getNumSubscribers() < 1) {
        rate.sleep();
        ROS_INFO("Wait for /mapf_base/rb_0/goal subscriber");
    }
    while (pub_g1.getNumSubscribers() < 1) {
        rate.sleep();
        ROS_INFO("Wait for /mapf_base/rb_1/goal subscriber");
    }
    pub_g0.publish(goal_0);
    pub_g1.publish(goal_1);
    timesleep.sleep();
    timesleep.sleep();
    pub_g0.shutdown();
    pub_g1.shutdown();

    // publish goal transform flag
    ros::Publisher pub_flag= nh.advertise<std_msgs::Bool>("/mapf_base/goal_init_flag", 1000);
    std_msgs::Bool flag;
    flag.data = true;
    while (pub_flag.getNumSubscribers() < 1) {
        rate.sleep();
        ROS_INFO("Wait for /mapf_base/goal_init_flag subscriber");
    }
    pub_flag.publish(flag);
    timesleep.sleep();
    pub_flag.shutdown();

    



    ros::Subscriber sub = nh.subscribe("/mapf_base/global_plan", 1000, sub_callback);

    ros::spin();

    return 0;
}