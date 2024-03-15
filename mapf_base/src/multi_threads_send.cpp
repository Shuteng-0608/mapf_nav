#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"
#include "mapf_msgs/GlobalPlan.h"
#include "mapf_msgs/SinglePlan.h"
#include "nav_msgs/Path.h"
#include <boost/thread.hpp>
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"
 
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void sendGoal(const move_base_msgs::MoveBaseGoal& goal, MoveBaseClient& ac, int rb_name) {
    
    ac.sendGoal(goal);
    ROS_INFO("[Agent %d] Goal (%.2f, %.2f) Received",
            rb_name,
            goal.target_pose.pose.position.x, 
            goal.target_pose.pose.position.y );
    ac.waitForResult();
    ROS_INFO("[Agent %d] Goal (%.2f, %.2f) Arrived",
            rb_name,
            goal.target_pose.pose.position.x, 
            goal.target_pose.pose.position.y );

}


 
void sub_callback(const mapf_msgs::GlobalPlan &msg_plan) {
    // mapf_msgs/GlobalPlan
    // Create action clients
    MoveBaseClient ac_0("/rb_0/move_base", true);
    while(!ac_0.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the rb_0/move_base/goal action server to come up");
    }
 
    MoveBaseClient ac_1("/rb_1/move_base", true);
    while(!ac_1.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the rb_1/move_base/goal action server to come up");
    }
 
    // Create goals
    move_base_msgs::MoveBaseGoal goal_0, goal_1;
    goal_0.target_pose.header.frame_id = "map";
    goal_1.target_pose.header.frame_id = "map";
    ROS_INFO("===== Makespan : %d =====", msg_plan.makespan);
    for (int i = 0; i < msg_plan.makespan; i++) {
        int rb_0 = 0;
        if (i < msg_plan.global_plan[0].time_step.size()){
            goal_0.target_pose.header.stamp = ros::Time::now();
            goal_0.target_pose.pose.position.x = msg_plan.global_plan[0].plan.poses[i].pose.position.x;
            goal_0.target_pose.pose.position.y = msg_plan.global_plan[0].plan.poses[i].pose.position.y;
            goal_0.target_pose.pose.orientation.w = 1;
        }
        
        int rb_1 = 1;
        if (i < msg_plan.global_plan[1].time_step.size()) {
            goal_1.target_pose.header.stamp = ros::Time::now();
            goal_1.target_pose.pose.position.x = msg_plan.global_plan[1].plan.poses[i].pose.position.x;
            goal_1.target_pose.pose.position.y = msg_plan.global_plan[1].plan.poses[i].pose.position.y;
            goal_1.target_pose.pose.orientation.w = 1;
        }
        
 
        ROS_INFO("[=== At Time Stamp %d ===]", i);
        // Create threads to send goals for each robot
        boost::thread thread_0(boost::bind(&sendGoal, goal_0, boost::ref(ac_0), rb_0));
        boost::thread thread_1(boost::bind(&sendGoal, goal_1, boost::ref(ac_1), rb_1));
 
        // Join threads
        thread_0.join();
        thread_1.join();
    }
    ROS_INFO("==== MAPF Navigation Finished Successfully ====");

}
 
int main(int argc, char** argv) {
    ros::init(argc, argv, "send_goal_client");
 
    ros::NodeHandle nh;

    ros::Rate rate(1);
    ros::Duration timesleep(1);

    // publish init mapf goal for each agent
    ros::Publisher pub_g0 = nh.advertise<geometry_msgs::PoseStamped>("/mapf_base/rb_0/goal", 1);
    geometry_msgs::PoseStamped goal_0;
    goal_0.header.frame_id = "map";
    goal_0.pose.position.x = 8.0;
    goal_0.pose.position.y = 0.0;
    goal_0.pose.orientation.w = 1;
    ros::Publisher pub_g1 = nh.advertise<geometry_msgs::PoseStamped>("/mapf_base/rb_1/goal", 1);
    geometry_msgs::PoseStamped goal_1;
    goal_1.header.frame_id = "map";
    goal_1.pose.position.x = 6.0;
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

    // publish goal transform flag
    ros::Publisher pub_flag= nh.advertise<std_msgs::Bool>("/mapf_base/goal_init_flag", 1);
    std_msgs::Bool flag;
    flag.data = true;
    while (pub_flag.getNumSubscribers() < 1) {
        rate.sleep();
        ROS_INFO("Wait for /mapf_base/goal_init_flag subscriber");
    }
    pub_flag.publish(flag);

    ROS_INFO("===== Planning successful! =====");

 
    ros::Subscriber sub = nh.subscribe("/mapf_base/global_plan", 1, sub_callback);
    timesleep.sleep();
    ros::MultiThreadedSpinner spinner(2);
    ros::spinOnce();
 
    return 0;
}