#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"

#include <mutex>

namespace custom_dwa_planner
{

class DwaPlannerNode : public rclcpp::Node
{
public:
    DwaPlannerNode();

private:

    // Helper Structs
    struct VelocityWindow
    {
        double v_min, v_max, w_min, w_max;
    };

    struct Trajectory
    {
        double v_samp;
        double w_samp;
        std::vector<geometry_msgs::msg::Pose> poses;
        double cost{0.0}; 
    };

    // DWA Parameters
    double max_vel_x_;
    double min_vel_x_;
    double max_vel_theta_;
    double accel_lim_x_;
    double accel_lim_theta_;
    double sim_time_;
    double dt_;
    int v_samples_;
    int w_samples_;
    double controller_frequency_;
    double robot_radius_;
    double goal_tolerance_;

    // Cost Function Weights
    double w_goal_dist_;
    double w_obstacle_;
    double w_velocity_;

    // Helper Functions
    VelocityWindow calculate_dynamic_window(const geometry_msgs::msg::Twist& current_vel);
    std::vector<geometry_msgs::msg::Pose> predict_trajectory(
        double v, double w, const geometry_msgs::msg::Pose& start_pose);

    void publish_visualizations(
        const std::vector<Trajectory>& trajectories,
        const Trajectory& best_traj);

    double evaluate_trajectory(
        const Trajectory& traj,
        const geometry_msgs::msg::PoseStamped& goal,
        const std::vector<geometry_msgs::msg::Point>& obstacles);

    double goal_dist_cost(
        const Trajectory& traj,
        const geometry_msgs::msg::PoseStamped& goal);

    double obstacle_cost(
        const Trajectory& traj,
        const std::vector<geometry_msgs::msg::Point>& obstacles);

    double velocity_cost(const Trajectory& traj);

    // Callbacks
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void compute_and_publish();

    // ROS 2 Interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    // Member Variables
    std::mutex data_mutex_;
    nav_msgs::msg::Odometry current_odom_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    geometry_msgs::msg::PoseStamped current_goal_;

    bool odom_received_{false};
    bool scan_received_{false};
    bool goal_received_{false};
    bool goal_reached_{false};
};

}