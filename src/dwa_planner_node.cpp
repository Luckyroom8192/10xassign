#include "10xassign/dwa_planner_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace custom_dwa_planner
{

DwaPlannerNode::DwaPlannerNode() : Node("dwa_planner_node")
{
    RCLCPP_INFO(this->get_logger(), "Starting DWA Planner Node...");

    // Declare DWA parameters
    this->declare_parameter("max_vel_x", 1.0);
    this->declare_parameter("min_vel_x", 0.0);
    this->declare_parameter("max_vel_theta", 2.0);
    this->declare_parameter("accel_lim_x", 0.5);
    this->declare_parameter("accel_lim_theta", 1.5);
    this->declare_parameter("sim_time", 2.0);
    this->declare_parameter("dt", 0.1);
    this->declare_parameter("v_samples", 11);
    this->declare_parameter("w_samples", 21);
    this->declare_parameter("controller_frequency", 10.0);
    this->declare_parameter("w_goal_dist", 1.0);
    this->declare_parameter("w_obstacle", 2.0);
    this->declare_parameter("w_velocity", 0.5);
    this->declare_parameter("robot_radius", 0.18);
    this->declare_parameter("goal_tolerance", 0.2);

    // Get parameters
    max_vel_x_ = this->get_parameter("max_vel_x").as_double();
    min_vel_x_ = this->get_parameter("min_vel_x").as_double();
    max_vel_theta_ = this->get_parameter("max_vel_theta").as_double();
    accel_lim_x_ = this->get_parameter("accel_lim_x").as_double();
    accel_lim_theta_ = this->get_parameter("accel_lim_theta").as_double();
    sim_time_ = this->get_parameter("sim_time").as_double();
    dt_ = this->get_parameter("dt").as_double();
    v_samples_ = this->get_parameter("v_samples").as_int();
    w_samples_ = this->get_parameter("w_samples").as_int();
    controller_frequency_ = this->get_parameter("controller_frequency").as_double();
    w_goal_dist_ = this->get_parameter("w_goal_dist").as_double();
    w_obstacle_ = this->get_parameter("w_obstacle").as_double();
    w_velocity_ = this->get_parameter("w_velocity").as_double();
    robot_radius_ = this->get_parameter("robot_radius").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();

    // Subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&DwaPlannerNode::odom_callback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&DwaPlannerNode::scan_callback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        std::bind(&DwaPlannerNode::goal_callback, this, std::placeholders::_1));

    // Publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker_array", 10);

    // TF2
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // Main timer logic
    auto controller_period = std::chrono::duration<double>(1.0 / controller_frequency_);
    timer_ = this->create_wall_timer(
        controller_period, std::bind(&DwaPlannerNode::compute_and_publish, this));
}

void DwaPlannerNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_odom_ = *msg;
    odom_received_ = true;
    RCLCPP_DEBUG(this->get_logger(), "Odometry received.");
}

void DwaPlannerNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_scan_ = msg;
    scan_received_ = true;
    RCLCPP_DEBUG(this->get_logger(), "Scan received.");
}

void DwaPlannerNode::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_goal_ = *msg;
    goal_received_ = true;
    goal_reached_ = false;
    RCLCPP_INFO(this->get_logger(), "New goal received!");
}

void DwaPlannerNode::compute_and_publish()
{
    RCLCPP_DEBUG(this->get_logger(), "Computing command velocity...");

    // Lock data and copy
    nav_msgs::msg::Odometry odom_copy;
    sensor_msgs::msg::LaserScan::SharedPtr scan_copy;
    geometry_msgs::msg::PoseStamped goal_copy;

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!odom_received_ || !scan_received_ || !goal_received_)
        {
            RCLCPP_DEBUG(this->get_logger(), "Waiting for data...");
            return;
        }
        odom_copy = current_odom_;
        scan_copy = latest_scan_;
        goal_copy = current_goal_;
    }
    
    // --- GOAL CHECKING & LOGGING ---
    geometry_msgs::msg::Pose current_pose = odom_copy.pose.pose;
    double dx_goal = goal_copy.pose.position.x - current_pose.position.x;
    double dy_goal = goal_copy.pose.position.y - current_pose.position.y;
    double dist_to_goal = std::hypot(dx_goal, dy_goal);

    // LOGGING YOU REQUESTED:
    RCLCPP_INFO(this->get_logger(), "Current distance to goal: %.2f meters", dist_to_goal);

    if (goal_reached_) {
        // We are at the goal, stay stopped
        return;
    }

    if (dist_to_goal < goal_tolerance_)
    {
        RCLCPP_INFO(this->get_logger(), "Goal Reached! Stopping robot.");
        goal_reached_ = true;
        
        auto cmd = std::make_unique<geometry_msgs::msg::Twist>();
        cmd->linear.x = 0.0;
        cmd->angular.z = 0.0;
        cmd_vel_pub_->publish(std::move(cmd));
        return;
    }

    // Get current state
    geometry_msgs::msg::Twist current_vel = odom_copy.twist.twist;

    // --- Obstacle Pre-processing ---
    // Transform all laser scan points to the 'odom' frame once
    std::vector<geometry_msgs::msg::Point> obstacle_points_odom;
    geometry_msgs::msg::TransformStamped scan_to_odom_tf;
    try {
        // Use tf2::TimePointZero for latest available transform
        scan_to_odom_tf = tf_buffer_->lookupTransform(
            "odom", scan_copy->header.frame_id, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to odom: %s",
                    scan_copy->header.frame_id.c_str(), ex.what());
        return;
    }

    for (size_t i = 0; i < scan_copy->ranges.size(); ++i)
    {
        if (!std::isfinite(scan_copy->ranges[i])) continue;

        double angle = scan_copy->angle_min + i * scan_copy->angle_increment;
        geometry_msgs::msg::PointStamped scan_point_scan_frame;
        scan_point_scan_frame.header = scan_copy->header;
        scan_point_scan_frame.point.x = scan_copy->ranges[i] * std::cos(angle);
        scan_point_scan_frame.point.y = scan_copy->ranges[i] * std::sin(angle);
        scan_point_scan_frame.point.z = 0.0;

        geometry_msgs::msg::PointStamped scan_point_odom_frame;
        tf2::doTransform(scan_point_scan_frame, scan_point_odom_frame, scan_to_odom_tf);
        obstacle_points_odom.push_back(scan_point_odom_frame.point);
    }

    VelocityWindow window = calculate_dynamic_window(current_vel);

    std::vector<Trajectory> all_trajectories;
    Trajectory best_traj;
    double min_cost = DBL_MAX;
    bool best_traj_found = false;

    double v_step = (v_samples_ > 1) ? (window.v_max - window.v_min) / (v_samples_ - 1) : 0;
    double w_step = (w_samples_ > 1) ? (window.w_max - window.w_min) / (w_samples_ - 1) : 0;

    for (int i = 0; i < v_samples_; ++i)
    {
        double v = (v_samples_ == 1) ? window.v_min : window.v_min + i * v_step;
        
        for (int j = 0; j < w_samples_; ++j)
        {
            double w = (w_samples_ == 1) ? window.w_min : window.w_min + j * w_step;

            Trajectory traj;
            traj.v_samp = v;
            traj.w_samp = w;
            traj.poses = predict_trajectory(v, w, current_pose);
            
            traj.cost = evaluate_trajectory(traj, goal_copy, obstacle_points_odom);
            all_trajectories.push_back(traj);

            if (traj.cost < min_cost)
            {
                min_cost = traj.cost;
                best_traj = traj;
                best_traj_found = true;
            }
        }
    }

    auto cmd = std::make_unique<geometry_msgs::msg::Twist>();
    if (best_traj_found)
    {
        double cost_g = goal_dist_cost(best_traj, goal_copy);
        double cost_o = obstacle_cost(best_traj, obstacle_points_odom);
        double cost_v = velocity_cost(best_traj);
        RCLCPP_INFO(this->get_logger(), 
            "Best Traj: v=%.2f, w=%.2f, TotalCost=%.2f | GoalCost: %.2f (w*g=%.2f), ObsCost: %.2f (w*o=%.2f), VelCost: %.2f (w*v=%.2f)",
            best_traj.v_samp, best_traj.w_samp, best_traj.cost,
            cost_g, (w_goal_dist_ * cost_g),
            cost_o, (w_obstacle_ * cost_o),
            cost_v, (w_velocity_ * cost_v)
        );

        cmd->linear.x = best_traj.v_samp;
        cmd->angular.z = best_traj.w_samp;
    }
    else
    {
        // No valid path found, stop the robot
        cmd->linear.x = 0.0;
        cmd->angular.z = 0.0;
        RCLCPP_WARN(this->get_logger(), "No valid trajectory found!");
    }
    cmd_vel_pub_->publish(std::move(cmd));

    // --- Phase 5: Visualization ---
    publish_visualizations(all_trajectories, best_traj);
}

DwaPlannerNode::VelocityWindow DwaPlannerNode::calculate_dynamic_window(const geometry_msgs::msg::Twist& current_vel)
{
    VelocityWindow window;
    double dt_main = 1.0 / controller_frequency_;

    // 1. Window from acceleration limits
    double v_min_accel = current_vel.linear.x - accel_lim_x_ * dt_main;
    double v_max_accel = current_vel.linear.x + accel_lim_x_ * dt_main;
    double w_min_accel = current_vel.angular.z - accel_lim_theta_ * dt_main;
    double w_max_accel = current_vel.angular.z + accel_lim_theta_ * dt_main;

    // 2. Intersect with robot's hard limits
    window.v_min = std::max(min_vel_x_, v_min_accel);
    window.v_max = std::min(max_vel_x_, v_max_accel);
    window.w_min = std::max(-max_vel_theta_, w_min_accel);
    window.w_max = std::min(max_vel_theta_, w_max_accel);

    return window;
}

std::vector<geometry_msgs::msg::Pose> DwaPlannerNode::predict_trajectory(
    double v, double w, const geometry_msgs::msg::Pose& start_pose)
{
    std::vector<geometry_msgs::msg::Pose> trajectory_poses;
    trajectory_poses.push_back(start_pose);

    double current_x = start_pose.position.x;
    double current_y = start_pose.position.y;
    double current_theta = tf2::getYaw(start_pose.orientation);

    for (double t = dt_; t <= sim_time_; t += dt_)
    {
        // Simple differential drive model
        double delta_x = v * std::cos(current_theta) * dt_;
        double delta_y = v * std::sin(current_theta) * dt_;
        double delta_theta = w * dt_;

        current_x += delta_x;
        current_y += delta_y;
        current_theta += delta_theta;

        // Create new pose
        geometry_msgs::msg::Pose new_pose;
        new_pose.position.x = current_x;
        new_pose.position.y = current_y;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, current_theta);
        new_pose.orientation = tf2::toMsg(q);

        trajectory_poses.push_back(new_pose);
    }
    return trajectory_poses;
}

void DwaPlannerNode::publish_visualizations(
    const std::vector<Trajectory>& trajectories,
    const Trajectory& best_traj)
{
    visualization_msgs::msg::MarkerArray marker_array;

    // Delete all old markers
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = "odom";
    delete_marker.header.stamp = this->now();
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    int id = 0;
    for (const auto& traj : trajectories)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = this->now();
        marker.ns = "dwa_trajectories";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.01; // Line width

        if (traj.cost == DBL_MAX) {
            // Red for invalid (collision) paths
            marker.color.a = 0.5;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        } else {
            // Blue for valid paths
            marker.color.a = 0.5;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        }

        for (const auto& pose : traj.poses)
        {
            marker.points.push_back(pose.position);
        }
        marker_array.markers.push_back(marker);
    }

    // Highlight the best trajectory in Green
    if (!best_traj.poses.empty())
    {
        visualization_msgs::msg::Marker best_marker;
        best_marker.header.frame_id = "odom";
        best_marker.header.stamp = this->now();
        best_marker.ns = "best_trajectory";
        best_marker.id = -1; // Special ID
        best_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        best_marker.action = visualization_msgs::msg::Marker::ADD;
        best_marker.pose.orientation.w = 1.0;
        best_marker.scale.x = 0.05; // Thicker line
        
        // Green color
        best_marker.color.a = 1.0;
        best_marker.color.r = 0.0;
        best_marker.color.g = 1.0;
        best_marker.color.b = 0.0;

        for (const auto& pose : best_traj.poses)
        {
            best_marker.points.push_back(pose.position);
        }
        marker_array.markers.push_back(best_marker);
    }

    viz_pub_->publish(marker_array);
}

double DwaPlannerNode::evaluate_trajectory(
    const Trajectory& traj,
    const geometry_msgs::msg::PoseStamped& goal,
    const std::vector<geometry_msgs::msg::Point>& obstacles)
{
    // --- Obstacle Cost ---
    double cost_o = obstacle_cost(traj, obstacles);
    // If it collides, it's infinitely bad
    if (cost_o == DBL_MAX)
    {
        return DBL_MAX;
    }

    // --- Goal Distance Cost ---
    double cost_g = goal_dist_cost(traj, goal);

    // --- Velocity Cost ---
    double cost_v = velocity_cost(traj);

    // --- Final Weighted Cost ---
    return (w_goal_dist_ * cost_g) +
           (w_obstacle_ * cost_o) +
           (w_velocity_ * cost_v);
}

double DwaPlannerNode::goal_dist_cost(
    const Trajectory& traj,
    const geometry_msgs::msg::PoseStamped& goal)
{
    // Get the final pose of the trajectory
    const auto& final_pose = traj.poses.back();
    double dx = goal.pose.position.x - final_pose.position.x;
    double dy = goal.pose.position.y - final_pose.position.y;
    return std::hypot(dx, dy); // Euclidean distance
}

double DwaPlannerNode::obstacle_cost(
    const Trajectory& traj,
    const std::vector<geometry_msgs::msg::Point>& obstacles)
{
    double min_dist_to_obs = DBL_MAX;

    for (const auto& pose : traj.poses)
    {
        for (const auto& obs_point : obstacles)
        {
            double dist = std::hypot(pose.position.x - obs_point.x,
                                     pose.position.y - obs_point.y);
            
            if (dist < min_dist_to_obs)
            {
                min_dist_to_obs = dist;
            }
        }
    }

    // If the closest we get is inside the robot, this path is invalid
    if (min_dist_to_obs < robot_radius_)
    {
        return DBL_MAX;
    }

    // Cost is inverse of distance: higher cost as we get closer
    return (1.0 / min_dist_to_obs);
}

double DwaPlannerNode::velocity_cost(const Trajectory& traj)
{
    // Penalize slow trajectories to encourage moving
    return max_vel_x_ - traj.v_samp;
}

} // namespace custom_dwa_planner

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<custom_dwa_planner::DwaPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}