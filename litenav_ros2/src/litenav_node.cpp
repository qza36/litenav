#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "map_io.hpp"
#include "pid_path_follower.hpp"
#include "planning.hpp"

class LitenavNode : public rclcpp::Node {
public:
  explicit LitenavNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("litenav_node", options)
  {
    rclcpp::QoS latched(1);
    latched.transient_local();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", latched);
    plan_pub_ = create_publisher<nav_msgs::msg::Path>("/plan", 1);
    traj_pub_ = create_publisher<nav_msgs::msg::Path>("/trajectory", 1);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/robot_marker", 1);

    using PoseWithCov = geometry_msgs::msg::PoseWithCovarianceStamped;
    using PoseStamped = geometry_msgs::msg::PoseStamped;
    start_sub_ = create_subscription<PoseWithCov>(
        "/initialpose", 1,
        [this](PoseWithCov::ConstSharedPtr msg) {
          start_ = Pose2D{msg->pose.pose.position.x,
                          msg->pose.pose.position.y,
                          yawFromQuat(msg->pose.pose.orientation)};
          RCLCPP_INFO(get_logger(), "Start set: (%.2f, %.2f)", start_->x, start_->y);
          planAndPublish();
        });
    goal_sub_ = create_subscription<PoseStamped>(
        "/goal_pose", 1,
        [this](PoseStamped::ConstSharedPtr msg) {
          goal_ = Pose2D{msg->pose.position.x, msg->pose.position.y,
                         yawFromQuat(msg->pose.orientation)};
          RCLCPP_INFO(get_logger(), "Goal set: (%.2f, %.2f)", goal_->x, goal_->y);
          planAndPublish();
        });

    // PID parameters
    pid_config_.kp = declare_parameter("pid_kp", 1.0);
    pid_config_.ki = declare_parameter("pid_ki", 0.0);
    pid_config_.kd = declare_parameter("pid_kd", 0.3);
    pid_config_.max_linear_vel = declare_parameter("pid_max_linear_vel", 0.5);
    pid_config_.max_angular_vel = declare_parameter("pid_max_angular_vel", 1.0);
    pid_config_.dt = declare_parameter("pid_dt", 0.01);
    pid_config_.goal_tolerance = declare_parameter("pid_goal_tolerance", 0.1);
    pid_config_.lookahead_dist = declare_parameter("pid_lookahead_dist", 0.3);
    follower_.setConfig(pid_config_);

    // Load map
    const std::string map_yaml = declare_parameter("map_yaml",
        "/Users/zhiangqi/CLionProjects/litenav/map_module/maps/test.yaml");

    map_module::map_io io(map_yaml);
    if (io.loadMap() != map_module::map_io::LOAD_MAP_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to load map from: %s", map_yaml.c_str());
      return;
    }
    map_ = std::make_shared<GridMap>(io.getMap());
    planner_.setMap(map_);

    publishMap();
    publishStaticTransform();

    // Re-publish map periodically so late-joining subscribers (rviz2) always receive it
    map_timer_ = create_wall_timer(std::chrono::seconds(1),
                                   [this]() { publishMap(); });

    RCLCPP_INFO(get_logger(), "Litenav node ready. Map: %ux%u @ %.3f m/cell",
                map_->width, map_->height, map_->resolution);
  }

private:
  void publishMap() {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.header.stamp = now();
    grid.info.width = map_->width;
    grid.info.height = map_->height;
    grid.info.resolution = map_->resolution;
    grid.info.origin.position.x = map_->origin.x;
    grid.info.origin.position.y = map_->origin.y;
    grid.info.origin.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, map_->origin.yaw);
    grid.info.origin.orientation.w = q.w();
    grid.info.origin.orientation.x = q.x();
    grid.info.origin.orientation.y = q.y();
    grid.info.origin.orientation.z = q.z();
    grid.data = map_->data;
    map_pub_->publish(grid);
  }

  void publishStaticTransform() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = "map";
    t.child_frame_id = "odom";
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation.w = 1.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    tf_broadcaster_.sendTransform(t);
  }

  void planAndPublish() {
    if (!start_ || !goal_) return;

    // Cancel previous animation immediately before overwriting state
    if (anim_timer_) {
      anim_timer_->cancel();
      anim_timer_.reset();
    }
    trajectory_.clear();
    traj_index_ = 0;

    planning_module::PlanResult result;
    if (!planner_.createPlan(*start_, *goal_, result)) {
      RCLCPP_WARN(get_logger(), "Planning failed — start/goal may be occupied or unreachable");
      return;
    }

    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = now();

    for (const auto &wp : result.world_path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "map";
      ps.header.stamp = path.header.stamp;
      ps.pose.position.x = wp.x;
      ps.pose.position.y = wp.y;
      ps.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, wp.yaw);
      ps.pose.orientation.w = q.w();
      ps.pose.orientation.x = q.x();
      ps.pose.orientation.y = q.y();
      ps.pose.orientation.z = q.z();
      path.poses.push_back(ps);
    }

    plan_pub_->publish(path);
    RCLCPP_INFO(get_logger(), "Plan published: %zu waypoints, cost=%.2f m",
                result.world_path.size(), result.cost);

    // PID path following
    follower_.setPath(result.world_path);
    std::vector<Pose2D> trajectory;
    if (!follower_.followPath(*start_, trajectory)) {
      RCLCPP_WARN(get_logger(), "PID path following failed");
      return;
    }

    nav_msgs::msg::Path traj_msg;
    traj_msg.header.frame_id = "map";
    traj_msg.header.stamp = now();

    for (const auto &pose : trajectory) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "map";
      ps.header.stamp = traj_msg.header.stamp;
      ps.pose.position.x = pose.x;
      ps.pose.position.y = pose.y;
      ps.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, pose.yaw);
      ps.pose.orientation.w = q.w();
      ps.pose.orientation.x = q.x();
      ps.pose.orientation.y = q.y();
      ps.pose.orientation.z = q.z();
      traj_msg.poses.push_back(ps);
    }

    traj_pub_->publish(traj_msg);
    RCLCPP_INFO(get_logger(), "Trajectory published: %zu steps, end=(%.2f, %.2f)",
                trajectory.size(), trajectory.back().x, trajectory.back().y);

    // Start marker animation
    trajectory_ = std::move(trajectory);
    traj_index_ = 0;

    // Publish initial marker immediately
    publishMarker();

    // Animate at ~100 Hz (10ms per step)
    anim_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                    [this]() { publishMarker(); });
  }

  void publishMarker() {
    if (trajectory_.empty() || traj_index_ >= trajectory_.size()) {
      if (anim_timer_) {
        anim_timer_->cancel();
      }
      return;
    }

    const auto &pose = trajectory_[traj_index_];

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "robot";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = pose.x;
    marker.pose.position.y = pose.y;
    marker.pose.position.z = 0.05;
    tf2::Quaternion q;
    q.setRPY(0, 0, pose.yaw);
    marker.pose.orientation.w = q.w();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker.lifetime = rclcpp::Duration::max();

    marker_pub_->publish(marker);
    ++traj_index_;
  }

  static double yawFromQuat(const geometry_msgs::msg::Quaternion &q) {
    tf2::Quaternion tf2_q(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2_q).getRPY(roll, pitch, yaw);
    return yaw;
  }

  std::shared_ptr<GridMap> map_;
  planning_module::AStarPlanner planner_;
  control_module::PIDPathFollower follower_;
  control_module::PIDConfig pid_config_;
  std::optional<Pose2D> start_;
  std::optional<Pose2D> goal_;

  std::vector<Pose2D> trajectory_;
  std::size_t traj_index_{0};

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  tf2_ros::StaticTransformBroadcaster tf_broadcaster_{this};
  rclcpp::TimerBase::SharedPtr map_timer_;
  rclcpp::TimerBase::SharedPtr anim_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LitenavNode>());
  } catch (const std::exception &e) {
    fprintf(stderr, "[LITENAV] Fatal: %s\n", e.what());
    fflush(stderr);
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
