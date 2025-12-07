#ifndef BUMPERBOT_PLANNING_2__A_STAR_PLANNER_HPP_
#define BUMPERBOT_PLANNING_2__A_STAR_PLANNER_HPP_

#include <memory>
#include <vector>
#include <queue>
#include <functional>
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace path_planning
{

struct GraphNode
{
  int x;
  int y;
  double cost;
  double heuristic;
  std::shared_ptr<GraphNode> prev;
  
  bool operator==(const GraphNode& other) const
  {
    return x == other.x && y == other.y;
  }
  
  bool operator>(const GraphNode& other) const
  {
    return (cost + heuristic) > (other.cost + other.heuristic);
  }
  
  GraphNode operator+(const std::pair<int, int>& dir) const
  {
    return {x + dir.first, y + dir.second, 0.0, 0.0, nullptr};
  }
};

class AStarPlanner : public rclcpp::Node
{
public:
  AStarPlanner();

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
  nav_msgs::msg::Path plan(const geometry_msgs::msg::Pose& start, const geometry_msgs::msg::Pose& goal);
  
  // Helper functions
  GraphNode worldToGrid(const geometry_msgs::msg::Pose& pose);
  geometry_msgs::msg::Pose gridToWorld(const GraphNode& node);
  double manhattanDistance(const GraphNode& a, const GraphNode& b);
  bool poseInMap(const GraphNode& node);
  int poseToCell(const GraphNode& node);
  
  // ROS 2 members
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  
  // TF2 members
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Data members
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  nav_msgs::msg::OccupancyGrid visited_map_;
  std::vector<int8_t> visited_map_data_;
};

} // namespace path_planning

#endif 