#ifndef path_PLANNING_2_DIJKSTRA_PLANNER_HPP
#define path_PLANNING_2_DIJKSTRA_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>

namespace path_planning
{

struct GraphNode {
    int x;
    int y;
    double cost;
    std::shared_ptr<GraphNode> prev;

    GraphNode() : x(0), y(0), cost(0.0), prev(nullptr) {}
    
    GraphNode(int x_, int y_) : x(x_), y(y_), cost(0.0), prev(nullptr) {}
    
    GraphNode(int x_, int y_, double cost_) : x(x_), y(y_), cost(cost_), prev(nullptr) {}

    bool operator>(const GraphNode& other) const {
        return cost > other.cost;
    }

    bool operator==(const GraphNode& other) const {
        return x == other.x && y == other.y;
    }

    GraphNode operator+(const std::pair<int, int>& dir) const {
        GraphNode result;
        result.x = x + dir.first;
        result.y = y + dir.second;
        result.cost = cost;
        result.prev = prev;
        return result;
    }
};

class DijkstraPlanner : public rclcpp::Node
{
public:
    DijkstraPlanner();

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
    
    nav_msgs::msg::Path plan(const geometry_msgs::msg::Pose & start, const geometry_msgs::msg::Pose & goal);
    
    bool poseOnMap(const GraphNode & node);
    GraphNode worldToGrid(const geometry_msgs::msg::Pose & pose);
    geometry_msgs::msg::Pose gridToWorld(const GraphNode & node);
    unsigned int poseToCell(const GraphNode & node);

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    nav_msgs::msg::OccupancyGrid visited_map_;
};

}  // namespace path_planning

#endif  // path_PLANNING_2_DIJKSTRA_PLANNER_HPP