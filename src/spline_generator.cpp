#include "spline_generator/catmull_rom_spline.hpp"  // Custom header for spline generation (CatmullRomSpline & Point2D structs)
#include <rclcpp/rclcpp.hpp>                        // ROS 2 main client lib
#include <nav_msgs/msg/path.hpp>                    // To publish the final path as nav_msgs/Path
#include <geometry_msgs/msg/pose_stamped.hpp>       // Each point on the path is a PoseStamped

// Creating a ROS2 node for generating and publishing a Catmull-Rom spline
class CatmullRomNode : public rclcpp::Node {
public:
  CatmullRomNode() : Node("catmull_rom_node") {
    // Setting up publisher to publish the final path on 'catmull_rom_path' topic
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("catmull_rom_path", 10);

    // These are the 4 control points we'll use to create the spline
    Point2D p0{0.0, 0.0}, p1{1.0, 2.0}, p2{4.0, 2.0}, p3{5.0, 0.0};

    // Initializing the spline using the control points
    CatmullRomSpline spline(p0, p1, p2, p3);

    // Getting a list of interpolated points on the spline (0.1 is the step/resolution)
    auto spline_points = spline.computeSpline(0.1);

    // Preparing the nav_msgs::Path message to hold the result
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";        // Frame for RViz or tf reference
    path_msg.header.stamp = now();           // Timestamp to sync properly

    // Now convert every 2D point to a PoseStamped and push it to the path
    for (const auto& pt : spline_points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;         // Use same header for all poses
      pose.pose.position.x = pt.x;
      pose.pose.position.y = pt.y;
      pose.pose.position.z = 0.0;            // Since we're in 2D, z is 0
      pose.pose.orientation.w = 1.0;         // No rotation, so identity quaternion
      path_msg.poses.push_back(pose);
    }

    // Publish the generated path on the topic
    path_pub_->publish(path_msg);

    // Just logging how many points got published in total
    RCLCPP_INFO(this->get_logger(), "Published Catmull-Rom path with %lu points", spline_points.size());
  }

private:
  // ROS2 publisher object to publish the spline path
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

// Entry point of the program
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);  // ROS2 initialization

  // Creating the node and spinning it (so it stays alive)
  rclcpp::spin(std::make_shared<CatmullRomNode>());

  rclcpp::shutdown();  // Clean shutdown
  return 0;
}
