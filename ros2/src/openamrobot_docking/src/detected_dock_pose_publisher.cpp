/**
 * @file detected_dock_pose_publisher.cpp
 * @brief Publishes the pose of a detected docking tag from TF2
 *
 * This node listens for transforms between a parent frame (typically a camera
 * optical frame) and a child frame (typically an AprilTag frame). It then
 * republishes the transform as a PoseStamped message for downstream docking
 * consumers.
 */

#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class DetectedDockPosePublisher : public rclcpp::Node
{
public:
  DetectedDockPosePublisher()
  : Node("detected_dock_pose_publisher")
  {
    this->declare_parameter<std::string>("parent_frame", "map");
    this->declare_parameter<std::string>("child_frame", "charging_dock_apriltag");
    this->declare_parameter<std::string>("output_topic", "detected_dock_pose");
    this->declare_parameter<double>("publish_rate", 10.0);

    parent_frame_ = this->get_parameter("parent_frame").as_string();
    child_frame_ = this->get_parameter("child_frame").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    double publish_rate = this->get_parameter("publish_rate").as_double();

    if (publish_rate <= 0.0) {
      RCLCPP_WARN(this->get_logger(),
        "publish_rate <= 0.0 (%.3f), defaulting to 10.0 Hz", publish_rate);
      publish_rate = 10.0;
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    dock_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      output_topic_, 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
      std::bind(&DetectedDockPosePublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(),
      "Dock pose publisher ready. parent_frame='%s', child_frame='%s', topic='%s'",
      parent_frame_.c_str(), child_frame_.c_str(), output_topic_.c_str());
  }

private:
  void timer_callback()
  {
    geometry_msgs::msg::PoseStamped dock_pose;
    dock_pose.header.stamp = this->get_clock()->now();
    dock_pose.header.frame_id = parent_frame_;

    try {
      geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
        parent_frame_, child_frame_, tf2::TimePointZero);

      dock_pose.pose.position.x = transform.transform.translation.x;
      dock_pose.pose.position.y = transform.transform.translation.y;
      dock_pose.pose.position.z = transform.transform.translation.z;
      dock_pose.pose.orientation = transform.transform.rotation;

      dock_pose_pub_->publish(dock_pose);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(this->get_logger(), "Could not get transform: %s", ex.what());
    }
  }

  std::string parent_frame_;
  std::string child_frame_;
  std::string output_topic_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectedDockPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
