#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

class BoxPublisher : public rclcpp::Node
{
public:
  BoxPublisher() : Node("box_publisher"), is_blue_(true), is_left_(true)
  {
    marker_publisher_ =
      this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&BoxPublisher::timerCallback, this));
  }

private:
  void timerCallback()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = this->now();
    marker.ns = "box";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    if (is_left_) {
      marker.pose.position.x = 3.0;
    } else {
      marker.pose.position.x = -3.0;
    }
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.5;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    if (is_blue_) {
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.color.a = 1.0;
    } else {
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
    }

    is_left_ = !is_left_;
    is_blue_ = !is_blue_;

    marker_publisher_->publish(marker);
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool is_left_;
  bool is_blue_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BoxPublisher>());
  rclcpp::shutdown();
  return 0;
}
