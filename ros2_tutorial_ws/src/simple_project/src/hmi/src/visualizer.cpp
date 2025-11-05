#include "visualizer.hpp"


Visualizer::Visualizer(const std::string& node_name, const double& loop_rate)
    : Node(node_name)  {
    
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // -------------------To DO - define publisher and subscriber--------------------
        // ~~~

    t_run_node_ = this->create_wall_timer(
            std::chrono::milliseconds((int64_t)(1000 / loop_rate)),
            [this]()
            { this->Run(this->now()); });
}


void Visualizer::Run(const rclcpp::Time &current_time) {
    
    
    UpdateMarker(current_time, i_turtle_pose_, i_turtle_color_);
    Publish(current_time);

}


void Visualizer::Publish(const rclcpp::Time& current_time) {
    p_turtle_marker_->publish(o_turtle_marker_);
}


void Visualizer::UpdateMarker(  const rclcpp::Time& current_time,
                                const turtlesim::msg::Pose& turtle_pose,
                                const my_msgs::msg::TurtleColor& turtle_color) {

    visualization_msgs::msg::Marker marker_msg;

    // Set the frame ID and timestamp about the marker
 

    o_turtle_marker_ = marker_msg;
}


int main(int argc, char *argv[]) {
    std::string node_name = "hmi";
    double loop_rate = 60.0;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Visualizer>(node_name, loop_rate));
    rclcpp::shutdown();
    return 0;
}
