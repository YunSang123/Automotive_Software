#include "color_changer.hpp"


ColorChanger::ColorChanger(const std::string& node_name, const double& loop_rate)
    : Node(node_name)  {
    
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    
    // -------------------To DO - define publisher and subscriber--------------------
    p_turtle_color_ = this->create_publisher<turtlesim::msg::Color>("turtle1/color_sensor", qos_profile);
    

    params_.push_back(rclcpp::Parameter("background_r", 255));  
    params_.push_back(rclcpp::Parameter("background_g", 0));    
    params_.push_back(rclcpp::Parameter("background_b", 0));    


    t_run_node_ = this->create_wall_timer(
            std::chrono::milliseconds((int64_t)(1000 / loop_rate)),
            [this]()
            { this->Run(this->now()); });
}


void ColorChanger::Run(const rclcpp::Time &current_time) {
    // is before sub running enabled??

    // [To DO]  You should implement the algorithm to change the background color according to the turtle's yaw angle.
    // ~~~

    Publish(current_time);
}

void ColorChanger::Publish(const rclcpp::Time &current_time) {
    p_turtle_color_->publish(o_turtle_color_);
}


int main(int argc, char *argv[]) {
    std::string node_name = "color_changer";
    double loop_rate = 10.0;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColorChanger>(node_name, loop_rate));
    rclcpp::shutdown();
    return 0;
}
