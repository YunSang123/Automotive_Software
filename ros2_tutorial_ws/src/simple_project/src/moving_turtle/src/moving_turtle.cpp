#include "moving_turtle.hpp"


MovingTurtle::MovingTurtle(const std::string& node_name, const double& loop_rate)
    : Node("moving_turtle")  {
    
    // [To DO] You should define a publisher to publish the turtle velocity command.
    p_turtle_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    loop_count_ = 0;

    t_run_node_ = this->create_wall_timer(
            std::chrono::milliseconds((int64_t)(1000 / loop_rate)),
            [this]()
            { this->Run(this->now()); });
}


void MovingTurtle::Run(const rclcpp::Time &current_time) {

    double vx;
    double yawrate;


    if (loop_count_ < 100){
        vx = 2.0;
        yawrate = 0.0;
        loop_count_++;
    }
    else if (loop_count_ < 200){
        vx = 0.0;
        yawrate = 1.0;
        loop_count_++;
    }
    if (loop_count_ == 200){
        loop_count_ = 0;
    }
    ///////////////////////////////////////
    o_turtle_cmd_.linear.x = vx;
    o_turtle_cmd_.angular.z = yawrate;

    Publish(current_time);
}


void MovingTurtle::Publish(const rclcpp::Time &current_time) {
    p_turtle_cmd_->publish(o_turtle_cmd_);
}



int main(int argc, char *argv[]) {
    std::string node_name = "moving_turtle";
    double loop_rate = 100.0;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MovingTurtle>(node_name, loop_rate));
    rclcpp::shutdown();
    return 0;
}
