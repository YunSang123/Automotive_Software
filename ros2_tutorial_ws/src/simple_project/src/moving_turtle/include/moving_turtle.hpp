/**
 * @file      moving_turtle.hpp
 * @brief     turtlesim moving
 * 
 * @date      2024-09-24 created by Seounghoon Park (sunghoon8585@gmail.com)
 */

#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"  

// [To DO] You should include the message header file for the message types used in this node.
#include "geometry_msgs/msg/twist.hpp"


class MovingTurtle : public rclcpp::Node   {
    public:
        MovingTurtle(const std::string& node_name, const double& loop_rate);

    private:
        
        void Run(const rclcpp::Time &current_time);
        void Publish(const rclcpp::Time& current_time);
        
        // Publisher
        // [To DO] You should define a publisher to publish the turtle velocity command.
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr p_turtle_cmd_;  
        // Subscriber

        // Timer
        rclcpp::TimerBase::SharedPtr t_run_node_;

        // Inputs

        // Mutex

        // Outputs
        geometry_msgs::msg::Twist o_turtle_cmd_;

        // Tuning Parameters

        // Algorithm Variables
        unsigned int loop_count_ = 0;

};