/**
 * @file      moving_turtle.hpp
 * @brief     background color changes by turtle's yaw
 * 
 * @date      2024-09-24 created by Seounghoon Park (sunghoon8585@gmail.com)
 */

#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

/*
[TO DO]
You should include the message header files for the message types used in this node.
*/


class ColorChanger : public rclcpp::Node   {
    public:
        ColorChanger(const std::string& node_name, const double& loop_rate);

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions   
        inline void CallbackTurtlePose(const turtlesim::msg::Pose::SharedPtr msg) {            
            // mutex_manual_input_.lock();
            i_turtle_pose_ = *msg;
            // mutex_manual_input_.unlock();
        }

        void Run(const rclcpp::Time &current_time);
        void Publish(const rclcpp::Time &current_time);
        
        // Publisher 
            // [To DO] You should define a publisher to publish the turtle color.

        // Subscriber
            // [To DO] You should define a subscriber to subscribe to the turtle pose.

        // Timer
        rclcpp::TimerBase::SharedPtr t_run_node_;

        // Inputs
        turtlesim::msg::Pose i_turtle_pose_;

        // Mutex

        // Outputs
        my_msgs::msg::TurtleColor o_turtle_color_;

        // Tuning Parameters

        std::shared_ptr<rclcpp::AsyncParametersClient> param_client_;

        // Algorithm Variables
        std::vector<rclcpp::Parameter> params_;

};