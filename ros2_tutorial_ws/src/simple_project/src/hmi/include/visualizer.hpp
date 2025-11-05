/**
 * @file      hmi.hpp
 * @brief     visualize turtle's pose & etc by marker & custom msgs
 * 
 * @date      2024-09-25 created by Seounghoon Park (sunghoon8585@gmail.com)
 */

#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

// [To DO] You should include the message header files for the message types used in this node.



class Visualizer : public rclcpp::Node   {
    public:
        Visualizer(const std::string& node_name, const double& loop_rate);

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions   
        inline void CallbackTurtlePose(const turtlesim::msg::Pose::SharedPtr msg) {            
            i_turtle_pose_ = *msg;
        }

        inline void CallbackTurtleColor(const my_msgs::msg::TurtleColor::SharedPtr msg) {            
            i_turtle_color_ = *msg;
        }
        
        
        void Run(const rclcpp::Time &current_time);
        void Publish(const rclcpp::Time& current_time);
        
        void UpdateMarker(  const rclcpp::Time& current_time,
                            const turtlesim::msg::Pose& turtle_pose,
                            const my_msgs::msg::TurtleColor& turtle_color);
        
        
        // Publisher 
        // To DO - define a publisher to publish the turtle marker

        // Subscriber
        // To DO - define subscribers to subscribe to the turtle pose and turtle color

        // Timer
        rclcpp::TimerBase::SharedPtr t_run_node_;

        // Inputs
        turtlesim::msg::Pose i_turtle_pose_;
        my_msgs::msg::TurtleColor i_turtle_color_;

        // Mutex

        // Outputs
        visualization_msgs::msg::Marker o_turtle_marker_;

        // Tuning Parameters

        std::shared_ptr<rclcpp::AsyncParametersClient> param_client_;

        // Algorithm Variables
        std::vector<rclcpp::Parameter> params_;

};