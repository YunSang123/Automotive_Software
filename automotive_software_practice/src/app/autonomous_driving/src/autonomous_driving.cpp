/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      autonomous_driving.cpp
 * @brief     autonomous driving algorithm
 *
 * @date      2018-11-20 created by Kichun Jo (kichunjo@hanyang.ac.kr)
 *            2023-08-07 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : adapt new template
 *            2023-08-20 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : change to ROS2
 *            2024-10-21 updated by Seongjae Jeong (sjeong99@hanyang.ac.kr)
 *              : add the missions
 */

#include "autonomous_driving.hpp"

AutonomousDriving::AutonomousDriving(const std::string &node_name, const double &loop_rate,
                                     const rclcpp::NodeOptions &options)
    : Node(node_name, options) {

    RCLCPP_WARN(this->get_logger(), "Initialize node...");

    // QoS init
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // Parameters
    this->declare_parameter("autonomous_driving/ns", "");
    if (!this->get_parameter("autonomous_driving/ns", param_vehicle_namespace_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get vehicle_namespace");
        param_vehicle_namespace_ = "";
    }
    else {
        RCLCPP_INFO(this->get_logger(), "param_vehicle_namespace_: %s", param_vehicle_namespace_.c_str());
    }
    this->declare_parameter("autonomous_driving/use_manual_inputs", false);
    if (!this->get_parameter("autonomous_driving/use_manual_inputs", param_use_manual_inputs_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get use_manual_inputs");
        param_use_manual_inputs_ = true;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "param_use_manual_inputs_: %d", param_use_manual_inputs_);
    }
    this->declare_parameter("autonomous_driving/pure_pursuit_kd", 1.0);
    if (!this->get_parameter("autonomous_driving/pure_pursuit_kd", param_pp_kd_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get pure_pursuit_kd");
        param_pp_kd_ = 1.0;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "param_pp_kd_: %f", param_pp_kd_);
    }
    this->declare_parameter("autonomous_driving/pure_pursuit_kv", 0.0);
    if (!this->get_parameter("autonomous_driving/pure_pursuit_kv", param_pp_kv_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get pure_pursuit_kv");
        param_pp_kv_ = 0.0;
    }
    this->declare_parameter("autonomous_driving/pure_pursuit_kc", 0.0);
    if (!this->get_parameter("autonomous_driving/pure_pursuit_kc", param_pp_kc_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get pure_pursuit_kc");
        param_pp_kc_ = 0.0;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "param_pp_kv_: %f", param_pp_kv_);
    }
    this->declare_parameter("autonomous_driving/pid_kp", 0.0);
    if (!this->get_parameter("autonomous_driving/pid_kp", param_pid_kp_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get pid_kp");
        param_pid_kp_ = 0.0;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "param_pid_kp_: %f", param_pid_kp_);
    }
    this->declare_parameter("autonomous_driving/pid_ki", 0.0);
    if (!this->get_parameter("autonomous_driving/pid_ki", param_pid_ki_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get pid_ki");
        param_pid_ki_ = 0.0;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "param_pid_ki_: %f", param_pid_ki_);
    }
    this->declare_parameter("autonomous_driving/pid_kd", 0.0);
    if (!this->get_parameter("autonomous_driving/pid_kd", param_pid_kd_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get pid_kd");
        param_pid_kd_ = 0.0;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "param_pid_kd_: %f", param_pid_kd_);
    }
    this->declare_parameter("autonomous_driving/brake_ratio", 1.0);
    if (!this->get_parameter("autonomous_driving/brake_ratio", param_brake_ratio_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get brake_ratio");
        param_brake_ratio_ = 0.0;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "param_brake_ratio_: %f", param_brake_ratio_);
    }

    // Subscribers
    s_manual_input_ = this->create_subscription<ad_msgs::msg::VehicleInput>(
        "/manual_input", qos_profile, std::bind(&AutonomousDriving::CallbackManualInput, this, std::placeholders::_1));
    s_vehicle_state_ = this->create_subscription<ad_msgs::msg::VehicleOutput>(
        "vehicle_state", qos_profile, std::bind(&AutonomousDriving::CallbackVehicleState, this, std::placeholders::_1));
    s_limit_speed_ = this->create_subscription<std_msgs::msg::Float32>(
        "limit_speed", qos_profile, std::bind(&AutonomousDriving::CallbackLimitSpeed, this, std::placeholders::_1));
    s_lane_points_ = this->create_subscription<ad_msgs::msg::LanePointData>(
        "lane_points", qos_profile, std::bind(&AutonomousDriving::CallbackLanePoints, this, std::placeholders::_1));
    s_obstacles_ = this->create_subscription<ad_msgs::msg::Obstacles>(
        "obstacles", qos_profile, std::bind(&AutonomousDriving::CallbackObstacles, this, std::placeholders::_1));

    // Publishers
    p_vehicle_command_ = this->create_publisher<ad_msgs::msg::VehicleInput>(
        "vehicle_command", qos_profile);
    p_driving_way_ = this->create_publisher<ad_msgs::msg::PolyfitLaneData>(
        "driving_way", qos_profile);
    p_poly_lanes_ = this->create_publisher<ad_msgs::msg::PolyfitLaneDataArray>(
        "poly_lanes", qos_profile);

    // Initialize
    Init(this->now());

    // Timer init
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds((int64_t)(1000 / loop_rate)),
        [this]()
        { this->Run(this->now()); });
}

AutonomousDriving::~AutonomousDriving() {}

void AutonomousDriving::Init(const rclcpp::Time &current_time) {
}

void AutonomousDriving::UpdateParameter() {
}

void AutonomousDriving::Run(const rclcpp::Time &current_time) {
    UpdateParameter();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    mutex_vehicle_state_.lock();
    ad_msgs::msg::VehicleOutput vehicle_state = i_vehicle_state_;
    mutex_vehicle_state_.unlock();

    mutex_obstacles_.lock();
    ad_msgs::msg::Obstacles obstacles = i_obstacles_;
    mutex_obstacles_.unlock();

    mutex_limit_speed_.lock();
    double limit_speed = i_limit_speed_;
    mutex_limit_speed_.unlock();

    mutex_lane_points_.lock();
    ad_msgs::msg::LanePointData lane_points = i_lane_points_;
    mutex_lane_points_.unlock();

    mutex_vehicle_command_.lock();
    ad_msgs::msg::VehicleInput manual_vehicle_command = i_vehicle_command_;
    mutex_vehicle_command_.unlock();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Output variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    ad_msgs::msg::PolyfitLaneData driving_way;
    ad_msgs::msg::PolyfitLaneDataArray poly_lanes;
    ad_msgs::msg::VehicleInput vehicle_command;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Algorithm
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    /**
     * @brief Calculate the vehicle command using given inputs
     * 
     * inputs
     *      vehicle_state, limit_speed, obstacles, lane_points
     * outputs
     *      driving_way, poly_lanes, vehicle_command
     * 
     * TODO1. Find the driving way from the lane points
     * TODO2. Calculate the steering angle using lateral control
     */

    /* TODO1. Complete FindDrivingWay function*/
    driving_way = FindDrivingWay(vehicle_state, lane_points);
    
    /* TODO2. Complete LateralControl function*/
    double steering = LateralControl(vehicle_state, driving_way);


    // Update vehicle command
    vehicle_command.steering = steering;
    if (param_use_manual_inputs_) {
        vehicle_command.accel = manual_vehicle_command.accel;
        vehicle_command.brake = manual_vehicle_command.brake;
        std::cout << "[MODE] Manual Longitudinal Input Mode" << std::endl;
    } else {
        std::pair<double, double> accel_brake_command;
        accel_brake_command = AutonomousDriving::LongitudinalControl(vehicle_state, limit_speed);
        vehicle_command.accel = accel_brake_command.first;
        vehicle_command.brake = accel_brake_command.second;
        std::cout << "[MODE] PID Longitudinal Input Mode" << std::endl;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    o_driving_way_ = driving_way;
    o_poly_lanes_ = poly_lanes;
    o_vehicle_command_ = vehicle_command;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Publish output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    Publish(current_time);
}

ad_msgs::msg::PolyfitLaneData AutonomousDriving::FindDrivingWay(const ad_msgs::msg::VehicleOutput &vehicle_state,
                                                                const ad_msgs::msg::LanePointData &lane_points) {
    ad_msgs::msg::PolyfitLaneData driving_way;
    driving_way.frame_id = param_vehicle_namespace_ + "/body";
    /**
     * @brief Find the driving way from the lane points
     * inputs: vehicle_state, lane_points
     * outputs: driving_way
     * Purpose: Implement lane fitting algorithm to find the driving way (center line) from the given lane points
     */

    ////////////////////// TODO //////////////////////

    /**********   Step 1: Separate lane points into left and right lanes ***********/
    // (This is a placeholder; actual polynomial fitting code should be implemented here)
    
    // 1-1 Check and initialize the number of points
    size_t length_of_lane_points = lane_points.point.size();
    // 1-2 Separate into two groups : left and right lane
    // If the y value of first index of lane_points.point is positive, the lane is on the left side of the vehicle
    // Vehicle_state : absolute coordinate
    // Lane point : ego coordinate

    std::vector<geometry_msgs::msg::Point> left_lane;    // left lane
    std::vector<geometry_msgs::msg::Point> right_lane;   // right lane
    for (int i=0; i<length_of_lane_points; i++) {
        // left lane
        if (lane_points.point[i].y > 0) {
            left_lane.push_back(lane_points.point[i]);
        }

        else {
            right_lane.push_back(lane_points.point[i]);
        }
    }
    size_t length_of_left_lane = left_lane.size();
    size_t length_of_right_lane = right_lane.size();
    
    // 1-3 Initialize X, Y, and A matrices for left and right lanes with correct sizes
    MatrixXd Left_X (length_of_left_lane, 4);
    MatrixXd Left_Y (length_of_left_lane, 1);
    MatrixXd Right_X (length_of_right_lane, 4);
    MatrixXd Right_Y (length_of_right_lane, 1);
    Vector4d Left_Coefficient;
    Vector4d Right_Coefficient;
    for (int i=0; i<length_of_left_lane; i++) {
        Left_X(i, 0) = 1;
        Left_X(i, 1) = left_lane[i].x;
        Left_X(i, 2) = pow(left_lane[i].x, 2);
        Left_X(i, 3) = pow(left_lane[i].x, 3);
        Left_Y(i, 0) = left_lane[i].y;
    }

    for (int i=0; i<length_of_right_lane; i++) {
        Right_X(i, 0) = 1;
        Right_X(i, 1) = right_lane[i].x;
        Right_X(i, 2) = pow(right_lane[i].x, 2);
        Right_X(i, 3) = pow(right_lane[i].x, 3);
        Right_Y(i, 0) = right_lane[i].y;
    }

    /**********   Step 2: Fit a polynomial to each lane's points *******************/
    // (This is a placeholder; actual polynomial fitting code should be implemented here)

    // 2-1 Get optimized left lane coefficients A_left
    Left_Coefficient = (Left_X.transpose() * Left_X).inverse()*Left_X.transpose()*Left_Y;
    // 2-2 Get optimized right lane coefficients A_right
    Right_Coefficient = (Right_X.transpose() * Right_X).inverse()*Right_X.transpose()*Right_Y;

    // Step 3: Determine the driving way as the center line between the left and right lane
    // (This is a placeholder; actual center line calculation code should be implemented here)
    // Update driving_way with calculated center line
    driving_way.a0 = (Left_Coefficient[0] + Right_Coefficient[0])/2;
    driving_way.a1 = (Left_Coefficient[1] + Right_Coefficient[1])/2;
    driving_way.a2 = (Left_Coefficient[2] + Right_Coefficient[2])/2;
    driving_way.a3 = (Left_Coefficient[3] + Right_Coefficient[3])/2;

    ////////////////////////////////////////////////////
    return driving_way;
}

double AutonomousDriving::LateralControl(const ad_msgs::msg::VehicleOutput &vehicle_state,
                                          const ad_msgs::msg::PolyfitLaneData &driving_way) {
    /**
     * @brief Calculate the steering angle using pure pursuit or PID control
     * inputs: vehicle_state, driving_way
     * outputs: steering_angle
     * Purpose: Implement Pure Pursuit Control to calculate the steering angle based on the look-ahead point on the driving way
     */
                                              
    ////////////////////// TODO //////////////////////

    /***** Initialize variables */
    // Initialize Inputs
    double l_xd; // look-ahead distance [m]
    double g_x, g_y; // look-ahead point coordinates [m]
    double l_d; // distance between vehicle and look-ahead point [m]
    // Initialize Outputs
    double steering_angle = 0.0;

    // Step 0: Set look-ahead distance
    l_xd = 2.0;

    // Step 1: Get look-ahead point using look-ahead distance
    // (g_x g_y) = (x, ax^3 + bx^2 + cx + d)|x=l_xd
    g_x = l_xd;
    g_y = driving_way.a3*pow(g_x, 3) + driving_way.a2*pow(g_x, 2) + driving_way.a1*pow(g_x, 1) + driving_way.a0;

    // Step 2: The distance between vehicle position and look-ahead point
    l_d = pow(pow(g_x, 2) + pow(g_y, 2), 0.5);

    // Step 3: Calculate steering angle using Pure Pursuit formula
    // steering_angle = atan2(2 * L * e_ld / l_d^2)
    steering_angle = atan2(2*param_wheel_base_ * g_y, pow(l_d, 2));

    ////////////////////////////////////////////////////
   
    return steering_angle;
}

std::pair<double, double> AutonomousDriving::LongitudinalControl(const ad_msgs::msg::VehicleOutput &vehicle_state,
                                              const double &reference_speed) {
    /**
     * @brief Calculate the acceleration and brake commands using PID control
     * inputs: vehicle_state, reference_speed
     * outputs: accel_command, brake_command
     * Purpose: Implement PID control to compute acceleration and brake commands to follow the reference speed
     */
    // Initialize Outputs
    double accel_command = 0.0;
    double brake_command = 0.0;
    double dt = 0.01;

    ////////////////////// TODO //////////////////////

    // First, Initialize private member variables in autonomous_driving.hpp
    // [Error] Calculate speed error, cumulative error, and derivative error

    double speed_error = reference_speed - vehicle_state.velocity;
    speed_error_integral_ += speed_error;

    param_pid_kp_ = 3.0;
    param_pid_ki_ = 1.0;
    param_pid_kd_ = 2.0;
    
    double command = param_pid_kp_ * speed_error + param_pid_ki_ * speed_error_integral_ + param_pid_kd_ * (speed_error - speed_error_prev_) / dt;
    speed_error_prev_ = speed_error;
    if (command > 0) {
        accel_command = command;
    }
    else {
        brake_command = command;
    }
    // [PID Control] Calculate acceleration, brake commands using PID formula
    // Parameters of PID is initialized in autonomous_driving.hpp: param_pid_kp_, param_pid_ki_, param_pid_kd_

    // [Output] Set accel_command and brake_command values

    ///////////////////////////////////////////////////

    std::pair<double, double> accel_brake_command;
    accel_brake_command.first = accel_command;
    accel_brake_command.second = brake_command;

    return accel_brake_command;
}

void AutonomousDriving::Publish(const rclcpp::Time &current_time) {
    p_vehicle_command_->publish(o_vehicle_command_);
    p_driving_way_->publish(o_driving_way_);
    p_poly_lanes_->publish(o_poly_lanes_);
}

int main(int argc, char **argv) {
    std::string node_name = "autonomous_driving";
    double loop_rate = 100.0;

    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutonomousDriving>(node_name, loop_rate));
    rclcpp::shutdown();
    return 0;
}
