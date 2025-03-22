/**
 * -------------------------------------------- COPYRIGHT NOTICE ---------------------------------------------------
 * Copyright (C) 2023 Octobotics Tech Pvt. Ltd. All Rights Reserved.
 * Do not remove this copyright notice.
 * Do not use, reuse, copy, merge, publish, sub-license, sell, distribute or modify this code - except without explicit,
 * written permission from Octobotics Tech Pvt. Ltd.
 * Contact connect@octobotics.tech for full license information.
 * -------------------------------------------- COPYRIGHT NOTICE ---------------------------------------------------
 *
 *
 * @file my_actuator_control.h
 * @author Charith
 * @brief
 * @date 2023-02-15
 *
 *
 */

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"
#include <cstdlib>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <bits/stdc++.h>
#include <stdint.h>
#include "/home/track_sense/catkin_ws/src/my_actuator/RmdApiSerial/rmd-sdk/include/x10_api.h"
#include "my_actuator/plot.h"
#include "my_actuator/vitals.h"
#include "my_actuator/healthinfo.h"
#include "my_actuator/runtime.h"
#include <cmath>
#include <sstream>
#include <fstream>



#include <rosbag/bag.h>
#include <rosbag/view.h>


#include "stm_interface/RelayControl.h"
using namespace std;
enum JoyStick
{
    /// axes
    FORWARD_BACKWARD = 3,
    LEFT_RIGHT = 0,

    // buttons
    ACT1_2_ON_OFF = 4,
    ACT3_4_ON_OFF = 5
};

struct vit
{
    float voltage;
    int error;
    int temperature;
    int16_t current;
};

class my_actuator_control
{

public:
    my_actuator_control(ros::NodeHandle nh_);
    // struct vitals k;

    ~my_actuator_control();

    /**
     * @brief Get the parameters from the parameter server
     * @return none
     */
    void get_params();

    /**
     * @brief Initialize the subscribers, publishers, services
     * @return none
     */
    void init_subs_pubs_srvs();

    /**
     * @brief allback for the init teleop service
     * @return none
     */
    bool init_teleop_callback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

    /**
     * @brief Callback for the stop teleop service
     * @param req, request message
     * @param res, response message
     * @return success or failure
     */
    bool stop_teleop_callback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
    /**
     * @brief Callback for the stop motors service
     * @param req, request message
     * @param res, response message
     * @return success or failure
     */

    bool stop_motors_callback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
    /**
     * @brief Initialize the actuators
     * @return none
     */

    bool reset_motors_callback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
    bool reset_tripmeter_callback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
    bool set_joint_angle_callback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
    bool increaseRasterSpeedCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool decreaseRasterSpeedCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    void get_speed();

    void get_tripmeter();

    void get_odometer();
    // void publishMotorState1();
    // void publishMotorState2();
    // void publishMotorState3();
    // void publishMotorState4();
    void init_actuators();
    /**
     * @brief Stops the actuator with the given id
     * @param id, id of the actuator
     * @return none
     */
    void stop_actuators(int id);
    /**
     * @brief Run the actuator with the given id at the given velocity
     * @param id, id of the actuator
     * @param vel, direction of the actuator to be multiplied with the current velocity
     * @return none
     */
    void run_actuator(int id, double vel);
    /**
     * @brief Callback for the joystick
     * @param joy, joystick message
     * @return none
     */
    void joy_callback(const geometry_msgs::Twist &msg);
    /**
     * @brief Callback to get joystick data
     * @param msg, joint state message
     * @return none
     */

    void get_joy(const sensor_msgs::Joy::ConstPtr &joy);
    /**
     * @brief Callback for the actuator state
     * @param msg, joint state message
     * @return none
     */
    void actuator_state_callback(const sensor_msgs::JointState::ConstPtr &msg);

    /**
     * @brief Callback for the init teleop service
     * @param req, request message
     * @param res, response message
     * @return success or failure
     */

    void toggle_joy_callback_(const std_msgs::Bool::ConstPtr &msg);

    /**
     * @brief Checks Temperature,voltage,error of the motor
     * @param id, motor ID
     * @return none
     */
    void get_vitals(int id_);

    /**
     * @brief Checks Torque_current,Speed,Motor_angle of the motor
     * @param id,
     * @return none
     */
    void motor_status();
    void get_voltage();
    //LinearMotorController();
    void linearMotorCallback(const std_msgs::Int32::ConstPtr& msg);
    void angleValueCallback(const std_msgs::Int32::ConstPtr &msg);
    void strokeLengthCallback(const std_msgs::Int16::ConstPtr& msg);
    void manual_stroke_length_callback(const std_msgs::Int16::ConstPtr& msg);
    void voltageCallback(const std_msgs::Float32::ConstPtr& msg);
    void publishRotationTime();
    void clockwise();
    void anticlockwise();
    void stop();
    void manual_clockwise();
    void manual_anticlockwise();
    void manual_stop();
    void scanSpeed();
    

    /// velocity control parameters
    double curr_vel;

    /// log header string
    std::string log_header_;

    bool init_teleop_;
    bool reset_tripmeter_flag_;
    double last_save_time;
    geometry_msgs::Twist vel;

    stm_interface::RelayControl cotrol_relay_;

    std_srvs::Trigger reset_srv_var;  // Trigger Service to Reset variables

private:
    /// node specific parameters
    ros::NodeHandle nh_;
    std::string port_;
    X10ApiSerial *act_api_;

    /// publishers
    ros::Publisher vitals_pub_;
    ros::Publisher plot_pub_;
    ros::Publisher speed_pub_;
    ros::Publisher distance_pub_;
    ros::Publisher odometer_pub_;
    ros::Publisher voltage_pub_;
    ros::Publisher time_pub_;
    ros::Publisher scan_speed_pub_;
    ros::Publisher healthinfo_pub_;	
    // ros::Publisher motor_state1_publisher_;
    // ros::Publisher motor_state2_publisher_;
    // ros::Publisher motor_state3_publisher_;
    // ros::Publisher motor_state4_publisher_;
    /// subscribers
    ros::Subscriber joy_sub_;
    ros::Subscriber actuator_states_sub_;
    ros::Subscriber joy_sub_real;
    std::string toggle_joy_topic_;
    ros::Subscriber toggle_joy_sub_;
    ros::Subscriber linear_motor_sub_;
    ros::Subscriber angle_value_sub_;
    ros::Subscriber stroke_length_sub_;
    ros::Subscriber manual_stroke_length_sub_;
    ros::Subscriber voltage_sub_;
    ros::Publisher runtime_pub_;
    

    

    /// services
    ros::ServiceServer init_teleop_srv_;
    ros::ServiceServer stop_teleop_srv_;
    ros::ServiceServer stop_motors_srv_;
    ros::ServiceServer reset_motors_srv_;
    ros::ServiceServer reset_tripmeter_srv_;
    ros::ServiceServer set_joint_angle_srv_;
    ros::ServiceServer increase_raster_speed_srv_;
    ros::ServiceServer decrease_raster_speed_srv_;

    //client
    std::string toggle_relay_client_name;
    ros::ServiceClient toggle_relay_client_;

    vector<int> motor_ids;
    vector<vit> health;

    my_actuator::vitals send_vitals;
       my_actuator::healthinfo healthinfo_msg_array;	
    map<int, string> error_;

    // int16_t arr1[4];
    int toggle_joy_flag_;

    int velocity_scale_;

    int right_front_wheel_id_;
    int left_front_wheel_id_;
    int right_rear_wheel_id_;
    int left_rear_wheel_id_;

    int right_front_wheel_dir_;
    int left_front_wheel_dir_;
    int right_rear_wheel_dir_;
    int left_rear_wheel_dir_;
    int velocity_scake;
    int angle_value_;
    int raster_speed_;
    int rotation_value_;
    int stroke_length_; 
   
    /// actuator enable/disable flags
    bool act1_sw_;
    bool act2_sw_;
    bool act3_sw_;
    bool act4_sw_;
    bool act1_enabled_;
    bool act2_enabled_;
    bool act3_enabled_;
    bool act4_enabled_;

    float velocity_left_cmd_;
    float velocity_right_cmd_;

    double WHEEL_BASE_;
    double WHEEL_RADIUS_;

     int ret;
    int16_t error_ret_[5];
    int16_t current_info_[5];

    ros::Timer joy_reader_timer_;
    ros::Timer actuator_status_timer_;
    ros::Timer health_timer_;
    ros::Timer distance_timer_;
    ros::Timer odometer_timer_;
    ros::Time start_time_;
    
    // ros::Timer motor_state1_timer_;
    // ros::Timer motor_state2_timer_;
    // ros::Timer motor_state3_timer_;
    // ros::Timer motor_state4_timer_;
    ros::Timer speed_timer_;
    ros::Timer voltage_timer_;
    ros::Timer publish_timer_;
    ros::Timer scan_timer_;
    ros::Timer healthinfo_pub_timer_;
    void steer_calc_callback_(const ros::TimerEvent &event);
    void actuator_status_callback_(const ros::TimerEvent &event);
    void health_callback_(const ros::TimerEvent &event);
    // void publishMotorState1(const ros::TimerEvent& event);
    // void publishMotorState2(const ros::TimerEvent& event);
    // void publishMotorState3(const ros::TimerEvent& event);
    // void publishMotorState4(const ros::TimerEvent& event);
    void get_speed(const ros::TimerEvent &event);
    void get_tripmeter(const ros::TimerEvent &event);
    void get_odometer(const ros::TimerEvent &event);
    void get_voltage(const ros::TimerEvent &event);
    void publishRotationTime(const ros::TimerEvent &event);
    void scanSpeed(const ros::TimerEvent &event);
    void get_health_info(const ros::TimerEvent &event);    

    

    // rosbag::Bag bag1;
    // rosbag::Bag bag2;
    // rosbag::Bag bag3;
    // rosbag::Bag bag4;

    rosbag::Bag bag;
};
