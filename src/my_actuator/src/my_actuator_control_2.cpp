/*
 * -------------------------------------------- COPYRIGHT NOTICE ---------------------------------------------------
 * Copyright (C) 2023 Octobotics Tech Pvt. Ltd. All Rights Reserved.
 * Do not remove this copyright notice.
 * Do not use, reuse, copy, merge, publish, sub-license, sell, distribute or modify this code - except without explicit,
 * written permission from Octobotics Tech Pvt. Ltd.
 * Contact connect@octobotics.tech for full license information.
 * -------------------------------------------- COPYRIGHT NOTICE ---------------------------------------------------
 *
 *
 * @file my_actuator_control.cpp
 * @author Charith
 * @author2 grv
 * @brief
 * @date 2023-02-16
 *
 *
 */

#include "my_actuator/my_actuator_control.h"
#include <unistd.h>

my_actuator_control::my_actuator_control(ros::NodeHandle nh)
{
    /**
     * @brief Constructor
     * @return None
     */
    nh_ = nh;

    init_teleop_ = false;
    reset_tripmeter_flag_ = false;

    act1_enabled_ = false;
    act2_enabled_ = false;
    act3_enabled_ = false;
    act4_enabled_ = false;
    act1_sw_ = false;
    act2_sw_ = false;
    act3_sw_ = false;
    act4_sw_ = false;
    velocity_scale_ = 60;
    raster_speed_ = 150;
    motor_ids.reserve(4);

    error_[0] = "NO Error [Radhe Radhe]";
    error_[2] = "Motor Stalled [Critical]";
    error_[4] = "Low Pressure [Donno Dude]";
    error_[8] = " Over Voltage [Moderate]";
    error_[16] = "Over Current [Critical]";
    error_[64] = "Power Overrun [Moderate]";
    error_[256] = "Speeding [Moderate]";
    error_[512] = "Empty Error 1";
    error_[1024] = "Empty Error 2";
    error_[2048] = "Empty Error 3";
    error_[4096] = "Very High Temperature [Moderate]";
    error_[8192] = "Encoder Calibration Error [Moderate]";
    // health->resize(4);
    // ROS_INFO("%d: err: %s",64, error_[64].c_str());
    // cout<<error_[64];
    toggle_joy_flag_ = -1;
    // port_ = "/dev/crawler";
    port_ = "/dev/crawler";
    act_api_ = new X10ApiSerial();
    act_api_->get_port_address(port_);

    geometry_msgs::Twist vel;
    get_params();
    init_actuators();
    /// initialize the subscribers, publishers and services
    init_subs_pubs_srvs();

    joy_reader_timer_ = nh_.createTimer(ros::Duration(0.1), &my_actuator_control::steer_calc_callback_, this);
    //health_timer_ = nh_.createTimer(ros::Duration(0.1), &my_actuator_control::health_callback_, this);
    //speed_timer_ =  nh_.createTimer(ros::Duration(0.1), &my_actuator_control::get_speed, this);
    //distance_timer_ = nh_.createTimer(ros::Duration(0.1), &my_actuator_control::get_tripmeter, this);
    //odometer_timer_ = nh_.createTimer(ros::Duration(0.1), &my_actuator_control::get_odometer, this);
   // voltage_timer_ = nh_.createTimer(ros::Duration(1.0), &my_actuator_control::get_voltage, this);
    publish_timer_ = nh_.createTimer(ros::Duration(1.0), &my_actuator_control::publishRotationTime, this);
    scan_timer_ = nh_.createTimer(ros::Duration(1.0), &my_actuator_control::scanSpeed, this);
   // healthinfo_pub_timer_ = nh_.createTimer(ros::Duration(1.0), &my_actuator_control::get_health_info, this);
    //double last_save_time = ros::Time::now().toSec();
    start_time_ = ros::Time::now();
   // motor_state1_timer_ = nh_.createTimer(ros::Duration(0.1), [this](const ros::TimerEvent& event) { publishMotorState1(event); });
  //  motor_state2_timer_ = nh_.createTimer(ros::Duration(0.1), [this](const ros::TimerEvent& event) { publishMotorState2(event); });
  //  motor_state3_timer_ = nh_.createTimer(ros::Duration(0.1), [this](const ros::TimerEvent& event) { publishMotorState3(event); });
   // motor_state4_timer_ = nh_.createTimer(ros::Duration(0.1), [this](const ros::TimerEvent& event) { publishMotorState4(event); });
  //  bag1.open("motor_states1.bag", rosbag::bagmode::Write);
   // bag2.open("motor_states2.bag", rosbag::bagmode::Write);
   // bag3.open("motor_states3.bag", rosbag::bagmode::Write);
   // bag4.open("motor_states4.bag", rosbag::bagmode::Write);
    
}
my_actuator_control::~my_actuator_control()
{
    /**
     * @brief Destructor
     * @return None
     */
    // delete joy_reader_timer_;
    /// stop the timers
    act_api_->rmdX10_shut_down();
  //  bag1.close();
    //bag2.close();
    //g3.close();
    //bag4.close();

    ROS_INFO("%s: Shutting down", log_header_.c_str());
}
void my_actuator_control::get_params()
{
    /**
     * @brief Get the parameters from the parameter server
     * @return None
     */
    nh_.param<int>("/my_actuator/right_front_wheel/id", right_front_wheel_id_, 2);
    nh_.param<int>("/my_actuator/left_front_wheel/id", left_front_wheel_id_, 1);
    nh_.param<int>("/my_actuator/right_rear_wheel/id", right_rear_wheel_id_, 3);
    nh_.param<int>("/my_actuator/left_rear_wheel/id", left_rear_wheel_id_, 4);
    nh_.param<int>("/my_actuator/right_front_wheel/dir", right_front_wheel_dir_, 1);
    nh_.param<int>("/my_actuator/left_front_wheel/dir", left_front_wheel_dir_, 1);
    nh_.param<int>("/my_actuator/right_rear_wheel/dir", right_rear_wheel_dir_, 1);
    nh_.param<int>("/my_actuator/left_rear_wheel/dir", left_rear_wheel_dir_, 1);
    nh_.param<double>("/my_actuator/wheel_base", WHEEL_BASE_, 0.2913);
    nh_.param<double>("/my_actuator/wheel_radius", WHEEL_RADIUS_, 0.0525);
    /// nh_.param<int>("/my_actuator/velocity_scale", velocity_scale_, 40);

    if (!nh_.getParam("/my_actuator/right_front_wheel/id", right_front_wheel_id_))
    {
        right_front_wheel_id_ = 2;
        ROS_WARN("%s Right Front wheel motor id param not found, using the default value: %d", log_header_.c_str(),
                 right_front_wheel_id_);
    }
    if (!nh_.getParam("/my_actuator/left_front_wheel/id", left_front_wheel_id_))
    {
        left_front_wheel_id_ = 1;
        ROS_WARN("%s left Front wheel motor id param not found, using the default value: %d", log_header_.c_str(),
                 left_front_wheel_id_);
    }
    if (!nh_.getParam("/my_actuator/right_rear_wheel/id", right_rear_wheel_id_))
    {
        right_rear_wheel_id_ = 3;
        ROS_WARN("%s Right rear wheel motor id param not found, using the default value: %d", log_header_.c_str(),
                 right_rear_wheel_id_);
    }
    if (!nh_.getParam("/my_actuator/left_rear_wheel/id", left_rear_wheel_id_))
    {
        left_rear_wheel_id_ = 4;
        ROS_WARN("%s Left rear wheel motor id param not found, using the default value: %d", log_header_.c_str(),
                 left_rear_wheel_id_);
    }

    if (!nh_.getParam("/my_actuator/right_front_wheel/dir", right_front_wheel_dir_))
    {
        right_front_wheel_dir_ = -1;
        ROS_WARN("%s Right Front wheel motor dir param not found, using the default value: %d", log_header_.c_str(),
                 right_front_wheel_dir_);
    }
    if (!nh_.getParam("/my_actuator/left_front_wheel/dir", left_front_wheel_dir_))
    {
        left_front_wheel_dir_ = 1;
        ROS_WARN("%s left Front wheel motor dir param not found, using the default value: %d", log_header_.c_str(),
                 left_front_wheel_dir_);
    }
    if (!nh_.getParam("/my_actuator/right_rear_wheel/dir", right_rear_wheel_dir_))
    {
        right_rear_wheel_dir_ = -1;
        ROS_WARN("%s Right rear wheel motor dir param not found, using the default value: %d", log_header_.c_str(),
                 right_rear_wheel_dir_);
    }
    if (!nh_.getParam("/my_actuator/left_rear_wheel/dir", left_rear_wheel_dir_))
    {
        left_rear_wheel_dir_ = 1;
        ROS_WARN("%s Left rear wheel motor dir param not found, using the default value: %d", log_header_.c_str(),
                 left_rear_wheel_dir_);
    }

    if (!nh_.getParam("/my_actuator/wheel_base", WHEEL_BASE_))
    {
        WHEEL_BASE_ = 0.2913;
        ROS_WARN("%s Wheel Base param not found, using the default value: %f", log_header_.c_str(),
                 WHEEL_BASE_);
    }
    if (!nh_.getParam("/my_actuator/wheel_radius", WHEEL_RADIUS_))
    {
  //   void my_actuator_control::publishMotorState4(const ros::TimerEvent& event) {
//     int16_t motor_stat1[4] = {0};  // Initialize array
//     int16_t motor_stat3[4] = {0};

//     // Call the appropriate function from your X10ApiBase class
//     int8_t ret = act_api_->Motor_state1(4, motor_stat1);
//     ret = act_api_->Motor_state3(4, motor_stat3);

//     if (ret == 0) {
//         // Log motor state to ROS
//         // std::stringstream log_message;
//         // log_message << "Motor state1:" << std::endl;
//         // log_message << "\tTemperature----> " << motor_stat1[0] << std::endl;
//         // log_message << "\tVoltage--------> " << (float)motor_stat1[2] / 10.0 << std::endl;
//         // log_message << "\tCurrent----> " << motor_stat3[1] + motor_stat3[2] + motor_stat3[3] << std::endl;
//         // log_message << "-------------------------------------------------------";

//         // ROS_INFO_STREAM(log_message.str());
//         // std::cout << std::flush;  // Flush the output stream

//         // Publish motor state
//         std_msgs::Int16MultiArray motor_state4_msg;
//         motor_state4_msg.data = {motor_stat1[0], motor_stat1[2], (motor_stat3[1] + motor_stat3[2] + motor_stat3[3])*10};
//         motor_state4_publisher_.publish(motor_state4_msg);
//         bag4.write("motor_state4", ros::Time::now(), motor_state4_msg);
//     } else {
//         // Handle error or log it if needed
//         ROS_ERROR("Failed to get Motor_state1. Error code: %d", ret);
//     }

//     ros::spinOnce();  // Process callbacks
// }   WHEEL_RADIUS_ = 0.0525;
        ROS_WARN("%s Wheel Radius dir param not found, using the default value: %f", log_header_.c_str(),
                 WHEEL_RADIUS_);
    }

    motor_ids.push_back(left_front_wheel_id_);
    motor_ids.push_back(right_front_wheel_id_);
    motor_ids.push_back(right_rear_wheel_id_);
    motor_ids.push_back(left_rear_wheel_id_);
    sort(motor_ids.begin(), motor_ids.end());
    for (int8_t i = 0; i < 4; i++)
    {
        health.push_back(vit());
    }
    ROS_INFO("%d: Health:", health[0].temperature);
}
void my_actuator_control::init_subs_pubs_srvs()
{
    /**
     * @brief Initialize the subscribers, publishers, services and clients
     * @return none
     */
    // Publishers
    plot_pub_ = nh_.advertise<my_actuator::plot>("/plot", 10);
    vitals_pub_ = nh_.advertise<my_actuator::vitals>("/crawler_vitals", 10);
    runtime_pub_ = nh_.advertise<my_actuator::runtime>("/run_time", 10);

    speed_pub_ = nh_.advertise<std_msgs::Int16>("/motor_speed", 10);
    distance_pub_ = nh_.advertise<std_msgs::Int32>("/tripmeter", 10);
    odometer_pub_ = nh_.advertise<std_msgs::Int32>("/odometer", 10);
    voltage_pub_  = nh_.advertise<std_msgs::Float32>("/voltage", 10);
    time_pub_ = nh_.advertise<std_msgs::Float32>("/rotation_time", 10);
    scan_speed_pub_ = nh_.advertise<std_msgs::Int32>("/scan_speed", 10);    
    healthinfo_pub_ = nh_.advertise<my_actuator::healthinfo>("/healthinfo",10);   
// motor_state1_publisher_ = nh_.advertise<std_msgs::Int16MultiArray>("motor_state1", 10);
  //  motor_state2_publisher_ = nh_.advertise<std_msgs::Int16MultiArray>("motor_state2", 10);
  //  motor_state3_publisher_ = nh_.advertise<std_msgs::Int16MultiArray>("motor_state3", 10);
  //  motor_state4_publisher_ = nh_.advertise<std_msgs::Int16MultiArray>("motor_state4", 10);

    // Subscribers
    joy_sub_ = nh_.subscribe("/skid_steer/cmd_vel", 10, &my_actuator_control::joy_callback, this);
    joy_sub_real = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &my_actuator_control::get_joy, this);

    toggle_joy_topic_ = "/toggle_joy";
    toggle_joy_sub_ = nh_.subscribe(toggle_joy_topic_, 1,
                                    &my_actuator_control::toggle_joy_callback_, this);
    linear_motor_sub_ = nh_.subscribe("/linear_motor", 10, &my_actuator_control::linearMotorCallback, this);

    angle_value_sub_ = nh_.subscribe("/set_joint_angle_value", 10, &my_actuator_control::angleValueCallback, this);

    stroke_length_sub_ = nh_.subscribe("/stroke_length", 10, &my_actuator_control::strokeLengthCallback, this);

    manual_stroke_length_sub_ = nh_.subscribe("/manual_stroke_length", 10, &my_actuator_control::manual_stroke_length_callback, this);

    // voltage_sub_ = nh_.subscribe("/voltage", 10, &my_actuator_control::voltageCallback, this);


    // Services
    init_teleop_srv_ = nh_.advertiseService("/crawler_control_node/init_teleop",
                                            &my_actuator_control::init_teleop_callback, this);
    stop_teleop_srv_ = nh_.advertiseService("/crawler_control_node/stop_teleop",
                                            &my_actuator_control::stop_teleop_callback, this);
    stop_motors_srv_ = nh_.advertiseService("/crawler_control_node/stop_motors",
                                            &my_actuator_control::stop_motors_callback, this);

    reset_motors_srv_ = nh_.advertiseService("/crawler_control_node/reset_motors", &my_actuator_control::reset_motors_callback, this);

    reset_tripmeter_srv_ = nh_.advertiseService("/crawler_control_node/reset_tripmeter", &my_actuator_control::reset_tripmeter_callback, this);

    set_joint_angle_srv_ = nh_.advertiseService("/set_joint_angle", &my_actuator_control::set_joint_angle_callback, this);

    increase_raster_speed_srv_ = nh_.advertiseService("/increase_raster_speed", &my_actuator_control::increaseRasterSpeedCallback, this);

    decrease_raster_speed_srv_ = nh_.advertiseService("/decrease_raster_speed", &my_actuator_control::decreaseRasterSpeedCallback, this);

    toggle_relay_client_name = "/relay_toggle_channel";

    toggle_relay_client_ = nh_.serviceClient<stm_interface::RelayControl>(toggle_relay_client_name);

    cotrol_relay_.request.data = 9;

}

bool my_actuator_control::init_teleop_callback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
    /**
     * @brief Initialize the teleoperation
     * @param req
     * @type std_srvs::TriggerRequest
     * @param res
     * @type std_srvs::TriggerResponse
     * @return true
     */

    if (!init_teleop_)
    {
        // init_actuators();
        init_teleop_ = true;
        res.success = true;
        res.message = "teleop started";
    }
    else
    {
        res.message = "init is already done.";
    }

    return true;
}
bool my_actuator_control::stop_teleop_callback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
    /**
     * @brief Stop the teleoperation
     * @param req
     * @type std_srvs::TriggerRequest
     * @param res
     * @type std_srvs::TriggerResponse
     * @return true
     */

    init_teleop_ = false;
    ROS_INFO("%s Stopping teleop", log_header_.c_str());
    for (int8_t i = 0; i < 4; i++)
    {
        act_api_->Motor_stop(i + 1);
    }
    res.success = true;
    res.message = "teleop stopped";
    return true;
}

bool my_actuator_control::stop_motors_callback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
    /**
     * @brief Stop the teleoperation
     * @param req
     * @type std_srvs::TriggerRequest
     * @param res
     * @type std_srvs::TriggerResponse
     * @return true
     */

    ROS_INFO("%sStopping motors", log_header_.c_str());
    for (int8_t i = 0; i < 4; i++)
    {
        act_api_->Motor_stop(i + 1);
    }

    init_teleop_ = false;
    res.success = true;
    res.message = "teleop stopped";
    return true;
}
bool my_actuator_control::reset_motors_callback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
    ROS_INFO("Step 0");
    ros::Duration(3).sleep();
    cotrol_relay_.request.data = 1;
    toggle_relay_client_.call(cotrol_relay_);
    // sleep for 1 second
    ROS_INFO("crawler off");
    ros::Duration(5).sleep();
    toggle_relay_client_.call(cotrol_relay_);
    ROS_INFO("crawler on");
    ros::Duration(3).sleep();
    //bag.close();
    ROS_INFO("%sNode killed", log_header_.c_str());
    init_teleop_ = false;

    delete act_api_;
    // delete joy_reader_timer_;
    // delete health_timer_;
    // //free(actuator_status_timer_);
    // //ros::shutdown();
    ROS_INFO("%sNode killed", log_header_.c_str());
    return true;
}

bool my_actuator_control::set_joint_angle_callback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
    /**
     * @brief Stop the teleoperation
     * @param req
     * @type std_srvs::TriggerRequest
     * @param res
     * @type std_srvs::TriggerResponse
     * @return true
     */
    uint8_t motor_ids[] = {0x01, 0x02, 0x03, 0x04};
    
   /** for (uint8_t motor_id : motor_ids) {
        int steps;
        if (motor_id == 0x01 || motor_id == 0x04) {
            steps = -angle_value_ * 1.0909091 * 100;
        } else {
            steps = angle_value_ * 1.0909091 * 100;
        }
        act_api_->increment_control(motor_id, 900, -steps);
    }**/

    for (uint8_t motor_id : motor_ids) {
        int steps = (motor_id == 0x02) ? -angle_value_ * 1.0909091 * 100 : angle_value_ * 1.0909091 * 100;
        act_api_->increment_control(motor_id, 100, steps);
    }
    
    res.success = true;
    res.message = "joint angle set";
    return true;
}


bool my_actuator_control::reset_tripmeter_callback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
    if (!reset_tripmeter_flag_)
    {
        // init_actuators();
        reset_tripmeter_flag_ = true;
        res.success = true;
        res.message = "tripmeter reset";
    }
    

    return true;
}

bool my_actuator_control::increaseRasterSpeedCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    if (raster_speed_ + 75 <= 900) {
        raster_speed_ += 75; 
        res.success = true;
        res.message = "Raster speed increased to " + std::to_string(raster_speed_);
        //publishRotationTime();
    } else {
        res.success = false;
        res.message = "Raster speed is already at or above maximum value (900).";
    }
    return true;
}

bool my_actuator_control::decreaseRasterSpeedCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    if (raster_speed_ - 75 >= 75) {
        raster_speed_ -= 75;  
        res.success = true;
        res.message = "Raster speed decreased to " + std::to_string(raster_speed_);
        //publishRotationTime();
    } else {
        res.success = false;
        res.message = "Raster speed is already at or below minimum value (75).";
    }
    return true;
}


void my_actuator_control::init_actuators()
{
    /**
     * @brief Initialize the actuators
     * @return none
     */

    int setVEL, res;
    act_api_->rmdX10_init();
}
void my_actuator_control::stop_actuators(int id)
{
    /**
     * @brief Enable the actuator of given id
     * @param id, id of the actuator
     * @type int
     * @return none
     */
    if (init_teleop_)ros::Subscriber angle_value_sub_;
    {
    }
}
void my_actuator_control::run_actuator(int id, double curr_vel)
{
    if (init_teleop_)
    {
        act_api_->speedControl(id, (int)curr_vel);
    }
}

void my_actuator_control::joy_callback(const geometry_msgs::Twist &msg)
{

    vel.linear.x = msg.linear.x;
    vel.angular.z = msg.angular.z;
}
void my_actuator_control::toggle_joy_callback_(const std_msgs::Bool::ConstPtr &msg)
{

    toggle_joy_flag_ = msg->data;
}
void my_actuator_control::get_joy(const sensor_msgs::Joy::ConstPtr &joy)
{
    /**
     * @brief Callback for the joystick
     * @param joy
     * @type sensor_msgs::Joy
     * @return none
     */

    if (init_teleop_ && toggle_joy_flag_ == -1)
    {

        if (joy->buttons[JoyStick::ACT1_2_ON_OFF])
        {
            act1_sw_ = !act1_sw_;
            act2_sw_ = !act2_sw_;
        }
        else if (joy->buttons[JoyStick::ACT3_4_ON_OFF])
        {
            act3_sw_ = !act3_sw_;
            act4_sw_ = !act4_sw_;
        }
    }
}
void my_actuator_control::steer_calc_callback_(const ros::TimerEvent &)
{
    /**
     * @brief ROS timer thread callback for reading joy state and calling the corresponding methods
     * @param not used
     * @type ros::TimerEvent
     * @return none
     */
    if (init_teleop_)
    {

        if (vel.linear.x == 0 && vel.angular.z == 0)
        {

            velocity_left_cmd_ = 0;
            velocity_right_cmd_ = 0;
        }
        else
        {
            velocity_left_cmd_ = ((vel.linear.x - vel.angular.z * WHEEL_BASE_ / 2.0) / WHEEL_RADIUS_) * velocity_scale_;
            ROS_INFO("Speed_Left_Commmand%d",velocity_left_cmd_);

            velocity_right_cmd_ = ((vel.linear.x + vel.angular.z * WHEEL_BASE_ / 2.0) / WHEEL_RADIUS_) * velocity_scale_;
            ROS_INFO("Speed_Right_Command%d",velocity_right_cmd_);
        }

        if ((act1_sw_ == true && act2_sw_ == true) && (act3_sw_ == true && act4_sw_ == true))
        {
            act_api_->Motor_shut_down(left_front_wheel_id_);
            act_api_->Motor_shut_down(right_front_wheel_id_);
            act_api_->Motor_shut_down(left_rear_wheel_id_);
            act_api_->Motor_shut_down(right_rear_wheel_id_);
            // ROS_WARN("ALL shutdown.");
        }
        else if (act1_sw_ == true && act2_sw_ == true)
        {
            act_api_->Motor_shut_down(left_front_wheel_id_);
            act_api_->Motor_shut_down(right_front_wheel_id_);
            run_actuator(right_rear_wheel_id_, velocity_right_cmd_ * right_rear_wheel_dir_);
            run_actuator(left_rear_wheel_id_, velocity_left_cmd_ * left_rear_wheel_dir_);

            // ROS_INFO("In act1");
            // ROS_INFO("In act2");
        }
        else if (act3_sw_ == true && act4_sw_ == true)
        {
            act_api_->Motor_shut_down(left_rear_wheel_id_);
            act_api_->Motor_shut_down(right_rear_wheel_id_);
            run_actuator(left_front_wheel_id_, velocity_left_cmd_ * left_front_wheel_dir_);
            run_actuator(right_front_wheel_id_, velocity_right_cmd_ * right_front_wheel_dir_);
            // ROS_INFO("In act3");
            // ROS_INFO("In act4");
        }
        else
        {
            run_actuator(left_front_wheel_id_, velocity_left_cmd_ * left_front_wheel_dir_);
            run_actuator(right_front_wheel_id_, velocity_right_cmd_ * right_front_wheel_dir_);
            run_actuator(right_rear_wheel_id_, velocity_right_cmd_ * right_rear_wheel_dir_);
            run_actuator(left_rear_wheel_id_, velocity_left_cmd_ * left_rear_wheel_dir_ );
            // ROS_WARN("NORMAL drive");
        }
    }
}

void my_actuator_control::angleValueCallback(const std_msgs::Int32::ConstPtr &msg) {
    angle_value_ = msg->data;
}

void my_actuator_control::strokeLengthCallback(const std_msgs::Int16::ConstPtr& msg) {
    stroke_length_ = msg->data;
    rotation_value_ = static_cast<int>(msg->data * 2.98 * 100);
    //ROS_INFO("Received stroke length: %d, calculated rotation_value_: %d", msg->data, rotation_value_);
}


void my_actuator_control::linearMotorCallback(const std_msgs::Int32::ConstPtr& msg) {
    switch (msg->data) {
        case 1:
            clockwise();
            break;
        case 2:
            anticlockwise();
            break;
        case 3:
            stop();
            break;
        default:
            ROS_WARN("Received unknown command: %d", msg->data);
            break;
    }
}

void my_actuator_control::manual_stroke_length_callback(const std_msgs::Int16::ConstPtr& msg) {
    switch (msg->data) {
        case 1:
            manual_clockwise();
            break;
        case 2:
            manual_anticlockwise();
            break;
        case 3:
            manual_stop();
            break;
        default:
            ROS_WARN("Received unknown command: %d", msg->data);
            break;
    }
}

void my_actuator_control::clockwise() {
    act_api_->increment_control(0x05, raster_speed_, rotation_value_);
    //ROS_INFO("Performing clockwise action with raster_speed_: %d and rotation_value_: %d", raster_speed_, rotation_value_);
}

void my_actuator_control::anticlockwise() {
    act_api_->increment_control(0x05, raster_speed_, -rotation_value_);
    //ROS_INFO("Performing anticlockwise action with raster_speed_: %d and rotation_value_: %d", raster_speed_, rotation_value_);
}


void my_actuator_control::stop() {
    
    ROS_INFO("Action 3 performed.");
}


void my_actuator_control::manual_clockwise() {
    act_api_->speedControl(0x05, 10000);
    ROS_INFO("Performing clockwise action with raster_speed_: %d and rotation_value_: %d", raster_speed_, rotation_value_);
}

void my_actuator_control::manual_anticlockwise() {
    act_api_->speedControl(0x05, -10000);
    ROS_INFO("Performing anticlockwise action with raster_speed_: %d and rotation_value_: %d", raster_speed_, rotation_value_);
}


void my_actuator_control::manual_stop() {
    act_api_->speedControl(0x05, 0);
    ROS_INFO("Action 3 performed.");
}

// void my_actuator_control::voltageCallback(const std_msgs::Float32::ConstPtr& msg) {
//         ros::Time current_time = ros::Time::now();
//         ros::Duration runtime = current_time - start_time_;

//         float voltage = msg->data;
//         float voltage_percentage = (voltage - 37) / (39.7 - 37) * 100;

//         my_actuator::runtime run_time_msg;
//         run_time_msg.voltage_percentage = voltage_percentage;
//         run_time_msg.runtime_seconds = runtime.toSec();
//         run_time_msg.runtime_minutes = runtime.toSec() / 60.0;
//         run_time_msg.runtime_hours = runtime.toSec() / 3600.0;
//         run_time_msg.voltage = msg->data;

//         runtime_pub_.publish(run_time_msg);
//     }

/*
void my_actuator_control::manual_clockwise() {
    // Use a lambda function to capture class members and run the thread
    std::thread clockwise_thread([this]() {
        act_api_->speedControl(0x05, 10000);  // Call the speed control function
        ROS_INFO("Performing clockwise action with raster_speed_: %d and rotation_value_: %d", raster_speed_, rotation_value_);
    });

    clockwise_thread.detach();  // Detach the thread so it runs independently
}

void my_actuator_control::manual_anticlockwise() {
    std::thread anticlockwise_thread([this]() {
        act_api_->speedControl(0x05, -10000);
        ROS_INFO("Performing anticlockwise action with raster_speed_: %d and rotation_value_: %d", raster_speed_, rotation_value_);
    });

    anticlockwise_thread.detach();
}

void my_actuator_control::manual_stop() {
    std::thread stop_thread([this]() {
        act_api_->speedControl(0x05, 0);
        ROS_INFO("Action 3 performed.");
    });

    stop_thread.detach();
}
*/


// void my_actuator_control::get_speed(const ros::TimerEvent &)
// {
//     int16_t motor_stat2[4] = {0};
    
    
//     if(init_teleop_)
//     {
//         act_api_->Motor_state2(1, motor_stat2);
        
//         // if(motor_stat2[2] < 0)
//         // {
//         //     //int velocity = motor_stat2[2] * -1 * 0.166667 * 0.1047 * 52.5 * 1000;
//         //     speed_msg.data = motor_stat2[2] * -1;
//         //     speed_pub_.publish(speed_msg);
//         // }
//         // else
//         // {
//         //     //int velocity = motor_stat2[2] * 0.166667 * 0.1047 * 52.5 * 1000;
//         //     speed_msg.data = motor_stat2[2];
//         //     speed_pub_.publish(speed_msg);
//         // }
//         //ROS_INFO("Debug: motor_stat[2] = %d", motor_stat2[2]);
//         std_msgs::Int16 speed_msg;
//         int16_t velocity = std::abs(motor_stat2[2]) * 0.16667 * 3.14 * 105 / 60;
//         speed_msg.data = velocity;
//         speed_pub_.publish(speed_msg); 
         
//     }

    
// }

// void my_actuator_control::get_voltage(const ros::TimerEvent &)
// {
//     int16_t motor_stat1[4] = {0};
    
//     act_api_->Motor_state1(1, motor_stat1);
    
//     std_msgs::Float32 voltage_msg;  
//     float voltage = static_cast<float>(motor_stat1[2]) / 10.0;  
//     voltage_msg.data = voltage;
//     voltage_pub_.publish(voltage_msg);
// }

// void my_actuator_control::get_tripmeter(const ros::TimerEvent &)
// {
//     int16_t motor_stat2[4] = {0};
   
//     int32_t accumulated_distance;
    
//     //int32_t distance = 0;
    
//     //ros::Time last_time = ros::Time::now();
//     if(init_teleop_)
//     {
    
    
        
//         act_api_->Motor_state2(2, motor_stat2);
    
//         int32_t velocity = std::abs(motor_stat2[2]); 
//         //ros::Time current_time = ros::Time::now();
//         //ros::Duration dt = current_time - last_time;
        
    


        
//          int32_t distance = (velocity * 2 * 3.14 * 52.5 * 0.1667 / 60);

        

//         accumulated_distance = accumulated_distance + distance; 
        
//         std_msgs::Int32 distance_msg;
        
        
//         distance_msg.data = accumulated_distance;
//         distance_pub_.publish(distance_msg);
        

//         //last_time = current_time;
       
//     }
    
    
// }

void my_actuator_control::publishRotationTime(const ros::TimerEvent& event) {
    std_msgs::Float32 time_msg;

    
    if (raster_speed_ != 0) {
        time_msg.data = (static_cast<float>(stroke_length_) / (raster_speed_ / 3.0f)) + 0.5f;  
    } else {
        time_msg.data = 0.0f;
        //ROS_WARN("Raster speed is zero, unable to calculate rotation time.");
    }

    time_pub_.publish(time_msg);
    //ROS_INFO("Published rotation time: %.2f seconds for raster_speed_: %d", time_msg.data, raster_speed_);
}


void my_actuator_control::scanSpeed(const ros::TimerEvent& event)
{
    std_msgs::Int32 speed_msg;
    speed_msg.data = static_cast<int>(raster_speed_);
    scan_speed_pub_.publish(speed_msg);
}

// void my_actuator_control::get_odometer(const ros::TimerEvent &)
// {
//     static double last_save_time = ros::Time::now().toSec();
//     static double cumulative_distance = 0.0;
//     static double accumulated_distance = 0.0;
//     static std::fstream file("/home/sensei/catkin_ws/src/my_actuator/last_distance.txt", std::ios::in | std::ios::out);

//     int16_t motor_stat[4] = {0};

    
//     file >> cumulative_distance;

//     std_msgs::Int32 cumulative_distance_msg;
//     cumulative_distance_msg.data = static_cast<int32_t>(cumulative_distance);
//     odometer_pub_.publish(cumulative_distance_msg);

//     while (ros::ok())
//     {
//         act_api_->Motor_state2(2, motor_stat);
    
//         int32_t velocity = std::abs(motor_stat[2]); 
//         int32_t distance = velocity * 2 * 3.14 * 52.5 * 0.1667 / 60;
        
//         cumulative_distance += static_cast<int32_t>(distance);
//         accumulated_distance += static_cast<int32_t>(distance);

//         if(reset_tripmeter_flag_)
//         {
//             accumulated_distance = 0;
//             reset_tripmeter_flag_ = false;
//         }
        
//         std_msgs::Int32 distance_msg;
//         distance_msg.data = static_cast<int32_t>(accumulated_distance );
//         distance_pub_.publish(distance_msg);

//         cumulative_distance_msg.data = static_cast<int32_t>(cumulative_distance);
//         odometer_pub_.publish(cumulative_distance_msg);

        
//         if (ros::Time::now().toSec() - last_save_time >= 2.0) {
//             file.seekp(0); 
//             file << cumulative_distance << std::endl;
//             file.flush();
//             last_save_time = ros::Time::now().toSec();
//         }

//         ros::spinOnce();
//         ros::Duration(0.1).sleep();
//     }
// }





// void my_actuator_control::health_callback_(const ros::TimerEvent &)
// {
//     uint8_t mode_var;
//     for (auto &it : motor_ids)
//     {
//         mode_var = -1;
//         get_vitals(it);
//         send_vitals.temp[it - 1] = health[it - 1].temperature;
//         send_vitals.error[it - 1] = health[it - 1].error;
//         send_vitals.current[it - 1] = (health[it - 1].current)*0.01;

//         int ret = act_api_->Motor_mode(it, mode_var);
//         // ROS_WARN("Motor %d: %u", it, mode_var);
//         if (ret)
//             send_vitals.mode[it - 1] = mode_var;
//     }
//     send_vitals.voltage = health[0].voltage;
//     send_vitals.header.frame_id = "crawler";
//     vitals_pub_.publish(send_vitals);
// }

// void my_actuator_control::get_vitals(int id_)
// {
//     int ret = 0;
//     int16_t error_ret_[4];
//     // 0: temp | 2: voltage | 3: error
//     ret = act_api_->Motor_state1(id_, error_ret_);
//     if (ret)
//     {
//         health[id_ - 1].temperature = (int)error_ret_[0];
//         health[id_ - 1].voltage = (float)error_ret_[2] / 10.0;
//         health[id_ - 1].error = (int)error_ret_[3];
//     }

//     // 1: current
//     ret = act_api_->Motor_state2(id_, error_ret_);
//     if (ret)
//     {
//         health[id_ - 1].current = error_ret_[1];
//     }
// }


// void my_actuator_control::get_health_info(const ros::TimerEvent& event) {
    
//         // for (int i=1; i<=5 ; i++){
//         //     ret = act_api_->Motor_state1(i,error_ret_);
//         //     act_api_->Motor_state3(i,current_info_);

//         //     healthinfo_msg_array.temp[i-1] = error_ret_[0];
//         //     healthinfo_msg_array.error[i-1] = ret;
//         //     healthinfo_msg_array.current[i-1] = (current_info_[1]+current_info_[2]+current_info_[3])*0.1;
//         //     healthinfo_msg_array.voltage[i-1] = error_ret_[2] / 10.0;
//         //     sleep(1);
//         // }
//         // healthinfo_msg_array.header.frame_id = "crawler";
//         // healthinfo_pub_.publish(healthinfo_msg_array);

//         // ret = act_api_->Motor_state1(1,error_ret_);
//         //     act_api_->Motor_state3(1,current_info_);

//         //     healthinfo_msg_array.temp[1-1] = error_ret_[0];
//         //     healthinfo_msg_array.error[1-1] = ret;
//         //     healthinfo_msg_array.current[1-1] = (current_info_[1]+current_info_[2]+current_info_[3])*0.1;
//         //     healthinfo_msg_array.voltage[1-1] = error_ret_[2] / 10.0;



//         //     ret = act_api_->Motor_state1(2,error_ret_);
//         //     act_api_->Motor_state3(2,current_info_);

//         //     healthinfo_msg_array.temp[2-1] = error_ret_[0];
//         //     healthinfo_msg_array.error[2-1] = ret;
//         //     healthinfo_msg_array.current[2-1] = (current_info_[1]+current_info_[2]+current_info_[3])*0.1;
//         //     healthinfo_msg_array.voltage[2-1] = error_ret_[2] / 10.0;



//         //     ret = act_api_->Motor_state1(3,error_ret_);
//         //     act_api_->Motor_state3(3,current_info_);

//         //     healthinfo_msg_array.temp[3-1] = error_ret_[0];
//         //     healthinfo_msg_array.error[3-1] = ret;
//         //     healthinfo_msg_array.current[3-1] = (current_info_[1]+current_info_[2]+current_info_[3])*0.1;
//         //     healthinfo_msg_array.voltage[3-1] = error_ret_[2] / 10.0;



//         //     ret = act_api_->Motor_state1(4,error_ret_);
//         //     act_api_->Motor_state3(4,current_info_);

//         //     healthinfo_msg_array.temp[4-1] = error_ret_[0];
//         //     healthinfo_msg_array.error[4-1] = ret;
//         //     healthinfo_msg_array.current[4-1] = (current_info_[1]+current_info_[2]+current_info_[3])*0.1;
//         //     healthinfo_msg_array.voltage[4-1] = error_ret_[2] / 10.0;


//         //     ret = act_api_->Motor_state1(5,error_ret_);
//         //     act_api_->Motor_state3(5,current_info_);

//         //     healthinfo_msg_array.temp[5-1] = error_ret_[0];
//         //     healthinfo_msg_array.error[5-1] = ret;
//         //     healthinfo_msg_array.current[5-1] = (current_info_[1]+current_info_[2]+current_info_[3])*0.1;
//         //     healthinfo_msg_array.voltage[5-1] = error_ret_[2] / 10.0;


            
//         // healthinfo_msg_array.header.frame_id = "crawler";
//         // healthinfo_pub_.publish(healthinfo_msg_array);
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "crawler_control_node_2");
    ROS_INFO("[CrawlerTeleOpNode]: Starting Node");
    ros::NodeHandle nh;
    my_actuator_control crawler_teleop(nh);

    try
    {
        ros::spin();
    }
    catch (std::exception &e)
    {
        ROS_ERROR("%sException thrown: %s", crawler_teleop.log_header_.c_str(), e.what());
    }
    return 0;
}
