#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Bool.h"
#include "my_actuator/ang_lin_arr.h"
#include <std_srvs/Trigger.h>

class Joy_teleop
{
public:
    Joy_teleop();
    int lin_inc = 5;
    int ang_inc = 5;

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
    void toggleJoyCallback(const std_msgs::Bool::ConstPtr &msg);

    ros::NodeHandle nh_;
    int toggle_joy_flag_ = -1;
    int linear_, angular_;
    int l_inc_, a_inc_, l_dec_, a_dec_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
    ros::Subscriber toggle_joy_sub_;

    ros::Publisher vel_gui_pub_;
    my_actuator::ang_lin_arr gui_vel;
};

Joy_teleop::Joy_teleop() : linear_(3),
                                       angular_(0),
                                       l_inc_(3),
                                       a_inc_(0),
                                       l_dec_(1),
                                       a_dec_(2)
{

    nh_.param("axis_linear", linear_, linear_);
    nh_.param("axis_angular", angular_, angular_);
    // nh_.param("scale_angular", a_scale_, a_scale_);
    // nh_.param("scale_linear", l_scale_, l_scale_);

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("skid_steer/cmd_vel", 1);

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &Joy_teleop::joyCallback, this);
    toggle_joy_sub_ = nh_.subscribe<std_msgs::Bool>("/toggle_joy", 50, &Joy_teleop::toggleJoyCallback, this);
    vel_gui_pub_ = nh_.advertise<my_actuator::ang_lin_arr>("/vel_status", 1);
    ROS_WARN("teleop_joy NODE STARTED!!!");
}

void Joy_teleop::toggleJoyCallback(const std_msgs::Bool::ConstPtr &msg)
{

    toggle_joy_flag_ = msg->data;
}

void Joy_teleop::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    // ROS_INFO("toggle: %d craweler", toggle_joy_flag_);
    // if (joy->axes[6] == 1.0)  // D-pad keft
    //     {
    //         ROS_INFO("D-pad Right pressed. Calling ROS service for clockwise movement.");

    //         // Create a service client
    //         ros::ServiceClient anticlockwise_client = nh_.serviceClient<std_srvs::Trigger>("/anticlockwise");

    //         // Create a service request
    //         std_srvs::Trigger srv1;

    //         // Call the service
    //         if (anticlockwise_client.call(srv1))
    //         {
    //             // Successfully called the service, process the response if needed
    //             if (srv1.response.success)
    //             {
    //                 ROS_INFO("Service call successful");
    //             }
    //             else
    //             {
    //                 ROS_ERROR("Service call failed: %s", srv1.response.message.c_str());
    //             }
    //         }
    //         else
    //         {
    //             // Failed to call the service
    //             ROS_ERROR("Failed to call /clockwise service");
    //         }
    //     }
    //     if (joy->axes[6] == -1.0)  // D-pad Right
    //     {
    //         ROS_INFO("D-pad Right pressed. Calling ROS service for clockwise movement.");

    //         // Create a service client
    //         ros::ServiceClient clockwise_client = nh_.serviceClient<std_srvs::Trigger>("/clockwise");

    //         // Create a service request
    //         std_srvs::Trigger srv2;

    //         // Call the service
    //         if (clockwise_client.call(srv2))
    //         {
    //             // Successfully called the service, process the response if needed
    //             if (srv2.response.success)
    //             {
    //                 ROS_INFO("Service call successful");
    //             }
    //             else
    //             {
    //                 ROS_ERROR("Service call failed: %s", srv2.response.message.c_str());
    //             }
    //         }
    //         else
    //         {
    //             // Failed to call the service
    //             ROS_ERROR("Failed to call /clockwise service");
    //         }
    //     }
    //     if (joy->axes[7] == 1.0)  // D-pad Right
    //     {
    //         ROS_INFO("D-pad Right pressed. Calling ROS service for clockwise movement.");

    //         // Create a service client
    //         ros::ServiceClient add_three_client = nh_.serviceClient<std_srvs::Trigger>("/add_three_service");

    //         // Create a service request
    //         std_srvs::Trigger srv3;

    //         // Call the service
    //         if (add_three_client.call(srv3))
    //         {
    //             // Successfully called the service, process the response if needed
    //             if (srv3.response.success)
    //             {
    //                 ROS_INFO("Service call successful");
    //             }
    //             else
    //             {
    //                 ROS_ERROR("Service call failed: %s", srv3.response.message.c_str());
    //             }
    //         }
    //         else
    //         {
    //             // Failed to call the service
    //             ROS_ERROR("Failed to call /clockwise service");
    //         }
    //     }
    //     if (joy->axes[7] == -1.0)  
    //     {
    //         ROS_INFO("D-pad Right pressed. Calling ROS service for clockwise movement.");

            
    //         ros::ServiceClient reduce_three_client = nh_.serviceClient<std_srvs::Trigger>("/reduce_three_service");

    //         // Create a service request
    //         std_srvs::Trigger srv;

    //         // Call the service
    //         if (reduce_three_client.call(srv))
    //         {
    //             // Successfully called the service, process the response if needed
    //             if (srv.response.success)
    //             {
    //                 ROS_INFO("Service call successful");
    //             }
    //             else
    //             {
    //                 ROS_ERROR("Service call failed: %s", srv.response.message.c_str());
    //             }
    //         }
    //         else
    //         {
    //             // Failed to call the service
    //             ROS_ERROR("Failed to call /clockwise service");
    //         }
    //     }
    if (toggle_joy_flag_ == -1)
    {
        ROS_INFO("inside: %d craweler", toggle_joy_flag_);

        geometry_msgs::Twist twist;
        lin_inc = lin_inc + 1 * joy->buttons[l_inc_];
        ang_inc = ang_inc + 2 * joy->buttons[a_inc_];

        lin_inc = lin_inc - 1 * joy->buttons[l_dec_];
        ang_inc = ang_inc - 2 * joy->buttons[a_dec_];

        if (lin_inc <= 0)
        {
            lin_inc = 0;
        }

        if (ang_inc <= 0)
        {
            ang_inc = 0;
        }

        if (lin_inc >= 28)
        {
            lin_inc = 28;
        }

        if (ang_inc >= 15)
        {
            ang_inc = 15;
        }

        if (joy->buttons[l_inc_] || joy->buttons[a_inc_] || joy->buttons[l_dec_] || joy->buttons[a_dec_])
        {
            ROS_INFO("linear: %d , angular: %d", lin_inc, ang_inc);
        }
        
        twist.angular.z = ang_inc * joy->axes[angular_];
        twist.linear.x = lin_inc * joy->axes[linear_];
        
        gui_vel.data[0] = twist.linear.x;
        gui_vel.data[1] = twist.angular.z;
        gui_vel.data[2] = lin_inc;
        gui_vel.data[3] = ang_inc;
        vel_gui_pub_.publish(gui_vel);
        
        vel_pub_.publish(twist);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_joy");
    Joy_teleop teleop_skidsteer;

    ros::spin();
}
