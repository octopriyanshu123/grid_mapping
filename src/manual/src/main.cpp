#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>


class JoyToMotorDirection
{
public:
    JoyToMotorDirection()
    {
        joy_sub_ = nh_.subscribe("/joy", 10, &JoyToMotorDirection::joyCallback, this);
        motor_dir_pub_ = nh_.advertise<std_msgs::Int8>("/motor_direction", 10);
        linear_motor_pub_ = nh_.advertise<std_msgs::Int16>("/manual_stroke_length", 10);
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
    {
        if (joy_msg->axes.size() > 5)  
        {
            std_msgs::Int8 motor_direction_msg;

            if (joy_msg->axes[5] == 1.0)
            {
                motor_direction_msg.data = 1;  
                motor_dir_pub_.publish(motor_direction_msg);
            }
            else if (joy_msg->axes[5] == -1.0)
            {
                motor_direction_msg.data = 2;  
                motor_dir_pub_.publish(motor_direction_msg);
            }
            else if (joy_msg->axes[5] == 0.0)
            {
                motor_direction_msg.data = 0; 
                motor_dir_pub_.publish(motor_direction_msg);
            }
        }

        if (joy_msg->axes.size() > 4)  
        {
            std_msgs::Int16 linear_motor_msg;

            if (joy_msg->axes[4] == -1.0)
            {	stop_flag = true;
                linear_motor_msg.data = 2;  
                linear_motor_pub_.publish(linear_motor_msg);
            }
            else if (joy_msg->axes[4] == 1.0)
            {	stop_flag = true;
                linear_motor_msg.data = 1;  
                linear_motor_pub_.publish(linear_motor_msg);
            }
            else if (joy_msg->axes[4] == 0.0 && stop_flag)
            {   
                linear_motor_msg.data = 3;  
                linear_motor_pub_.publish(linear_motor_msg);
		stop_flag = false;
            }
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber joy_sub_;
    ros::Publisher motor_dir_pub_;
    ros::Publisher linear_motor_pub_;
    bool stop_flag = false;	
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joy_to_motor_direction");

    JoyToMotorDirection joy_to_motor_direction;

    ros::spin();

    return 0;
}
