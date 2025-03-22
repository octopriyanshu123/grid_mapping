#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"

ros::Publisher linear_motor_pub;
ros::Publisher stroke_length_pub;
ros::Publisher cycles_pub;
ros::Publisher ui_stroke_length_pub;
ros::Publisher multiplier_pub;
ros::Publisher joint_pub;


std_msgs::Int32 latest_linear_motor_msg;
std_msgs::Int16 latest_stroke_length_msg;
std_msgs::Int16 cycles_msg;
std_msgs::Int16 ui_stroke_length_msg;
std_msgs::Int16 multiplier_msg;
std_msgs::Int32 joint_msg;

int cycle;
int ui_stroke_length;

bool linear_motor_received = false;
bool stroke_length_received = false;
bool cycle_data_received = false;
bool ui_data_received = false;
bool joint_received = false;


void linearMotorCallback(const std_msgs::Int32::ConstPtr& msg)
{
    latest_linear_motor_msg.data = msg->data;  
    linear_motor_received = true;  
}

void strokeLengthCallback(const std_msgs::Int16::ConstPtr& msg)
{
    latest_stroke_length_msg.data = msg->data; 
    stroke_length_received = true;  
}

void cycleCallback(const std_msgs::Int16::ConstPtr& msg)
{
    cycle = msg->data;
    cycles_msg.data = msg->data;
    cycle_data_received = true;
}

void uistrokelengthCallback(const std_msgs::Int16::ConstPtr& msg)
{
    ui_stroke_length = msg->data;
    ui_stroke_length_msg.data = msg->data;
    ui_data_received = true;

}

void jointangleCallback(const std_msgs::Int32::ConstPtr& msg)
{
    joint_msg.data = msg->data;
    joint_received = true;
}
void timerCallback(const ros::TimerEvent&)
{
    if (linear_motor_received) {
        linear_motor_pub.publish(latest_linear_motor_msg); 
    }
    if (stroke_length_received) {
        stroke_length_pub.publish(latest_stroke_length_msg);  
    }
    if (cycle_data_received)
    {   multiplier_msg.data = cycles_msg.data - 1;
        cycles_pub.publish(cycles_msg);
        multiplier_pub.publish((multiplier_msg));
    }
    if (ui_data_received)
    {
        ui_stroke_length_pub.publish(ui_stroke_length_msg);
    }
    if (joint_received)
    {
	joint_pub.publish(joint_msg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_publisher_node");
    ros::NodeHandle nh;

    
    linear_motor_pub = nh.advertise<std_msgs::Int32>("/linear_motor_publisher", 10);
    stroke_length_pub = nh.advertise<std_msgs::Int16>("/stroke_length_publisher", 10);
    cycles_pub = nh.advertise<std_msgs::Int16>("/cycles_publisher", 10);
    ui_stroke_length_pub = nh.advertise<std_msgs::Int16>("/ui_stroke_length_publisher", 10);
    multiplier_pub = nh.advertise<std_msgs::Int16>("/multiplier_publisher", 10);
    joint_pub = nh.advertise<std_msgs::Int32>("/set_joint_angle_publisher", 10);

    ros::Subscriber linear_motor_sub = nh.subscribe("/linear_motor", 10, linearMotorCallback);
    ros::Subscriber stroke_length_sub = nh.subscribe("/stroke_length", 10, strokeLengthCallback);
    ros::Subscriber cycles_sub = nh.subscribe("/cycles", 10, cycleCallback);
    ros::Subscriber ui_stroke_length_sub = nh.subscribe("/ui_stroke_length", 10, uistrokelengthCallback);
    ros::Subscriber set_joint_angle_sub = nh.subscribe("/set_joint_angle_value", 10, jointangleCallback);

    ros::Timer timer = nh.createTimer(ros::Duration(0.01), timerCallback);

    ros::spin();

    return 0;
}
