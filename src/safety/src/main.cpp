#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_srvs/Trigger.h>

ros::ServiceClient stop_motors_client;

void distLCallback(const std_msgs::Int64::ConstPtr& msg)
{
    if (msg->data > 150)
    {
        ROS_INFO("Stopping motors...");
        std_srvs::Trigger srv;
        if (stop_motors_client.call(srv))
        {
            if (srv.response.success)
            {
                ROS_INFO("Motors stopped successfully.");
            }
            else
            {
                ROS_ERROR("Failed to stop motors: %s", srv.response.message.c_str());
            }
        }
        else
        {
            ROS_ERROR("Failed to call service /crawler_control_node/stop_motors");
        }
    }
}

void distRCallback(const std_msgs::Int64::ConstPtr& msg)
{
    if (msg->data > 150)
    {
        ROS_INFO("Stopping motors...");
        std_srvs::Trigger srv;
        if (stop_motors_client.call(srv))
        {
            if (srv.response.success)
            {
                ROS_INFO("Motors stopped successfully.");
            }
            else
            {
                ROS_ERROR("Failed to stop motors: %s", srv.response.message.c_str());
            }
        }
        else
        {
            ROS_ERROR("Failed to call service /crawler_control_node/stop_motors");
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "safety");
    ros::NodeHandle nh;

    stop_motors_client = nh.serviceClient<std_srvs::Trigger>("/crawler_control_node/stop_motors");
    ros::Subscriber dist_l_sub = nh.subscribe("/dist_l", 10, distLCallback);
    ros::Subscriber dist_r_sub = nh.subscribe("/dist_r", 10, distRCallback);

    ros::spin();

    return 0;
}