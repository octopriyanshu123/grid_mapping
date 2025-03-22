#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

class WifiStrengthMonitor
{
public:
    WifiStrengthMonitor()
    {
        
        wifi_sub_ = nh_.subscribe("wifi_strength", 10, &WifiStrengthMonitor::wifiCallback, this);

        
        timeout_timer_ = nh_.createTimer(ros::Duration(3.0), &WifiStrengthMonitor::timeoutCallback, this);

        
        stop_motors_client_ = nh_.serviceClient<std_srvs::Trigger>("/crawler_control_node/stop_motors");

        
        last_msg_time_ = ros::Time::now();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber wifi_sub_;
    ros::Timer timeout_timer_;
    ros::ServiceClient stop_motors_client_;
    ros::Time last_msg_time_;

    void wifiCallback(const std_msgs::String::ConstPtr& msg)
    {
        
        last_msg_time_ = ros::Time::now();

        try {
            
            int strength = boost::lexical_cast<int>(msg->data);

            
            if (strength > 80)
            {
                callStopMotorsService();
            }
        }
        catch (const boost::bad_lexical_cast& e) {
            ROS_WARN("Received invalid wifi strength value: %s", msg->data.c_str());
        }
    }

    void timeoutCallback(const ros::TimerEvent&)
    {
        
        if ((ros::Time::now() - last_msg_time_).toSec() > 3.0)
        {
            callStopMotorsService();
        }
    }

    void callStopMotorsService()
    {
        std_srvs::Trigger srv;
        if (stop_motors_client_.call(srv))
        {
            if (srv.response.success)
            {
                ROS_INFO("Successfully called stop motors service: %s", srv.response.message.c_str());
            }
            else
            {
                ROS_WARN("Failed to call stop motors service: %s", srv.response.message.c_str());
            }
        }
        else
        {
            ROS_ERROR("Failed to call service /crawler_control_node/stop_motors");
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wifi_strength_monitor");
    WifiStrengthMonitor wifi_strength_monitor;

    ros::spin();

    return 0;
}