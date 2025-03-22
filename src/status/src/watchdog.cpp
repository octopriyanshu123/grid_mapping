#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <cstdint>

class Watchdog {
public:
    Watchdog() {
        ros::NodeHandle nh_;
        status_update_pub_ = nh_.advertise<std_msgs::Bool>("/bot_heartbeat", 1);
        timer = nh_.createTimer(ros::Duration(0.01), &Watchdog::timerCallback, this); 
    }

private:
    ros::Publisher status_update_pub_;
    ros::Timer timer;

    void timerCallback(const ros::TimerEvent& event) {
        std_msgs::Bool msg;
        msg.data = true;  
        status_update_pub_.publish(msg); 
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "saver_node");

    Watchdog node;

    ros::spin();

    return 0;
}
