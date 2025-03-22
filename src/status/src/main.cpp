#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <status/status.h>
#include <std_msgs/Int64.h>
#include <cstdint>  

class SaverNode {
public:
    SaverNode() {
        ros::NodeHandle nh_;
        status_update_pub_ = nh_.advertise<status::status>("/saver", 1);
        sub = nh_.subscribe("/saver", 1, &SaverNode::saverCallback, this);
        timer = nh_.createTimer(ros::Duration(0.01), &SaverNode::timerCallback, this);
    }

private:
    bool data_val[5] = {0, 0, 0, 0, 0};
    bool initalize_crawler_status = false;
    bool joy_status = false;
    ros::Publisher status_update_pub_;
    ros::Subscriber sub;
    ros::Timer timer;

    void timerCallback(const ros::TimerEvent& event) {
        status::status status_data;
        status_data.ip = 1;
        status_data.initalize_crawler = initalize_crawler_status;
        status_data.joy = joy_status;
        for (size_t i = 0; i < 5; ++i) {
            status_data.motor_status[i] = data_val[i];  
        }
        status_update_pub_.publish(status_data);
    }

    void saverCallback(const status::status::ConstPtr& msg) {
        for (size_t i = 0; i < msg->motor_status.size() && i < 5; ++i) {
            data_val[i] = msg->motor_status[i];

        }
        initalize_crawler_status = msg->initalize_crawler;
        joy_status = msg-> joy ; 
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "saver_node");

    SaverNode node;

    ros::spin();

    return 0;
}