#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <regex>

int main(int argc, char **argv) {
    ros::init(argc, argv, "wifi_strength_node");
    ros::NodeHandle nh;

    ros::Publisher wifi_pub = nh.advertise<std_msgs::String>("/wifi_strength", 1000);

    ros::Rate loop_rate(1); // 1 Hz

    while (ros::ok()) {
        FILE* pipe = popen("iw dev wlan0 station dump | grep 'signal:'", "r");
        if (!pipe) {
            return -1;
        }
        char buffer[128];
        while (fgets(buffer, sizeof buffer, pipe) != NULL) {
            std::string str(buffer);
            std::regex r("-\\d+");
            std::smatch m;
            std::regex_search(str, m, r);
            if (!m.empty()) {
                std_msgs::String msg;
                msg.data = m[0].str().substr(1); // remove the leading '-'
                wifi_pub.publish(msg);
                ROS_INFO("Signal level: %s", msg.data.c_str());
            }
        }
        pclose(pipe);
        loop_rate.sleep();
    }
    return 0;
}