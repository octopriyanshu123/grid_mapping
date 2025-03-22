#include "ros/ros.h"
#include <cstdlib>
#include <cstdio>
#include <string>
#include <array>
#include <iostream>
#include <memory>
#include <stdexcept>

float getDiskUsage() {
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen("df -h /dev/nvme0n1p1 | grep /dev/nvme0n1p1 | awk '{print $5}'", "r"), pclose);
    if (!pipe) throw std::runtime_error("popen() failed!");
    while (fgets(buffer.data(), 128, pipe.get()) != nullptr) {
        result += buffer.data();
    }
    
    result.pop_back();
    return std::stof(result);
}

void clearLogsAndRestartSyslog() {
    std::string password = "y"; 
    std::string command = "echo " + password + " | sudo -S rm -rf /var/log";
    std::system(command.c_str());
    command = "echo " + password + " | sudo -S systemctl restart syslog.service";
    std::system(command.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "disk_monitor_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10); 
    
    std::system("bash /home/track_sense/cam.sh&");
    std::system("bash /home/track_sense/zed.sh&");
    while (ros::ok()) {
        try {
            float disk_usage = getDiskUsage();
            //ROS_INFO("Current disk usage: %.2f%%", disk_usage);
            if (disk_usage >= 30.0) {
                ROS_WARN("Disk usage exceeded 30%%, executing cleanup commands.");
                clearLogsAndRestartSyslog();
            }
        } catch (const std::exception &e) {
            ROS_ERROR("Error checking disk usage: %s", e.what());
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
