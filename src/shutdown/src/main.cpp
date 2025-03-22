
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <cstdlib>

bool shutdownCallback(std_srvs::Trigger::Request& req,
                      std_srvs::Trigger::Response& res)
{
    ROS_INFO("Shutdown request received.");

    
    int shutdown_result = std::system("sudo shutdown now -h");

    
    if (WIFEXITED(shutdown_result) && (WEXITSTATUS(shutdown_result) == 0)) {
        ROS_INFO("System is shutting down.");
        res.success = true;
        res.message = "System is shutting down.";
    } else {
        ROS_ERROR("Failed to shut down the system.");
        res.success = false;
        res.message = "Failed to shut down the system.";
    }

    return true;
}

bool handleRebootService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    ROS_INFO("Rebooting system...");
    int ret = system("reboot");
    if (ret == 0)
    {
        ROS_INFO("Reboot command executed successfully.");
        return true;
    }
    else
    {
        ROS_ERROR("Failed to execute reboot command.");
        return false;
    }
}

// Service to shut down the system
// bool handleShutdownService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
// {
//     ROS_INFO("Shutting down system...");
//     int ret = system("shutdown now");
//     if (ret == 0)
//     {
//         ROS_INFO("Shutdown command executed successfully.");
//         return true;
//     }
//     else
//     {
//         ROS_ERROR("Failed to execute shutdown command.");
//         return false;
//     }
// }


bool stmrebootCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{

//	ROS_INFO("first start");
//	system("rosnode kill /serial_node");
//ROS_INFO("first system end");
//	
//sleep(1);
//ROS_INFO("sec system start");

//	system("source /home/track_sense/catkin_ws/devel/setup.bash") ;

//ROS_INFO("sec system end ");
//ROS_INFO("third system end");
//system("rosrun rosserial_python serial_node.py /dev/stm");
//ROS_INFO("third system end");
	system("bash /home/track_sense/stm_restart.sh&");
	res.success = true;
        res.message = "STM restarting";
	return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "shutdown_service");
    ros::NodeHandle nh;


    ros::ServiceServer service = nh.advertiseService("shutdown", shutdownCallback);
    ros::ServiceServer reboot_service = nh.advertiseService("bot/reboot_service", handleRebootService);
    //ros::ServiceServer shutdown_service = nh.advertiseService("bot/shutdown_service", handleShutdownService);
    ros::ServiceServer reboot_stm = nh.advertiseService("restart_stm", stmrebootCallback);
	
    ROS_INFO("Shutdown service ready.");

    ros::spin();

    return 0;
}
