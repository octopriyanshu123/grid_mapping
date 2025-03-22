
#include "ros/ros.h"
#include "std_msgs/Int64.h"

long int velocity = 0;
// ros::Subscriber sub = nh.subscribe("/send_ut_velocity", 10, callback);
void chatterCallback(const std_msgs::Int64::ConstPtr &msg_)
    {
        ROS_INFO("I heard: [%lld]", msg_->data);
        velocity = msg_->data;  
    }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "intrupt_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Int64>("/send_ut_velocity", 10);
    ros::Subscriber sub = nh.subscribe("/send_ut_velocity", 10, chatterCallback);

    
    
    std_msgs::Int64 msg;
    msg.data = 5920;

    long long int curr_time = 0;
    long long int prev_time = 0;
    pub.publish(msg);

    while (ros::ok())
    {
        curr_time = ros::Time::now().toSec();
        if (curr_time - prev_time > ros::Duration(5.0).toSec())
        {
            if(velocity != 0){
                msg.data = velocity;
                pub.publish(msg);
            }
            else{
                msg.data = 5920;
                pub.publish(msg);
            }
            
            prev_time = curr_time;
        }

        ros::spinOnce();
    }
    return 0;
}
