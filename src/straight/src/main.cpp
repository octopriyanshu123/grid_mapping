#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Trigger.h>

ros::Publisher cmd_vel_pub;
enum ControlMode { STOPPED, NORMAL, REVERSE };
ControlMode control_mode = STOPPED;
double linear_velocity = 1.5;

bool increaseLinearVelocity(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (linear_velocity + 0.5 <= 3.0) {
        linear_velocity += 0.5;
        res.success = true;
        res.message = "Increased linear velocity.";
    } else {
        res.success = false;
        res.message = "Linear velocity is already at the maximum limit.";
    }
    return true;
}

bool decreaseLinearVelocity(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (linear_velocity - 0.5 >= 0.5) {
        linear_velocity -= 0.5;
        res.success = true;
        res.message = "Decreased linear velocity.";
    } else {
        res.success = false;
        res.message = "Linear velocity is already at the minimum limit.";
    }
    return true;
}

void positionYawCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (control_mode == STOPPED) {
        return;
    }

    double orientation_z = msg->pose.orientation.z;
    double orientation_y = msg->pose.position.y;
    geometry_msgs::Twist cmd_vel_msg;

    if (control_mode == NORMAL) {
        if (orientation_z < 89.9 && orientation_y < 0.0) {
            cmd_vel_msg.linear.x = linear_velocity;
            cmd_vel_msg.angular.z = 8.0;
        } else if (orientation_z > 90.0) {
            cmd_vel_msg.linear.x = linear_velocity;
            cmd_vel_msg.angular.z = -0.5;
        } else {
            cmd_vel_msg.linear.x = linear_velocity;
            cmd_vel_msg.angular.z = 0.0;
        }
    } else if (control_mode == REVERSE) {
        if (orientation_z < 89.9 || orientation_y < 0.0) {
            cmd_vel_msg.linear.x = -linear_velocity;
            cmd_vel_msg.angular.z = 5.0;
        } else if (orientation_z > 90.0 || orientation_y > 0.0) {
            cmd_vel_msg.linear.x = -linear_velocity;
            cmd_vel_msg.angular.z = -5.0;
        } else {
            cmd_vel_msg.linear.x = -linear_velocity;
            cmd_vel_msg.angular.z = 0.0;
        }
    }

    cmd_vel_pub.publish(cmd_vel_msg);
}

void navigationControlCallback(const std_msgs::Int32::ConstPtr& msg) {
    switch (msg->data) {
        case 0:
            control_mode = STOPPED;
            {
                geometry_msgs::Twist stop_msg;
                stop_msg.linear.x = 0.0;
                stop_msg.angular.z = 0.0;
                cmd_vel_pub.publish(stop_msg);
            }
            break;
        case 1:
            control_mode = NORMAL;
            break;
        case 2:
            control_mode = REVERSE;
            break;
        default:
            ROS_WARN("Received unknown control mode: %d", msg->data);
            break;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "yaw_controller");
    ros::NodeHandle nh;

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/skid_steer/cmd_vel", 10);
    ros::Subscriber yaw_sub = nh.subscribe("/position_yaw", 10, positionYawCallback);
    ros::Subscriber control_sub = nh.subscribe("/navigation_control", 10, navigationControlCallback);

    ros::ServiceServer increase_service = nh.advertiseService("/increase_linear_velocity", increaseLinearVelocity);
    ros::ServiceServer decrease_service = nh.advertiseService("/decrease_linear_velocity", decreaseLinearVelocity);

    ros::spin();

    return 0;
}

