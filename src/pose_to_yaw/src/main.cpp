#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include <math.h>

ros::Publisher imu_pub;

double normalize_angle(double angle) {
    return fmod(angle + 360.0, 360.0);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );
    
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double roll_deg = normalize_angle(roll * 180.0 / M_PI);
    double pitch_deg = normalize_angle(pitch * 180.0 / M_PI);
    double yaw_deg = normalize_angle(yaw * 180.0 / M_PI);

    ROS_INFO("Roll: %f degrees, Pitch: %f degrees, Yaw: %f degrees", roll_deg, pitch_deg, yaw_deg);

    geometry_msgs::Vector3 imu_msg;
    imu_msg.x = roll_deg;
    imu_msg.y = pitch_deg;
    imu_msg.z = yaw_deg;

    imu_pub.publish(imu_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_to_yaw_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/zed2/zed_node/imu/data", 1000, imuCallback);

    imu_pub = nh.advertise<geometry_msgs::Vector3>("/imu/angles", 1000);

    ros::spin();

    return 0;
}
