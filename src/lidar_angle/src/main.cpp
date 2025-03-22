#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32.h>
#include <tf/transform_datatypes.h>

// Function to convert quaternion component to angle in degrees
float quaternionComponentToDegrees(float component) {
    // Convert quaternion component to angle in degrees
    return acos(component) * 180.0 / M_PI; // Convert from radians to degrees
}

// Callback function to process the received PoseStamped message
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, ros::Publisher& pub) {
    float z_angle = quaternionComponentToDegrees(msg->pose.orientation.z);
    float w_angle = quaternionComponentToDegrees(msg->pose.orientation.w);

    // Normalize the angles between 0 and 360 degrees
    if (z_angle < 0) {
        z_angle += 360.0;
    }
    if (w_angle < 0) {
        w_angle += 360.0;
    }

    // Create a new PoseStamped message to publish
    geometry_msgs::PoseStamped output_msg;
    output_msg.header = msg->header;
    output_msg.pose.position = msg->pose.position;
    output_msg.pose.orientation.x = 0.0;
    output_msg.pose.orientation.y = 0.0;
    output_msg.pose.orientation.z = z_angle;
    output_msg.pose.orientation.w = w_angle;

    // Log the position and angles for debugging
    ROS_INFO("Received PoseStamped: [position: x: %f, y: %f, z: %f], [orientation: z: %f, w: %f], Converted angles: z: %f degrees, w: %f degrees",
             msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
             msg->pose.orientation.z, msg->pose.orientation.w,
             z_angle, w_angle);

    // Publish the new message
    pub.publish(output_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_subscriber");

    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("position_yaw", 10);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/slam_out_pose", 10, boost::bind(poseCallback, _1, boost::ref(pub)));

    ros::spin();

    return 0;
}
