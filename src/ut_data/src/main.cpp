#include <ros/ros.h>
#include <serialtoros/thick_arr.h>
#include <std_msgs/Float64.h>

class UTThicknessProcessor
{
public:
    UTThicknessProcessor() : last_valid_time_(ros::Time::now()), is_tracking_(false)
    {
        thickness_sub_ = nh_.subscribe("/ut_thickness", 10, &UTThicknessProcessor::thicknessCallback, this);

        val_pub_ = nh_.advertise<std_msgs::Float64>("/ut_val", 10);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber thickness_sub_;
    ros::Publisher val_pub_;

    float tracked_value_ = 0.0;
    ros::Time last_valid_time_;
    bool is_tracking_ = false;

    const float tolerance_ = 1.0;  
    const double required_duration_ = 1.0;  
    void thicknessCallback(const serialtoros::thick_arr::ConstPtr& msg)
    {
        if (msg->data.size() >= 2)
        {
            float first_value = msg->data[0];
            float second_value = msg->data[1];

            if (second_value == 1.0 && first_value != 0.0)
            {
                if (!is_tracking_)
                {
                    tracked_value_ = first_value;
                    last_valid_time_ = ros::Time::now();
                    is_tracking_ = true;
                    ROS_INFO("Started tracking value: %f", first_value);
                }
                else
                {
                    if (std::abs(first_value - tracked_value_) <= tolerance_)
                    {
                        if ((ros::Time::now() - last_valid_time_).toSec() >= required_duration_)
                        {
                            std_msgs::Float64 output_msg;
                            output_msg.data = tracked_value_;
                            val_pub_.publish(output_msg);
                            ROS_INFO("Published value after stability: %f", tracked_value_);

                            is_tracking_ = false;
                        }
                    }
                    else
                    {
                        ROS_WARN("Value out of tolerance. Resetting tracking. First value: %f, Tracked value: %f", first_value, tracked_value_);
                        is_tracking_ = false;
                    }
                }
            }
            else
            {
                ROS_WARN("Condition not met: first_value=%f, second_value=%f", first_value, second_value);
                is_tracking_ = false;  
            }
        }
        else
        {
            ROS_ERROR("Received invalid message with insufficient data.");
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ut_thickness_processor");
    UTThicknessProcessor processor;
    ros::spin();
    return 0;
}
