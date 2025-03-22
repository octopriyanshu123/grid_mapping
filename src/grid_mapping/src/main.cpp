#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/decorators/timer_queue.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>
#include <atomic>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <csignal>
#include <unistd.h>  // For getuid() and getpwuid()
#include <pwd.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h> 
#include "stm_interface/RelayControl.h"
#include <std_srvs/SetBool.h>   // For getpwuid()

using namespace std::chrono;

class PauseService
{
public:
    PauseService(ros::NodeHandle& nh) : paused_(false)
    {
        pause_service_ = nh.advertiseService("pause_tree", &PauseService::pauseCallback, this);
    }

    bool isPaused() const { return paused_; }

private:
    ros::ServiceServer pause_service_;
    std::atomic<bool> paused_;

    bool pauseCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
    {
        paused_ = !paused_;
        res.success = true;
        res.message = paused_ ? "Behavior Tree paused." : "Behavior Tree resumed.";
        return true;
    }
};

class Dynamic_Sleep : public BT::SyncActionNode
{
public:
    Dynamic_Sleep(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    virtual BT::NodeStatus tick() override;

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    float duration_;

    void rotationTimeCallback(const std_msgs::Float32::ConstPtr& msg);
};

Dynamic_Sleep::Dynamic_Sleep(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), duration_(1.0f)  
{
    sub_ = nh_.subscribe("/rotation_time", 10, &Dynamic_Sleep::rotationTimeCallback, this);
}

void Dynamic_Sleep::rotationTimeCallback(const std_msgs::Float32::ConstPtr& msg)
{
    duration_ = msg->data;
}

BT::PortsList Dynamic_Sleep::providedPorts()
{
    return {};
}

BT::NodeStatus Dynamic_Sleep::tick()
{
   std::this_thread::sleep_for(std::chrono::duration<float>(duration_)); 
   std::cout << "Sleep for: " << duration_ << " seconds" << std::endl;
   return BT::NodeStatus::SUCCESS;
}

/**class LAC_Zero : public BT::SyncActionNode
{
public:
    LAC_Zero(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        pub_ = nh_.advertise<std_msgs::Int8>("/motor_direction", 1);
        linear_motor_pub_ = nh_.advertise<std_msgs::Int32>("/linear_motor", 1, true);

    }

    BT::NodeStatus tick() override
    {
        ROS_INFO("Executing LAC_Zero...");
        
        std_msgs::Int8 msg;
        msg.data = 1;

        pub_.publish(msg);
        ROS_INFO("Published 0 to /servo_pose");

        ros::Duration(1.0).sleep();

//        msg.data = 0;

//        pub_.publish(msg);

        std_msgs::Int32 linear_motor_msg;
        linear_motor_msg.data = 1;
        linear_motor_pub_.publish(linear_motor_msg);

        msg.data = 0;

        pub_.publish(msg);
        ROS_INFO("Published 0 to /servo_pose");


        ros::Duration(1.0).sleep(); 

        ROS_INFO("LAC_Zero completed.");
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

private:
    ros::NodeHandle nh_;          
    ros::Publisher pub_;
    ros::Publisher linear_motor_pub_;
           
};**/

class LAC_Zero : public BT::SyncActionNode
{
public:
    LAC_Zero(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        pub_ = nh_.advertise<std_msgs::Int8>("/motor_direction", 1);
        linear_motor_pub_ = nh_.advertise<std_msgs::Int32>("/linear_motor", 1, true);

       
        ui_stroke_sub_ = nh_.subscribe("/ui_stroke_length_publisher", 1, &LAC_Zero::uiStrokeCallback, this);
        cycles_sub_ = nh_.subscribe("/cycles_publisher", 1, &LAC_Zero::cyclesCallback, this);

    }

   /** BT::NodeStatus tick() override
    {   ros::spinOnce();

        ROS_INFO("Executing LAC_Zero...");
        
        std_msgs::Int8 msg;
        msg.data = 1;

        pub_.publish(msg);
        ROS_INFO("Published 1 to /motor_direction");

        ros::Duration(1.0).sleep();

       
        ROS_INFO("Received stroke length: %d", stroke_length_);

        std_msgs::Int32 linear_motor_msg;
        linear_motor_msg.data = 1;
        linear_motor_pub_.publish(linear_motor_msg);

        msg.data = 0;

        pub_.publish(msg);
        ROS_INFO("Published 0 to /motor_direction");

        ros::Duration(1.0).sleep(); 

        ROS_INFO("LAC_Zero completed.");
        return BT::NodeStatus::SUCCESS;
    }**/

    static BT::PortsList providedPorts()
    {
        return {};
    }

private:
    ros::NodeHandle nh_;          
    ros::Publisher pub_;
    ros::Publisher linear_motor_pub_;
    ros::Subscriber ui_stroke_sub_; 
    ros::Subscriber cycles_sub_;
    int stroke_length_ = 0;  
    int cycle_limit_;
    int counter_ = 1;
    float time;


    
    void uiStrokeCallback(const std_msgs::Int16::ConstPtr& msg)
    { 
        stroke_length_ = msg->data;  
    }

    void cyclesCallback(const std_msgs::Int16::ConstPtr& msg)
    {
        cycle_limit_ = msg->data;
       // multiplier_ = msg->data;
        ROS_INFO("Updated cycle limit to %d", cycle_limit_);
    }


    BT::NodeStatus tick() override
    {   ros::spinOnce();
        counter_++;

        ROS_INFO("Executing LAC_Zero...");
        
        std_msgs::Int8 msg;
        msg.data = 1;

        pub_.publish(msg);
        ROS_INFO("Published 1 to /motor_direction");

        ros::Duration(2.0).sleep();

        if(counter_ > cycle_limit_)
        {
        ROS_INFO("Skipped slider");
	counter_ = 1;
	}
        else
	{
        std_msgs::Int32 linear_motor_msg;
        linear_motor_msg.data = 1;
        linear_motor_pub_.publish(linear_motor_msg);
	}
       // msg.data = 0;

       // pub_.publish(msg);

        if(stroke_length_ > 35)

        {time = stroke_length_ / 50.0;
        ROS_INFO("Published 0 to /motor_direction");

        ros::Duration((time - 0.5)).sleep(); }

	else
	{
	ros::Duration(0.5).sleep();
	}

        ROS_INFO("LAC_Zero completed.");

        return BT::NodeStatus::SUCCESS;
    }

};


class Pump : public BT::SyncActionNode
{
public:
    Pump(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        client_ = nh_.serviceClient<stm_interface::RelayControl>("/relay_toggle_channel");
    }

    BT::NodeStatus tick() override
    {
        ROS_INFO("Executing Pump...");

        stm_interface::RelayControl srv;
        srv.request.data = 3;

        if (client_.call(srv) && srv.response.response)
        {
            ROS_INFO("Sent 3 to /relay_toggle_channel (first call)");
        }
        else
        {
            ROS_ERROR("Failed to call service /relay_toggle_channel");
            return BT::NodeStatus::FAILURE;
        }

        ros::Duration(1.0).sleep();

        if (client_.call(srv) && srv.response.response)
        {
            ROS_INFO("Sent 3 to /relay_toggle_channel (second call)");
        }
        else
        {
            ROS_ERROR("Failed to call service /relay_toggle_channel (second call)");
            return BT::NodeStatus::FAILURE;
        }

        ROS_INFO("Pump action completed.");
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceClient client_;
};

class Electromagnet_On : public BT::SyncActionNode
{
public:
    Electromagnet_On(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) , gridCounter_(1) ,grid_size_(0) ,maxGridNum(0)
    {
        client_ = nh_.serviceClient<stm_interface::RelayControl>("/relay_toggle_channel");
        gridpub_ = nh_.advertise<std_msgs::Int32>("/grid_number" , 1);
        cycles_sub_ = nh_.subscribe("/cycles_publisher", 1, &Electromagnet_On::cycle_sub_callback_, this);

       // ui_stroke_length_sub_ = nh_.subscribe("/ui_stroke_length_publisher", 1, &Electromagnet_On::uiStrokeLengthCallback, this);

    }

    BT::NodeStatus tick() override
    {
        ROS_INFO("Executing Electromagnet_On...");


        if(gridCounter_ ==1){
            sleep(1);
        }
        if(gridCounter_ < maxGridNum){
            gridCounter_++;
        }
        else{
           gridCounter_ = 1; 
        }
        std_msgs::Int32 grid_number_msg;
        grid_number_msg.data = gridCounter_;
        gridpub_.publish(grid_number_msg);
        ROS_INFO("Grid Number is: %d" , gridCounter_ );

        stm_interface::RelayControl srv;
        srv.request.data = 5;

        if (client_.call(srv) && srv.response.response)
        {
            ROS_INFO("Sent 5 to /relay_toggle_channel (Electromagnet_On)");
        }
        else
        {
            ROS_ERROR("Failed to call service /relay_toggle_channel for Electromagnet_On");
            return BT::NodeStatus::FAILURE;
        }

        ROS_INFO("Electromagnet_On action completed.");

       // time = ui_stroke_length_value_ / 50;

       // ros::Duration(time).sleep();

        return BT::NodeStatus::SUCCESS;

         }

    static BT::PortsList providedPorts()
    {
        return {};
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceClient client_;
    ros::Subscriber ui_stroke_length_sub_;
    ros::Subscriber cycles_sub_;
    int grid_size_;
    int ui_stroke_length_value_ = 6;
    float time;
    ros::Publisher gridpub_;
    int gridCounter_;
    int maxGridNum;

    void uiStrokeLengthCallback(const std_msgs::Int16::ConstPtr& msg)
    {
        ui_stroke_length_value_ = msg->data;
    }

    void cycle_sub_callback_(const std_msgs::Int16::ConstPtr& msg){
    grid_size_ = msg->data;
    maxGridNum = grid_size_ * grid_size_;
    }


};

class Sleep : public BT::SyncActionNode
{
public:
    Sleep(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    virtual BT::NodeStatus tick() override;

private:
    int duration_;
};

Sleep::Sleep(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
}

BT::PortsList Sleep::providedPorts()
{
    return {BT::InputPort<int>("duration")};
}

BT::NodeStatus Sleep::tick()
{
   if(!getInput<int>("duration", duration_))
   {
    throw BT::RuntimeError("Missing parameter [duration] in Sleep");
   }
   std::this_thread::sleep_for(std::chrono::seconds(duration_));
   std::cout << "Sleep for:" << duration_ << "seconds" << std::endl;
   return BT::NodeStatus::SUCCESS;
}

class LAC_Down : public BT::SyncActionNode
{
public:
    LAC_Down(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        pub_ = nh_.advertise<std_msgs::Int8>("/motor_direction", 1);
        client_ = nh_.serviceClient<stm_interface::RelayControl>("/relay_toggle_channel");
        ui_stroke_length_sub_ = nh_.subscribe("/ui_stroke_length_publisher", 1, &LAC_Down::uiStrokeLengthCallback, this);
       // scan_speed_sub_ = nh_.subscribe("/scan_speed", 1, &LAC_Down::scanspeedCallback, this);


    }


    BT::NodeStatus tick() override
    {
        ROS_INFO("Executing LAC_Down...");
        stm_interface::RelayControl srv;
        srv.request.data = 3;

        // if (client_.call(srv) && srv.response.response)
        // {
        //     //ROS_INFO("Sent 3 to /relay_toggle_channel (first call)");
        // }
        // else
        // {
        //     //ROS_ERROR("Failed to call service /relay_toggle_channel");
        //     return BT::NodeStatus::FAILURE;
        // }

        std_msgs::Int8 msg;
        msg.data = 2;

        pub_.publish(msg);
        ROS_INFO("Published 0 to /servo_pose");

        if (client_.call(srv) && srv.response.response)
        {
            //ROS_INFO("Sent 3 to /relay_toggle_channel (first call)");
        }
        else
        {
            //ROS_ERROR("Failed to call service /relay_toggle_channel");
            return BT::NodeStatus::FAILURE;
        }

        ros::Duration(1.0).sleep();

        if (client_.call(srv) && srv.response.response)
        {
            //ROS_INFO("Sent 3 to /relay_toggle_channel (second call)");
        }
        else
        {
            //ROS_ERROR("Failed to call service /relay_toggle_channel (second call)");
            return BT::NodeStatus::FAILURE;
        }
        
        

        //ros::Duration(1.0).sleep(); 

        ROS_INFO("LAC_Zero completed.");
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

private:
    ros::NodeHandle nh_;          
    ros::Publisher pub_;
    ros::ServiceClient client_;
    ros::Subscriber ui_stroke_length_sub_;           
    ros::Subscriber scan_speed_sub_;
    int ui_stroke_length_value_ = 6;
    


    void uiStrokeLengthCallback(const std_msgs::Int16::ConstPtr& msg)
    {
        ui_stroke_length_value_ = msg->data;
    }

 


};

class Electromagnet_Off : public BT::SyncActionNode
{
public:
    Electromagnet_Off(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        client_ = nh_.serviceClient<stm_interface::RelayControl>("/relay_toggle_channel");
    }

    BT::NodeStatus tick() override
    {
        ROS_INFO("Executing Electromagnet_Off...");
       

        stm_interface::RelayControl srv;
        srv.request.data = 5;

        if (client_.call(srv) && srv.response.response)
        {
            ROS_INFO("Sent 5 to /relay_toggle_channel (Electromagnet_Off)");
        }
        else
        {
            ROS_ERROR("Failed to call service /relay_toggle_channel for Electromagnet_Off");
            return BT::NodeStatus::FAILURE;
        }

        ROS_INFO("Electromagnet_On action completed.");
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceClient client_;
    
};

class Slider_shift : public BT::SyncActionNode
{
public:
    Slider_shift(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config), counter_(1), status_(BT::NodeStatus::FAILURE), cycle_limit_(12), multiplier_(12)
    {
        linear_motor_pub_ = nh_.advertise<std_msgs::Int32>("/linear_motor", 1, true);
        stroke_length_pub_ = nh_.advertise<std_msgs::Int16>("/stroke_length", 1, true);
        linear_servo_pub_ = nh_.advertise<std_msgs::Int32>("/linear_servo", 1, true);

        
        stroke_length_sub_ = nh_.subscribe("/stroke_length_publisher", 1, &Slider_shift::strokeLengthCallback, this);
        cycles_sub_ = nh_.subscribe("/cycles_publisher", 1, &Slider_shift::cyclesCallback, this);
        ui_stroke_length_sub_ = nh_.subscribe("/ui_stroke_length_publisher", 1, &Slider_shift::uiStrokeLengthCallback, this);
        set_joint_angle_client_ = nh_.serviceClient<std_srvs::Trigger>("/set_joint_angle");
        multiplier_sub_ = nh_.subscribe("/multiplier_publisher", 1, &Slider_shift::multiplierCallback, this);
        forward_sub_ = nh_.subscribe("/set_joint_angle_publisher", 1, &Slider_shift::forwardCallback, this);

    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher linear_motor_pub_;
    ros::Publisher stroke_length_pub_;
    ros::Publisher linear_servo_pub_;
    ros::Subscriber stroke_length_sub_;
    ros::Subscriber cycles_sub_;
    ros::Subscriber ui_stroke_length_sub_;
    ros::Subscriber multiplier_sub_; 
    ros::Subscriber forward_sub_;
    ros::ServiceClient set_joint_angle_client_;
    int counter_;
    int cycle_limit_;
    int multiplier_;
    int forward_;
    float time_;
    std_msgs::Int16 last_stroke_length_msg_;
    int ui_stroke_length_value_ = 6; 
    BT::NodeStatus status_;

    void strokeLengthCallback(const std_msgs::Int16::ConstPtr& msg)
    {
        last_stroke_length_msg_ = *msg;
    }

    void cyclesCallback(const std_msgs::Int16::ConstPtr& msg)
    {
        cycle_limit_ = msg->data;
       // multiplier_ = msg->data;
        ROS_INFO("Updated cycle limit to %d", cycle_limit_);
    }
    
    void multiplierCallback(const std_msgs::Int16::ConstPtr& msg)
    {
       // cycle_limit_ = msg->data;
        multiplier_ = msg->data;
        ROS_INFO("Updated multiplier to %d", multiplier_);
    }



    void uiStrokeLengthCallback(const std_msgs::Int16::ConstPtr& msg)
    {
        ui_stroke_length_value_ = msg->data;
    }

    void forwardCallback(const std_msgs::Int32::ConstPtr& msg)
    {
	forward_ = msg->data;
    }

    BT::NodeStatus tick() override
    {
        ROS_INFO("Executing Slider_shift...");

        std_msgs::Int32 linear_motor_msg;

        if (counter_ < cycle_limit_)
        {
            counter_++;
            status_ = BT::NodeStatus::FAILURE;
        }
        else
        {
            ros::spinOnce();

            int16_t stroke_length_value = last_stroke_length_msg_.data;
            ROS_INFO("Received data from /stroke_length_publisher: %d", stroke_length_value);

            if (stroke_length_value != 0)
            {
                std_msgs::Int16 stroke_length_msg;
                stroke_length_msg.data = stroke_length_value * multiplier_;
                stroke_length_pub_.publish(stroke_length_msg);
                ROS_INFO("Published %d to /stroke_length (multiplied by %d)", stroke_length_msg.data, multiplier_);

                ROS_INFO("Before publishing 2 to linear msg");
                ros::Duration(1.0).sleep();

		

                linear_motor_msg.data = 2;
                linear_motor_pub_.publish(linear_motor_msg);
                ROS_INFO("Published 2 to /linear_servo");


                float calculated_duration = ((ui_stroke_length_value_ * multiplier_)/50);
                ROS_INFO("Calculated sleep duration: %.2f seconds", calculated_duration); 
                std::this_thread::sleep_for(std::chrono::duration<float>(calculated_duration));

		ROS_INFO("Before moving forward");

                std_srvs::Trigger srv;
                if (set_joint_angle_client_.call(srv))
                {
                    if (srv.response.success)
                    {
                        ROS_INFO("Successfully called /set_joint_angle");
                    }
                    else
                    {
                        ROS_WARN("Service /set_joint_angle called but did not succeed.");
                    }
                }
                else
                {
                    ROS_ERROR("Failed to call service /set_joint_angle");
                    return BT::NodeStatus::FAILURE;
                }

		time_ = forward_ / 80.0;
                ros::Duration(time_).sleep();
		ROS_INFO("After moving forward");

               // float calculated_duration = ((ui_stroke_length_value_ * multiplier_)/50);
               // ROS_INFO("Calculated sleep duration: %.2f seconds", calculated_duration); 
               // std::this_thread::sleep_for(std::chrono::duration<float>(calculated_duration));
	//	ros::Duration(5.0).sleep();
                stroke_length_msg.data = stroke_length_value;
                stroke_length_pub_.publish(stroke_length_msg);
                ROS_INFO("Published %d to /stroke_length (divided by %d)", stroke_length_msg.data, multiplier_);
	//	ros::Duration(5.0).sleep();
            }

            counter_ = 1;
            status_ = BT::NodeStatus::SUCCESS;
        }

        ROS_INFO("Slider_shift action completed.");
        return status_;
    }
};


void signal_handler(int signal)
{
  if (signal == SIGABRT)
    std::cerr << "SIGABRT received\n";
  else
    std::cerr << "Unexpected signal " << signal << " received\n";
  std::_Exit(EXIT_FAILURE);
}

bool abortCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  if (req.data)
  {
    ROS_WARN("Abort service called. Raising SIGABRT.");
    res.success = true;
    res.message = "Behavior Tree aborted";
    raise(SIGABRT);
  }
  res.success = false;
  res.message = "Abort service called with false";
  return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "bt_lac_zero_node");
    ros::NodeHandle nh;

    signal(SIGABRT, signal_handler);

    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<LAC_Zero>("LAC_Zero");
    factory.registerNodeType<Pump>("Pump");
    factory.registerNodeType<Electromagnet_On>("Electromagnet_On");
    factory.registerNodeType<Sleep>("Sleep");
    factory.registerNodeType<LAC_Down>("LAC_Down");
    factory.registerNodeType<Electromagnet_Off>("Electromagnet_Off");
    factory.registerNodeType<Dynamic_Sleep>("Dynamic_Sleep");
    factory.registerNodeType<Slider_shift>("Slider_shift");

    ros::ServiceServer abort_service = nh.advertiseService("/abort_grid_mapping", abortCallback);



    BT::Blackboard::Ptr blackboard = BT::Blackboard::create();

    const std::string xml_text = R"(
    <root main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree">
            <Sequence>
                <Electromagnet_On/>
                <LAC_Down/>
                <Sleep duration="5"/>
                <Electromagnet_Off/>
                <LAC_Zero/>
                <Slider_shift/>
            </Sequence>
        </BehaviorTree>
    </root>)";

    BT::Tree tree = factory.createTreeFromText(xml_text);

    BT::StdCoutLogger logger_cout(tree);

    PauseService pause_service(nh);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        if (!pause_service.isPaused())
        {
            tree.tickRoot();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
