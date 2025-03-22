#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float32.h> 
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <signal.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <cstdlib>

using namespace std::chrono;

class Sleep : public BT::SyncActionNode
{
public:
    Sleep(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    virtual BT::NodeStatus tick() override;

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    float duration_;

    void rotationTimeCallback(const std_msgs::Float32::ConstPtr& msg);
};

Sleep::Sleep(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), duration_(1.0f)  
{
    sub_ = nh_.subscribe("/rotation_time", 10, &Sleep::rotationTimeCallback, this);
}

void Sleep::rotationTimeCallback(const std_msgs::Float32::ConstPtr& msg)
{
    duration_ = msg->data;
}

BT::PortsList Sleep::providedPorts()
{
    return {};
}

BT::NodeStatus Sleep::tick()
{
   std::this_thread::sleep_for(std::chrono::duration<float>(duration_)); 
   std::cout << "Sleep for: " << duration_ << " seconds" << std::endl;
   return BT::NodeStatus::SUCCESS;
}

class TriggerServo : public BT::SyncActionNode
{
public:
  TriggerServo(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  virtual BT::NodeStatus tick() override;
    
private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  int counter_;
};  

TriggerServo::TriggerServo(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config), counter_(1)
{
  pub_ = getInput<ros::Publisher>("linear_motor").value(); 
}

BT::PortsList TriggerServo::providedPorts()
{
    return { BT::InputPort<ros::Publisher>("linear_motor") };
}

BT::NodeStatus TriggerServo::tick()
{
    std_msgs::Int32 msg;
    if (counter_ % 2 == 0)
    {
      msg.data = 1;
    }
    else
    {
      msg.data = 2;
    }
    pub_.publish(msg);
    counter_++;
    return BT::NodeStatus::SUCCESS;
}


class TriggerJointAngle : public BT::SyncActionNode
{
public:
  TriggerJointAngle(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  virtual BT::NodeStatus tick() override;

private:
  ros::NodeHandle nh_;
  ros::ServiceClient client_;
  bool motorTurnedOn_;
};

TriggerJointAngle::TriggerJointAngle(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), motorTurnedOn_(false)
{
  client_ = nh_.serviceClient<std_srvs::Trigger>("/set_joint_angle");
}


BT::PortsList TriggerJointAngle::providedPorts()
{
    return {};
}

BT::NodeStatus TriggerJointAngle::tick() 
{
     std_srvs::Trigger srv;
        if (client_.call(srv)) {
            if (srv.response.success) {
                std::cout << "Motor turned on successfully." << std::endl;
                motorTurnedOn_ = true;
                return BT::NodeStatus::SUCCESS;
            } else {
                std::cout << "Failed to turn on motor: " << srv.response.message << std::endl;
                return BT::NodeStatus::FAILURE;
            }
        } else {
            std::cout << "Service call to /clockwise failed." << std::endl;
            return BT::NodeStatus::FAILURE;
        }
    
    return BT::NodeStatus::SUCCESS;
}

class SleepNode : public BT::SyncActionNode
{
public:
    SleepNode(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    virtual BT::NodeStatus tick() override;

private:
    int duration_;
};

SleepNode::SleepNode(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
}

BT::PortsList SleepNode::providedPorts()
{
    return {BT::InputPort<int>("duration")};
}

BT::NodeStatus SleepNode::tick()
{
   if(!getInput<int>("duration", duration_))
   {
    throw BT::RuntimeError("Missing parameter [duration] in SleepNode");
   }
   std::this_thread::sleep_for(std::chrono::seconds(duration_));
   std::cout << "SleepNode for: " << duration_ << " seconds" << std::endl;
   return BT::NodeStatus::SUCCESS;
}

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
  ros::init(argc, argv, "trigger_servo_node");
  ros::NodeHandle nh;

  signal(SIGABRT, signal_handler);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<TriggerServo>("TriggerServo");
  factory.registerNodeType<TriggerJointAngle>("TriggerJointAngle");
  factory.registerNodeType<SleepNode>("SleepNode");
  factory.registerNodeType<Sleep>("Sleep");

 
  ros::Publisher pub = nh.advertise<std_msgs::Int32>("/linear_motor", 1);
  ros::ServiceServer abort_service = nh.advertiseService("/abort_raster_scanning", abortCallback);

 
  BT::Blackboard::Ptr blackboard = BT::Blackboard::create();
  blackboard->set("linear_motor", pub);

  std::string xml_text = R"(
    <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <Sequence>
          <TriggerServo linear_motor="{linear_motor}"/>
          <Sleep/>
          <TriggerJointAngle/>
          <SleepNode duration="1"/>
        </Sequence>
      </BehaviorTree>
    </root>
  )";

  std::cout << "Loaded XML:\n" << xml_text << std::endl;

  auto tree = factory.createTreeFromText(xml_text, blackboard);

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    tree.tickRoot();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
