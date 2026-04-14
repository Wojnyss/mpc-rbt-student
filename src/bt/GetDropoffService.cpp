#include "behaviortree_ros2/bt_service_node.hpp"
#include "behaviortree_ros2/plugins.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "rclcpp/rclcpp.hpp"

class GetDropoffService : public BT::RosServiceNode<std_srvs::srv::Trigger>
{
public:
  GetDropoffService(
      const std::string& name,
      const BT::NodeConfig& config,
      const BT::RosNodeParams& params)
      : BT::RosServiceNode<std_srvs::srv::Trigger>(name, config, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::OutputPort<std::string>("storage_id")
    });
  }

  bool setRequest(Request::SharedPtr& /*request*/) override
  {
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override
  {
    if (!response->success) {
      return BT::NodeStatus::FAILURE;
    }

    setOutput("storage_id", response->message);
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override
  {
    RCLCPP_ERROR(logger(), "GetDropoffService failed, error code=%d",
                 static_cast<int>(error));
    return BT::NodeStatus::FAILURE;
  }
};

CreateRosNodePlugin(GetDropoffService, "GetDropoffService");