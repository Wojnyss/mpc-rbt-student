#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"

#include <map>
#include <string>
#include <stdexcept>

struct Pose2D {
    double x;
    double y;
};

class LookupPose : public BT::SyncActionNode {
public:
    LookupPose(const std::string& name, const BT::NodeConfig& config)
        : SyncActionNode(name, config)
    {
      // manipulátory
      pose_table_["1"] = {4.5, 1.5};
      pose_table_["2"] = {4.5, -0.5};
      pose_table_["3"] = {4.5, -2.5};

      // sklady
      pose_table_["A1"] = {1.5, 0.5};
      pose_table_["A2"] = {1.5, -1.5};
      pose_table_["B1"] = {-0.5, 0.5};
      pose_table_["B2"] = {-0.5, -1.5};
      pose_table_["C1"] = {-2.5, 0.5};
      pose_table_["C2"] = {-2.5, -1.5};
      pose_table_["D1"] = {-4.5, 0.5};
      pose_table_["D2"] = {-4.5, -1.5};

      // start (doporučeno přidat)
      pose_table_["START"] = {0.0, 0.0};
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("location_id", "Location ID to look up (e.g. '1', 'A1')"),
            BT::OutputPort<double>("x", "Looked up X coordinate"),
            BT::OutputPort<double>("y", "Looked up Y coordinate")
        };
    }

  BT::NodeStatus tick() override {
      auto id = getInput<std::string>("location_id");

      if (!id) {
        std::cout << "LookupPose FAILED: missing location_id" << std::endl;
        return BT::NodeStatus::FAILURE;
      }

      std::cout << "LookupPose input = [" << id.value() << "]" << std::endl;

      auto it = pose_table_.find(id.value());
      if (it == pose_table_.end()) {
        std::cout << "LookupPose FAILED: unknown id = [" << id.value() << "]" << std::endl;
        return BT::NodeStatus::FAILURE;
      }

      setOutput("x", it->second.x);
      setOutput("y", it->second.y);

      std::cout << "LookupPose SUCCESS: x=" << it->second.x
                << " y=" << it->second.y << std::endl;

      return BT::NodeStatus::SUCCESS;
    }

private:
    std::map<std::string, Pose2D> pose_table_;
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<LookupPose>("LookupPose");
};
