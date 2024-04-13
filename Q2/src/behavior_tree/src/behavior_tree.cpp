// main.cpp
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace BT;

class Forward : public SyncActionNode {
public:
    Forward(const std::string& name, const NodeConfiguration& config, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher)
        : SyncActionNode(name, config), publisher_(publisher) {}

    NodeStatus tick() override {
        RCLCPP_INFO(node()->get_logger(), "Moving forward towards the ball...");
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 1.0; // Move forward
        publisher_->publish(msg);
        // Simulate behavior execution with a delay
        std::this_thread::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(node()->get_logger(), "Reached the ball!");
        return NodeStatus::SUCCESS;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("behavior_tree_node");
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>(
        "/turtle1/cmd_vel", rclcpp::QoS(10));

    // Create the Behavior Tree factory
    BehaviorTreeFactory factory;

    // Register custom actions
    factory.registerNodeType<Forward>("Forward", std::cref(publisher));

    // Load the Behavior Tree structure from XML file
    auto tree = factory.createTreeFromFile("tree.xml");

    // Run the Behavior Tree
    auto status = NodeStatus::RUNNING;
    while (rclcpp::ok() && status == NodeStatus::RUNNING) {
        status = tree.rootNode()->executeTick();
    }

    rclcpp::shutdown();
    return 0;
}
