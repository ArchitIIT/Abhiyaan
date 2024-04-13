// main.cpp
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace BT;

// Action node to move the turtle forward
class Forward : public SyncActionNode {
public:
    Forward(const std::string& name, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher)
        : SyncActionNode(name, {}), publisher_(publisher) {}

    NodeStatus tick() override {
        // Publish a twist message to move the turtle forward
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 1.0; // Set linear velocity
        publisher_->publish(msg);

        // Simulate behavior execution with a delay
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Stop the turtle by publishing an empty twist message
        msg.linear.x = 0.0;
        publisher_->publish(msg);

        return NodeStatus::SUCCESS;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

// Action node to pass the ball to Thortle
class Pass : public SyncActionNode {
public:
    Pass(const std::string& name, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher)
        : SyncActionNode(name, {}), publisher_(publisher) {}

    NodeStatus tick() override {
        // Implement passing the ball to Thortle
        RCLCPP_INFO(node()->get_logger(), "Passing the ball to Thortle...");
        // Simulate behavior execution with a delay
        std::this_thread::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(node()->get_logger(), "Ball passed to Thortle!");
        return NodeStatus::SUCCESS;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

// Action node for Thortle to shoot the goal
class Goal : public SyncActionNode {
public:
    Goal(const std::string& name, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher)
        : SyncActionNode(name, {}), publisher_(publisher) {}

    NodeStatus tick() override {
        // Implement shooting the goal
        RCLCPP_INFO(node()->get_logger(), "Thortle shooting the goal...");
        // Simulate behavior execution with a delay
        std::this_thread::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(node()->get_logger(), "Goal! Thanos defeated!");
        return NodeStatus::SUCCESS;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    // Initialize ROS node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("behavior_tree_node");

    // Create a publisher to control the turtle
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>(
        "/turtle1/cmd_vel", rclcpp::QoS(10));

    // Create the Behavior Tree factory
    BehaviorTreeFactory factory;

    // Register custom actions
    factory.registerNodeType<Forward>("Forward", std::cref(publisher));
    factory.registerNodeType<Pass>("Pass", std::cref(publisher));
    factory.registerNodeType<Goal>("Goal", std::cref(publisher));

    // Load the Behavior Tree structure from XML file
    auto tree = factory.createTreeFromFile("behavior_tree.xml");

    // Run the Behavior Tree
    while (rclcpp::ok()) {
        tree.rootNode()->executeTick();
    }

    // Shutdown ROS node
    rclcpp::shutdown();

    return 0;
}
