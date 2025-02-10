#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <iostream>

class NavigationNode : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    NavigationNode() : Node("navigation_node") {
        goal_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
        set_initial_pose();
        prompt_new_goal();
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr goal_client_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    void set_initial_pose() {
        RCLCPP_INFO(this->get_logger(), "Setting initial pose...");
        // Simulated pose setting (use /initialpose topic in real application)
    }

    void send_goal(float x, float y, float yaw) {
        if (!goal_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available!");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->get_clock()->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.w = cos(yaw / 2);
        goal_msg.pose.pose.orientation.z = sin(yaw / 2);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this, x, y](const GoalHandleNav::WrappedResult &result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                add_marker(x, y);
            } else {
                RCLCPP_WARN(this->get_logger(), "Goal failed!");
            }
            prompt_new_goal();
        };

        goal_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void add_marker(float x, float y) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "goal_marker";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.2;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker_pub_->publish(marker);
    }

    void prompt_new_goal() {
        float x, y, yaw;
        std::cout << "Enter new goal (x, y, yaw): ";
        std::cin >> x >> y >> yaw;
        send_goal(x, y, yaw);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationNode>());
    rclcpp::shutdown();
    return 0;
}
