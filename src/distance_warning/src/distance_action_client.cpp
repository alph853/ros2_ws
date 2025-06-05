#include "distance_warning/distance_action_client.hpp"

DistanceActionClient::DistanceActionClient()
    : Node("distance_action_client"), m_gen(m_rd()), m_dist(MIN_DISTANCE, std::nextafter(MAX_DISTANCE, std::numeric_limits<float>::max()))
{
    m_action_client = rclcpp_action::create_client<CheckDistance>(this, "check_distance");
    m_timer         = this->create_wall_timer(
        5s, std::bind(&DistanceActionClient::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "distance_action_client initialized, sending goals every 5 seconds");
}

void DistanceActionClient::timer_callback()
{
    if (!m_action_client->wait_for_action_server(1s)) {
        RCLCPP_WARN(this->get_logger(), "Action server not available, skipping goal");
        return;
    }
    
    float random_distance = m_dist(m_gen);
    send_goal(random_distance);
}

void DistanceActionClient::send_goal(float distance) {
    auto goal_msg = CheckDistance::Goal();
    goal_msg.distance_to_check = distance;
    
    RCLCPP_INFO(this->get_logger(), "Sending goal to check distance: %.2f m", distance);
    
    auto send_goal_options = rclcpp_action::Client<CheckDistance>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&DistanceActionClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&DistanceActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&DistanceActionClient::result_callback, this, std::placeholders::_1);
    
    m_action_client->async_send_goal(goal_msg, send_goal_options);
}

void DistanceActionClient::goal_response_callback(
    const rclcpp_action::ClientGoalHandle<CheckDistance>::SharedPtr & goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void DistanceActionClient::feedback_callback(
    rclcpp_action::ClientGoalHandle<CheckDistance>::SharedPtr,
    const std::shared_ptr<const CheckDistance::Feedback> feedback)
{
    RCLCPP_INFO(this->get_logger(), "Feedback: %s", feedback->feedback_msg.c_str());
}

void DistanceActionClient::result_callback(
    const rclcpp_action::ClientGoalHandle<CheckDistance>::WrappedResult & result)
{
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        if (result.result->is_safe) {
            RCLCPP_INFO(this->get_logger(), "Result: Distance is SAFE!");
        } else {
            RCLCPP_WARN(this->get_logger(), "Result: Distance is UNSAFE - WARNING!");
        }
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        break;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        break;
    default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DistanceActionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
