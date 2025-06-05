#include "distance_warning/distance_action_server.hpp"
#include <thread>

using namespace std::chrono_literals;

DistanceActionServer::DistanceActionServer()
    : Node("distance_action_server"), m_threshold(DEFAULT_THRESHOLD)
{
    m_action_server = rclcpp_action::create_server<CheckDistance>(
        this,
        "check_distance",
        std::bind(&DistanceActionServer::handle_goal,     this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&DistanceActionServer::handle_cancel,   this, std::placeholders::_1),
        std::bind(&DistanceActionServer::handle_accepted, this, std::placeholders::_1));

    m_param_event_subscription = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
        "/parameter_events", 10,
        std::bind(&DistanceActionServer::parameter_event_callback, this, std::placeholders::_1));

    m_param_client = std::make_shared<rclcpp::AsyncParametersClient>(this, "/set_threshold_service");

    while (rclcpp::ok() && !m_param_client->wait_for_service(1s)) {
        rclcpp::spin_some(this->get_node_base_interface());
    }
    RCLCPP_INFO(this->get_logger(), "set_threshold_service is available, initializing threshold");
    initialize_threshold_from_service();

    RCLCPP_INFO(this->get_logger(), "distance_action_server initialized with threshold: %.2f m", m_threshold);
}

void DistanceActionServer::initialize_threshold_from_service() {
    auto timeout = std::chrono::seconds(5);
    RCLCPP_INFO(this->get_logger(), "Waiting for set_threshold_service to be available with timeout: 5s");

    if (m_param_client->wait_for_service(timeout)) {
        try {
            auto future = m_param_client->get_parameters({"threshold"});
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
                auto parameters = future.get();
                if (!parameters.empty()) {
                    m_threshold = static_cast<float>(parameters[0].as_double());
                    RCLCPP_INFO(this->get_logger(), "Got initial threshold: %.2f m from set_threshold_service", m_threshold);
                    return;
                } else {
                    RCLCPP_WARN(this->get_logger(), "No parameters found");
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Timeout waiting for parameter response");
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Error getting initial threshold: %s", e.what());
        }
    }
    
    RCLCPP_WARN(this->get_logger(), "Could not get initial threshold, using default: %.2f m", DEFAULT_THRESHOLD);
    m_threshold = DEFAULT_THRESHOLD;
}

void DistanceActionServer::parameter_event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr msg) {
    if (msg->node == "/set_threshold_service") {
        for (const auto& changed_param : msg->changed_parameters) {
            if (changed_param.name == "threshold") {
                if (changed_param.value.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
                    m_threshold = static_cast<float>(changed_param.value.double_value);
                    RCLCPP_INFO(this->get_logger(), "Threshold updated via parameter event: %.2f m", m_threshold);
                }
            }
        }
        
        for (const auto& new_param : msg->new_parameters) {
            if (new_param.name == "threshold") {
                if (new_param.value.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
                    m_threshold = static_cast<float>(new_param.value.double_value);
                    RCLCPP_INFO(this->get_logger(), "Threshold set via parameter event: %.2f m", m_threshold);
                }
            }
        }
    }
}

rclcpp_action::GoalResponse DistanceActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const CheckDistance::Goal> goal)
{
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received goal request to check distance: %.2f m", goal->distance_to_check);
    
    if (goal->distance_to_check < MIN_DISTANCE || goal->distance_to_check > MAX_DISTANCE) {
        RCLCPP_WARN(this->get_logger(), "Invalid distance value: %.2f m", goal->distance_to_check);
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DistanceActionServer::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<CheckDistance>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void DistanceActionServer::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<CheckDistance>> goal_handle)
{
    std::thread{std::bind(&DistanceActionServer::execute, this, goal_handle)}.detach();
}

void DistanceActionServer::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<CheckDistance>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<CheckDistance::Feedback>();
    auto result = std::make_shared<CheckDistance::Result>();
    
    float current_threshold = m_threshold;
    
    RCLCPP_INFO(this->get_logger(), "Using threshold: %.2f m for distance check", current_threshold);
    
    const int steps = 5;
    for (int i = 1; i <= steps; ++i) {
        if (goal_handle->is_canceling()) {
            result->is_safe = false;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }

        feedback->feedback_msg = "Processing step " + std::to_string(i) + "/" + std::to_string(steps) + 
                                " - Checking distance: " + std::to_string(goal->distance_to_check) + " m";
        goal_handle->publish_feedback(feedback);

        RCLCPP_INFO(this->get_logger(), "%s", feedback->feedback_msg.c_str());
        std::this_thread::sleep_for(250ms);
    }
    
    if (goal_handle->is_canceling()) {
        result->is_safe = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
    }
    
    result->is_safe = goal->distance_to_check >= current_threshold;
    goal_handle->succeed(result);
    
    if (result->is_safe) {
        RCLCPP_INFO(this->get_logger(), "Distance check completed: SAFE (%.2f m >= %.2f m)", 
                    goal->distance_to_check, current_threshold);
    } else {
        RCLCPP_WARN(this->get_logger(), "Distance check completed: UNSAFE (%.2f m < %.2f m)", 
                    goal->distance_to_check, current_threshold);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DistanceActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
