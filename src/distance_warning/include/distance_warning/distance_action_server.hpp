#ifndef DISTANCE_WARNING_DISTANCE_ACTION_SERVER_HPP
#define DISTANCE_WARNING_DISTANCE_ACTION_SERVER_HPP

#include "distance_warning/types.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"

class DistanceActionServer : public rclcpp::Node
{
public:
    DistanceActionServer();

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const CheckDistance::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<CheckDistance>> goal_handle);

    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<CheckDistance>> goal_handle);

    void execute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<CheckDistance>> goal_handle);

    void parameter_event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr msg);
    void initialize_threshold_from_service();

    SharedActionPtr<CheckDistance> m_action_server;
    SharedSubscriptionPtr<rcl_interfaces::msg::ParameterEvent> m_param_event_subscription;
    SharedParamClientPtr m_param_client;
    
    float m_threshold;
    static constexpr float MIN_DISTANCE = 0.1f;
    static constexpr float MAX_DISTANCE = 1.5f;
    static constexpr float DEFAULT_THRESHOLD = 0.5f;
};

#endif
