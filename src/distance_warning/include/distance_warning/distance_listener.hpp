#ifndef DISTANCE_WARNING_DISTANCE_LISTENER_HPP
#define DISTANCE_WARNING_DISTANCE_LISTENER_HPP

#include "distance_warning/types.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"

class DistanceListener : public rclcpp::Node
{
public:
    DistanceListener();

private:
    void distance_callback(const Distance::SharedPtr msg);
    void parameter_event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr msg);
    void initialize_threshold_from_service();
    
    SharedSubscriptionPtr<Distance> m_subscription;
    SharedSubscriptionPtr<rcl_interfaces::msg::ParameterEvent> m_param_event_subscription;
    SharedParamClientPtr m_param_client;

    float m_threshold;
    
    static constexpr float DEFAULT_THRESHOLD = 0.5f;
};

#endif
