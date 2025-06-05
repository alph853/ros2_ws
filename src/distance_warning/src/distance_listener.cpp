#include "distance_warning/distance_listener.hpp"
#include <chrono>


DistanceListener::DistanceListener()
    : Node("distance_listener"), m_threshold(DEFAULT_THRESHOLD)
{
    m_subscription = this->create_subscription<Distance>(
        "distance_topic", 10,
        std::bind(&DistanceListener::distance_callback, this, std::placeholders::_1));

    m_param_event_subscription = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
        "/parameter_events", 10,
        std::bind(&DistanceListener::parameter_event_callback, this, std::placeholders::_1));
    
    m_param_client = std::make_shared<rclcpp::AsyncParametersClient>(this, "/set_threshold_service");
    initialize_threshold_from_service();
    
    RCLCPP_INFO(this->get_logger(), "distance_listener initialized with threshold: %.2f m", m_threshold);
}

void DistanceListener::initialize_threshold_from_service() {
    auto timeout = std::chrono::seconds(5);
    RCLCPP_INFO(this->get_logger(), "Waiting for set_threshold_service to be available with timeout: 5s");

    if (m_param_client->wait_for_service(timeout)) {
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
    } else {
        RCLCPP_WARN(this->get_logger(), "set_threshold_service not available within timeout");
    }
    
    RCLCPP_WARN(this->get_logger(), "Could not get initial threshold, using default: %.2f m", DEFAULT_THRESHOLD);
    m_threshold = DEFAULT_THRESHOLD;
}

void DistanceListener::parameter_event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr msg) {
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

void DistanceListener::distance_callback(const Distance::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received distance: %.2f m (threshold: %.2f m)", msg->distance, m_threshold);
    
    if (msg->distance < m_threshold) {
        RCLCPP_WARN(this->get_logger(), "Warning: Object too close! Distance: %.2f m < Threshold: %.2f m", 
                    msg->distance, m_threshold);
    } else {
        RCLCPP_INFO(this->get_logger(), "Distance safe: %.2f m >= %.2f m", msg->distance, m_threshold);
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DistanceListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
