#include "distance_warning/set_threshold_service.hpp"

SetThresholdService::SetThresholdService()
    : Node("set_threshold_service")
{
    this->declare_parameter<float>("threshold", 0.5f);

    m_service = this->create_service<SetThreshold>(
        "set_threshold",
        std::bind(&SetThresholdService::handle_set_threshold, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "Other nodes can request threshold parameter from this service");
}

void SetThresholdService::handle_set_threshold(
    const std::shared_ptr<SetThreshold::Request> request,
    std::shared_ptr<SetThreshold::Response> response)
{
    float new_threshold = request->data ? -0.1f : 0.1f;
    new_threshold += this->get_parameter("threshold").as_double();
    
    if (new_threshold < MIN_THRESHOLD || new_threshold > MAX_THRESHOLD) {
        RCLCPP_WARN(this->get_logger(), "Threshold out of range: %.2f m", new_threshold);
        response->success = false;
        return;
    }

    this->set_parameter(rclcpp::Parameter("threshold", new_threshold));
    response->success = true;
    
    RCLCPP_INFO(this->get_logger(), "Threshold updated to: %.2f m. Other nodes will get this value when they request it.", new_threshold);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SetThresholdService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
