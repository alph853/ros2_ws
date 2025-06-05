#ifndef DISTANCE_WARNING_SET_THRESHOLD_SERVICE_HPP
#define DISTANCE_WARNING_SET_THRESHOLD_SERVICE_HPP

#include "distance_warning/types.hpp"

class SetThresholdService : public rclcpp::Node
{
public:
    SetThresholdService();

private:
    void handle_set_threshold(
        const std::shared_ptr<SetThreshold::Request> request,
        std::shared_ptr<SetThreshold::Response> response);
    
    SharedServicePtr<SetThreshold> m_service;
    
    static constexpr float MIN_THRESHOLD = 0.1f;
    static constexpr float MAX_THRESHOLD = 1.5f;
    static constexpr float THRESHOLD_STEP = 0.1f;
};

#endif