#ifndef DISTANCE_WARNING_DISTANCE_PUBLISHER_HPP
#define DISTANCE_WARNING_DISTANCE_PUBLISHER_HPP

#include "distance_warning/types.hpp"
#include <random>

class DistancePublisher : public rclcpp::Node
{
public:
    DistancePublisher();

private:
    void timer_callback();
    
    SharedPublisherPtr<Distance> m_publisher;
    SharedTimerPtr m_timer;

    static constexpr float MIN_DISTANCE = 0.1f;
    static constexpr float MAX_DISTANCE = 1.5f;

    std::random_device m_rd;
    std::mt19937 m_gen;
    std::uniform_real_distribution<float> m_dist;
};

#endif
