#ifndef DISTANCE_WARNING_DISTANCE_ACTION_CLIENT_HPP
#define DISTANCE_WARNING_DISTANCE_ACTION_CLIENT_HPP

#include "distance_warning/types.hpp"
#include <random>

class DistanceActionClient : public rclcpp::Node
{
public:
    DistanceActionClient();

private:
    void timer_callback();
    void send_goal(float distance);
    
    void goal_response_callback(
        const rclcpp_action::ClientGoalHandle<CheckDistance>::SharedPtr & goal_handle);
    
    void feedback_callback(
        rclcpp_action::ClientGoalHandle<CheckDistance>::SharedPtr,
        const std::shared_ptr<const CheckDistance::Feedback> feedback);
    
    void result_callback(
        const rclcpp_action::ClientGoalHandle<CheckDistance>::WrappedResult & result);

    SharedActionClientPtr<CheckDistance> m_action_client;
    SharedTimerPtr m_timer;

    static constexpr float MIN_DISTANCE = 0.1f;
    static constexpr float MAX_DISTANCE = 1.5f;

    std::random_device m_rd;
    std::mt19937 m_gen;
    std::uniform_real_distribution<float> m_dist;
};

#endif
