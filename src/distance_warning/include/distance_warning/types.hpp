#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "distance_warning_interfaces/action/check_distance.hpp"
#include "distance_warning_interfaces/srv/set_threshold.hpp"
#include "distance_warning_interfaces/msg/distance.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"

// stds
using namespace std::chrono_literals;


// type definitions for messages, actions and services
using CheckDistance = distance_warning_interfaces::action::CheckDistance;
using SetThreshold  = distance_warning_interfaces::srv::SetThreshold;
using Distance      = distance_warning_interfaces::msg::Distance;


// Nodes
using SharedNodePtr          = typename rclcpp::Node::SharedPtr;

// Pub & Sub
template<typename MessageT>
using SharedPublisherPtr     = typename rclcpp::Publisher<MessageT>::SharedPtr;
template<typename MessageT>
using SharedSubscriptionPtr  = typename rclcpp::Subscription<MessageT>::SharedPtr;

// Services
template<typename ServiceT>
using SharedServiceClientPtr = typename rclcpp::Client<ServiceT>::UniquePtr;
template<typename ServiceT>
using SharedServicePtr       = typename rclcpp::Service<ServiceT>::SharedPtr;

// Actions
template<typename ActionT>
using SharedActionClientPtr  = typename rclcpp_action::Client<ActionT>::SharedPtr;
template<typename ActionT>
using SharedActionPtr        = typename rclcpp_action::Server<ActionT>::SharedPtr;

// Parameters
using SharedParamClientPtr   = typename rclcpp::AsyncParametersClient::SharedPtr;

// Utils
using SharedTimerPtr         = typename rclcpp::TimerBase::SharedPtr;