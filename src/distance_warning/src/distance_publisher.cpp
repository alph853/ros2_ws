#include "distance_warning/distance_publisher.hpp"

DistancePublisher::DistancePublisher()
    : Node("distance_publisher"),
     m_gen(m_rd()),
      m_dist(MIN_DISTANCE, std::nextafter(MAX_DISTANCE, std::numeric_limits<float>::max()))
{
    m_publisher = this->create_publisher<Distance>("distance_topic", 10);
    m_timer     = this->create_wall_timer(
        1s, std::bind(&DistancePublisher::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "distance_publisher initialized, publishing at 1 Hz");
}

void DistancePublisher::timer_callback() {
    auto msg     = Distance();
    msg.distance = m_dist(m_gen);
    
    m_publisher->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published distance: %.2f m", msg.distance);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DistancePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
