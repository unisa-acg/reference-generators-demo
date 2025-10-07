/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   diagnostics.hpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Feb 7, 2025
 *
 * This library provides functions for diagnostics.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/publisher.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace acg_diagnostics
{

template <class T>
class PeriodicPublisher
{
public:
  /**
   * @brief Create a real-time publisher for a given message type with a lower bound on the message publishing period.
   *
   * @param[in] node A shared pointer to the lifecycle node.
   * @param[in] topic_name The name of the topic to publish to.
   * @param[in] qos The quality of service settings for the publisher.
   * @param[in] period The minimum period between two consecutive publications.
   */
  PeriodicPublisher(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const std::string& topic_name, const rclcpp::QoS& qos,
                    const rclcpp::Duration& period)
    : period_(period)
  {
    publisher_ = node->create_publisher<T>(topic_name, qos);
    real_time_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<T>>(publisher_);

    // If it is the first time, publish the message
    last_publish_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  /**
   * @brief Create a real-time publisher for a given message type with an upper bound on the message publishing frequency.
   *
   * @param[in] node A shared pointer to the lifecycle node.
   * @param[in] topic_name The name of the topic to publish to.
   * @param[in] qos The quality of service settings for the publisher.
   * @param[in] frequency The maximum frequency of the publisher.
   */
  PeriodicPublisher(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const std::string& topic_name, const rclcpp::QoS& qos,
                    const double frequency)
    : PeriodicPublisher(node, topic_name, qos, rclcpp::Duration::from_seconds(1.0 / frequency))
  {}

  /**
   * @brief Attempts to publish a message in a real-time safe context.
   *
   * This method checks whether enough time has passed since the last publication based on a defined period or frequency.
   * It then attempts to acquire a lock on the publisher to safely publish the message in a real-time context.
   * If the lock cannot be acquired immediately, the method does not publish the message and returns.
   *
   * @param[in] msg The message to publish.
   * @param[in] time The current time.
   */
  void publish(const T& msg, const rclcpp::Time& time)
  {
    // Try to publish the message periodically
    if (time - last_publish_time_ >= period_)
    {
      if (real_time_publisher_->trylock())
      {
        real_time_publisher_->msg_ = msg;
        real_time_publisher_->unlockAndPublish();
        last_publish_time_ = time;
      }
    }
  }

protected:
  rclcpp::Duration period_;

  // Here typename is needed because ‘rclcpp::Publisher<MessageT>’ is a dependent scope
  typename rclcpp::Publisher<T>::SharedPtr publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<T>> real_time_publisher_;
  rclcpp::Time last_publish_time_;
};

}  // namespace acg_diagnostics
