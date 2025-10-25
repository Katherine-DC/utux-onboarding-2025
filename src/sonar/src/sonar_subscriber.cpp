#include <chrono>
#include <functional>
#include <memory>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
/**
 * @brief Sonar data subscribing node
 * 
 * Subscribes to `sonar/data` topic and publishes the calculated sonar distance data
 */

 const float SPEED_OF_SOUND_FRESH_WATER = 1481.0f; // [ m/s ]


class SonarSubscriber : public rclcpp::Node{

  private:
      rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr distancePublisher_;
      rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;

    void topicCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const{
      if(msg->data.size() < 2){
        RCLCPP_WARN(this->get_logger(), "Received sonar data with insufficient elements");
        return;
      }

      float front_distance = (msg->data[0] * 0.001) * ( SPEED_OF_SOUND_FRESH_WATER) / 2.0f;
      float bottom_distance = (msg->data[1] * 0.001) * ( SPEED_OF_SOUND_FRESH_WATER) / 2.0f;

      auto distMessage = std_msgs::msg::Float32MultiArray();
      distMessage.data = { front_distance,
                           bottom_distance };

      RCLCPP_INFO(this->get_logger(), 
      "distance - [ %3.3f, %3.3f ]", distMessage.data[0], distMessage.data[1]);

      distancePublisher_->publish(distMessage);
    }

  public:
    SonarSubscriber() : Node("sonar_subscriber"){
      subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "sonar/data", 10, std::bind(&SonarSubscriber::topicCallback, this, std::placeholders::_1));

      distancePublisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("sonar/distance", 10);
    }

  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SonarSubscriber>());
  rclcpp::shutdown();

  return 0;
}