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
 const int history_size = 8;

 void push(float* array, float input, int n);
 float average(float* array, int n, float prev_average);
 void init_array(float* array, int n, float value);
 float deviation(float measured, float average);


class SonarSubscriber : public rclcpp::Node{

  private:
      rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr distancePublisher_;
      rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;

      float front_distance_history_[history_size];
      float bottom_distance_history_[history_size];

      float previous_front_distance_;
      float previous_bottom_distance_;

    void topicCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
      if(msg->data.size() < 2){
        RCLCPP_WARN(this->get_logger(), "Received sonar data with insufficient elements");
        return;
      }

      float front_distance = (msg->data[0] * 0.001) * ( SPEED_OF_SOUND_FRESH_WATER / 2.0f);
      float bottom_distance = (msg->data[1] * 0.001) * ( SPEED_OF_SOUND_FRESH_WATER / 2.0f);

      push(front_distance_history_, front_distance, history_size);
      push(bottom_distance_history_, bottom_distance, history_size);

      front_distance = average(front_distance_history_, history_size, previous_front_distance_);
      bottom_distance = average(bottom_distance_history_, history_size, previous_bottom_distance_);

      auto distMessage = std_msgs::msg::Float32MultiArray();
      distMessage.data = { front_distance,
                           bottom_distance };

      RCLCPP_INFO(this->get_logger(), 
      "distance - [ %3.3f, %3.3f ]", distMessage.data[0], distMessage.data[1]);

      distancePublisher_->publish(distMessage);

      previous_front_distance_ = front_distance;
      previous_bottom_distance_ = bottom_distance;
    }

  public:
    SonarSubscriber() : Node("sonar_subscriber"){
      subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "sonar/data", 10, std::bind(&SonarSubscriber::topicCallback, this, std::placeholders::_1));

      distancePublisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("sonar/distance", 10);
      
      init_array(front_distance_history_, history_size, -1.0f);
      init_array(bottom_distance_history_, history_size, -1.0f);
      previous_front_distance_ = -1.0f;
      previous_bottom_distance_ = -1.0f;
    }

  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SonarSubscriber>());
  rclcpp::shutdown();

  return 0;
}

void push(float* array, float input, int n){
  for(int i = n-1; i > 0; i--){
    array[i] = array[i-1];
  }
  array[0] = input;
}

float average(float* array, int n, float prev_average) {
  float sum = 0.0f;
  float count = 0.0f;

  if(prev_average < 0.0f){
    prev_average = array[0];
  }

  for(int i = 0; i < n; i++){

    if(array[i] >= 0.0f && deviation(array[i], prev_average) < 3.0f){
      sum += array[i];
      count++;
    }
  }

  if(count == 0.0f){
    return 0;
  }
    return sum / count;
}

  void init_array(float* array, int n, float value){
    for(int i = 0; i < n; i++){
      array[i] = value;
    }
  }

  float deviation(float measured, float average){
    float deviation = measured - average;
    if(deviation < 0.0f){
      deviation = -deviation;
    }

    return deviation;
  }