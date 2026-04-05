// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

class LineExtractor : public rclcpp::Node
{
public:
  // Constructor
  LineExtractor() : Node("line_extractor")
  {
    // Define subscriber
    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/cam0/image_raw/compressed", 10,
        std::bind(&LineExtractor::imageCallback, this, std::placeholders::_1));
  }

private:
  // Initialize subscriber
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;

  // Define callback
  void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "testing testy test");
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LineExtractor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}