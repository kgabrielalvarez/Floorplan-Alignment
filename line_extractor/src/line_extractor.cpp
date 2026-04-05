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
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
#include "opencv2/objdetect.hpp"
#include "vector"
#include "map"

// Class to store 2D lines
struct Line2D
{
    cv::Point2f p1;
    cv::Point2f p2;
};

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

    // Initialize LSD
    lsd_ = cv::createLineSegmentDetector(cv::LSD_REFINE_STD);
  }

private:
  // Initialize subscriber
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;

  // Initialize Line Segment Detector (LSD)
  cv::Ptr<cv::LineSegmentDetector> lsd_;

  // Initialize map to store detected lines
  std::map<rclcpp::Time, std::vector<Line2D>> lines_timestamped_;

  // Min length line must have to store it
  const float min_line_length_ = 30.0f;

  // Define callback
  void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    // Check that we are receiving an image
    cv::Mat img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_GRAYSCALE);
    if (img.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Empty image frame");
      return;
    }
    else {
      RCLCPP_INFO(this->get_logger(), "Image received");
    }

    // Detect lines
    std::vector<cv::Vec4f> lines_cv;
    lsd_->detect(img, lines_cv);

    // Store detected lines
    std::vector<Line2D> lines;
    for (const auto &l : lines_cv)
    {
      // Get line from lines_cv
      Line2D line{cv::Point2f(l[0], l[1]), cv::Point2f(l[2], l[3])};

      // Filter out lines that are too short
      float line_length = cv::norm(line.p2 - line.p1);
      if (line_length < min_line_length_) continue;

      lines.push_back(line);

      // Add detected lines to image
      cv::line(img, line.p1, line.p2, cv::Scalar(0, 0, 255), 2);
    }
    rclcpp::Time timestamp = msg->header.stamp;
    lines_timestamped_[timestamp] = lines;

    // Display image with detected lines
    cv::imshow("Camera Frame", img);
    cv::waitKey(1);

    // Logging message
    RCLCPP_INFO(this->get_logger(), "%zu lines detected", lines.size());
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