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
#include "opencv2/line_descriptor.hpp"
#include "vector"
#include "map"
#include "fstream"
#include "iostream"
#include "filesystem"

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

    // Define LBD
    lbd_ = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();
  }

  // Destructor
  ~LineExtractor()
  {
    std::filesystem::path src_path(__FILE__);
    auto base_path = src_path.parent_path().parent_path();
    auto output_dir = base_path / "lines_and_descriptors";
    auto file_path = output_dir / "keylines.yml";
    cv::FileStorage fs(file_path.string(), cv::FileStorage::WRITE);
    fs << "frames" << "[";
    for (const auto &pair : keyline_map_)
    {
      fs << "{";
      fs << "timestamp" << pair.first.seconds();
      fs << "keylines" << "[";
      for (const auto &kl : pair.second)
      {
        fs << "{"
          << "p1" << cv::Point2f(kl.startPointX, kl.startPointY)
          << "p2" << cv::Point2f(kl.endPointX, kl.endPointY)
          << "}";
      }
      fs << "]";
      fs << "}";
    }
    fs << "]";
    fs.release();
    RCLCPP_INFO(this->get_logger(), "Saved %zu frames to keylines.yml", keyline_map_.size());
  }

private:
  // Initialize subscriber
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;

  // Initialize Linear Binary Descriptor (LBD)
  cv::Ptr<cv::line_descriptor::BinaryDescriptor> lbd_;

  // Min length line must have to store it
  const float min_line_length_ = 30.0f;

  // Map to store lines
  std::map<rclcpp::Time, std::vector<cv::line_descriptor::KeyLine>> keyline_map_;

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

    // Detect lines
    std::vector<cv::line_descriptor::KeyLine> keylines;
    lbd_->detect(img, keylines);

    // Visualize lines
    for (const auto &kl : keylines)
    {
      // Get ends of line
      cv::Point2f pt1 = cv::Point2f(kl.startPointX, kl.startPointY);
      cv::Point2f pt2 = cv::Point2f(kl.endPointX, kl.endPointY);

      // Draw line
      cv::line(img, pt1, pt2, cv::Scalar(255, 0, 0), 2);
    }

    // Define descriptors
    cv::Mat descriptors;
    lbd_->compute(img, keylines, descriptors);

    // Map to store lines
    rclcpp::Time timestamp = msg->header.stamp;
    keyline_map_[timestamp] = keylines;

    // Display image with detected lines
    cv::imshow("Camera Frame", img);
    cv::waitKey(1);
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