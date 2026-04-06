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
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "floorplan_alignment_interfaces/msg/lines_in_frame.hpp"
#include "floorplan_alignment_interfaces/msg/line_with_descriptor.hpp"

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

    // Define rosbag writer
    // writer_ = std::make_unique<rosbag2_cpp::Writer>();
    // writer_->open("lines_and_descriptors_bag");

    // Define publisher
    pub_ = this->create_publisher<floorplan_alignment_interfaces::msg::LinesInFrame>("/lines_with_descriptors", 10);
  }

private:
  // Initialize subscriber
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;

  // Initialize Linear Binary Descriptor (LBD)
  cv::Ptr<cv::line_descriptor::BinaryDescriptor> lbd_;

  // Initialize rosbag writer
  // std::unique_ptr<rosbag2_cpp::Writer> writer_;

  // Initialize publisher
  rclcpp::Publisher<floorplan_alignment_interfaces::msg::LinesInFrame>::SharedPtr pub_;

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
      cv::line(img, pt1, pt2, cv::Scalar(255), 2);
    }

    // Define descriptors
    cv::Mat descriptors;
    lbd_->compute(img, keylines, descriptors);

    // Save lines and descriptors to msg
    floorplan_alignment_interfaces::msg::LinesInFrame lines_in_frame_msg;
    lines_in_frame_msg.header.stamp = msg->header.stamp;
    for (size_t i = 1; i < keylines.size(); i++)
    {
      // Start and end points
      floorplan_alignment_interfaces::msg::LineWithDescriptor lwd;
      lwd.start.x = keylines[i].startPointX;
      lwd.start.y = keylines[i].startPointY;
      lwd.start.z = 0.0;
      lwd.end.x = keylines[i].endPointX;
      lwd.end.y = keylines[i].endPointY;
      lwd.end.z = 0.0;

      // Descriptor
      cv::Mat desc_row = descriptors.row((int)i);
      lwd.descriptor.assign(desc_row.datastart, desc_row.dataend);

      // Add to lines in frame msg
      lines_in_frame_msg.lines.push_back(lwd);
    }

    // Publish lines with descriptors
    pub_->publish(lines_in_frame_msg);

    // Write msg to rosbag
    // auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    // bag_msg->time_stamp = msg->header.stamp.sec * 1'000'000'000ULL + msg->header.stamp.nanosec;
    // bag_msg->topic_name = "/lines_with_descriptors";
    // rclcpp::Serialization<floorplan_alignment_interfaces::msg::LinesInFrame> serializer;
    // rclcpp::SerializedMessage serialized_msg;
    // serializer.serialize_message(&lines_in_frame_msg, &serialized_msg);
    // bag_msg->serialized_data = std::vector<uint8_t>(
    //     serialized_msg.get_rcl_serialized_message().buffer,
    //     serialized_msg.get_rcl_serialized_message().buffer +
    //         serialized_msg.get_rcl_serialized_message().buffer_length);
    // writer_->write(bag_msg);

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