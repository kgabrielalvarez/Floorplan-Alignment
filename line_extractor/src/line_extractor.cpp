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
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/line_descriptor.hpp"
#include "vector"
#include <fstream>
#include <filesystem>
#include <iomanip>

class LineExtractor : public rclcpp::Node
{
public:
  // Constructor
  LineExtractor(const std::string &poses_file_name, const std::string &lines_file_name)
      : Node("line_extractor")
  {
    // Define line subscriber
    sub_lines_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/cam0/image_raw/compressed", 10,
        std::bind(&LineExtractor::imageCallback, this, std::placeholders::_1));

    // Define pose subscriber
    sub_poses_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/ov_msckf/poseimu", 10,
        std::bind(&LineExtractor::poseCallback, this, std::placeholders::_1));

    // Define LBD
    lbd_ = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();

    // Define LBDM
    lbdm_ = cv::line_descriptor::BinaryDescriptorMatcher::createBinaryDescriptorMatcher();

    // Define CSV file paths
    poses_file_path_ = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() / "poses_csv_files" / (poses_file_name + ".csv");
    lines_file_path_ = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() / "lines_csv_files" / (lines_file_name + ".csv");

    // Open poses CSV file
    poses_file_stream_.open(poses_file_path_);
    if (!poses_file_stream_.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", poses_file_path_.c_str());
    }
    poses_file_stream_ << "t,x,y,z,qx,qy,qz,qw\n";
    poses_file_stream_ << std::fixed << std::setprecision(5);

    // Open lines CSV file
    lines_file_stream_.open(lines_file_path_);
    if (!lines_file_stream_.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", lines_file_path_.c_str());
    }
    lines_file_stream_ << "timestamp,startX,startY,endX,endY\n";
    lines_file_stream_ << std::fixed << std::setprecision(5);

    // Define timer
    watchdog_timer_ = this->create_wall_timer(
        std::chrono::seconds(3),
        std::bind(&LineExtractor::saveEdges, this));
    // Instantly stop the timer
    watchdog_timer_->cancel();
  }

private:
  // Initialize subscriber for lines
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_lines_;

  // Initialize subscriber for poses
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_poses_;

  // Initialize Linear Binary Descriptor (LBD)
  cv::Ptr<cv::line_descriptor::BinaryDescriptor> lbd_;

  // Initialize Linear Binary Descriptor Matcher (LBDM)
  cv::Ptr<cv::line_descriptor::BinaryDescriptorMatcher> lbdm_;

  // Vector to store all keylines
  std::map<double, std::vector<cv::line_descriptor::KeyLine>> all_keylines_;

  // Vector to store all descriptors
  std::map<double, cv::Mat> all_descriptors_;

  // Vector to store all frames
  std::map<double, cv::Mat> all_images_;

  // Path to CSV files
  std::filesystem::path poses_file_path_;
  std::filesystem::path lines_file_path_;

  // File streams
  std::ofstream poses_file_stream_;
  std::ofstream lines_file_stream_;

  // Initialize timer variables
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  bool timer_expired_ = false;

  // Region of Interest on Frame
  std::vector<cv::Point> roi_ = {cv::Point(625, 200),
                                 cv::Point(1200, 200),
                                 cv::Point(1400, 400),
                                 cv::Point(1400, 450),
                                 cv::Point(625, 450)};
  std::vector<std::vector<cv::Point>> vector_of_rois_ = {roi_};

  // Image bounding box
  cv::Rect img_bbox_ = cv::Rect(300, 0, 1075, 750);

  // Acceptable angle from horizontal
  const float acceptable_angle_ = 30.0f * CV_PI / 180.0f; // [rad]

  // Accpetable length
  const float acceptable_length_ = 75.0f; // [pixels]

  // Number of edges to keep
  int num_edges_ = 3;

  // Define callback for lines
  void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    // Reset watchdog timer
    watchdog_timer_->reset();

    // Check that we are receiving an image
    cv::Mat img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_GRAYSCALE);
    cv::Mat img_color;
    cv::cvtColor(img, img_color, cv::COLOR_GRAY2BGR);
    if (img.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Empty image frame");
      return;
    }

    // Define mask
    cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC1);
    cv::fillPoly(mask, vector_of_rois_, cv::Scalar(255));

    // Detect lines
    std::vector<cv::line_descriptor::KeyLine> keylines;
    lbd_->detect(img, keylines, mask);

    // Draw mask
    cv::polylines(img, vector_of_rois_, true, cv::Scalar(0), 2);

    // Check if we detected any keylines
    if (keylines.empty())
    {
      cv::imshow("Camera Frame", img);
      cv::waitKey(1);
      return;
    }

    // Keep only keylines that are close to horizontal
    std::vector<cv::line_descriptor::KeyLine> filtered_keylines;
    for (const auto &kl : keylines)
    {
      float angle = kl.angle;       // [rad]
      float length = kl.lineLength; // [pixels]

      // Clamp angle to [0, 2pi)
      if (angle < 0)
        angle += 2.0f * CV_PI;

      // Check that angle is in range
      if ((angle > acceptable_angle_ && angle < CV_PI - acceptable_angle_) || (angle > CV_PI + acceptable_angle_ && angle < 2.0f * CV_PI - acceptable_angle_))
        continue;

      // Check that length is in range
      if (length < acceptable_length_)
        continue;

      // Keep keyline
      filtered_keylines.push_back(kl);
    }

    // Check if any keylines passed the checks
    if (filtered_keylines.empty())
    {
      cv::imshow("Camera Frame", img);
      cv::waitKey(1);
      return;
    }

    // Visualize lines
    for (const auto &kl : filtered_keylines)
    {
      // Get ends of line
      cv::Point2f pt1 = cv::Point2f(kl.startPointX, kl.startPointY);
      cv::Point2f pt2 = cv::Point2f(kl.endPointX, kl.endPointY);

      // Draw line
      cv::line(img, pt1, pt2, cv::Scalar(0), 2);
    }

    // Define descriptors
    cv::Mat descriptors;
    lbd_->compute(img, filtered_keylines, descriptors);

    // Store keylines and descriptors if any were found
    if (!filtered_keylines.empty())
    {
      double t = static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
      all_keylines_[t] = filtered_keylines;
      all_descriptors_[t] = descriptors;
      all_images_[t] = img_color(img_bbox_).clone();
    }

    // Display image with detected lines
    cv::imshow("Camera Frame", img);
    cv::waitKey(1);
  }

  // Define callback for poses
  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    double t = static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    // Add to CSV file
    poses_file_stream_ << t << "," << x << "," << y << "," << z << ","
                       << qx << "," << qy << "," << qz << "," << qw << "\n";
  }

  // Save edges to CSV file
  void saveEdges()
  {
    if (timer_expired_)
      return;

    timer_expired_ = true;
    RCLCPP_INFO(this->get_logger(), "No more messages, showing extracted edges");

    // Close previous frame
    cv::destroyWindow("Camera Frame");

    // Keep track of number of edges we saved
    int edge_counter = 0;

    for (auto frame = all_descriptors_.begin(); frame != all_descriptors_.end(); ++frame)
    {
      // Timestamp, descriptors, and keylines of the frame
      double timestamp = frame->first;
      const cv::Mat &desc = frame->second;
      const auto &keylines = all_keylines_[timestamp];

      // Cycle through all edges detected in frame
      for (const auto &kl : keylines)
      {
        double startX = static_cast<double>(kl.startPointX);
        double startY = static_cast<double>(kl.startPointY);
        double endX = static_cast<double>(kl.endPointX);
        double endY = static_cast<double>(kl.endPointY);

        // Save edge
        if (edge_counter < num_edges_)
        {
          // Image
          cv::Mat img_with_edge = all_images_[timestamp].clone();

          // Start and end points
          cv::Point2f start_point(startX - img_bbox_.x, startY - img_bbox_.y);
          cv::Point2f end_point(endX - img_bbox_.x, endY - img_bbox_.y);

          // Edge
          cv::line(img_with_edge, start_point, end_point, cv::Scalar(255, 0, 0), 2);

          // Display frame and edge
          cv::imshow("Saved Edges", img_with_edge);
          cv::waitKey(2500);

          // Save to CSV file
          lines_file_stream_ << timestamp << "," << startX << "," << startY << "," << endX << "," << endY << "\n";
        }

        edge_counter++;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Line matching complete!");
  }
};

int main(int argc, char *argv[])
{
  // Initialize
  rclcpp::init(argc, argv);

  // Default file names for poses and lines csv files
  std::string poses_file_name = "poses";
  std::string lines_file_name = "lines";

  // Assign user-specified filenames
  if (argc > 2)
  {
    poses_file_name = argv[1];
    lines_file_name = argv[2];
  }

  // Spin the node and pass filenames as arguments
  rclcpp::spin(std::make_shared<LineExtractor>(poses_file_name, lines_file_name));
  rclcpp::shutdown();
  return 0;
}