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
#include "floorplan_alignment_interfaces/msg/lines_in_frame.hpp"
#include "floorplan_alignment_interfaces/msg/line_with_descriptor.hpp"
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
    lines_file_stream_ << "t1,startX1,startY1,endX1,endY1,t2,startX2,startY2,endX2,endY2\n";
    lines_file_stream_ << std::fixed << std::setprecision(5);

    // Define timer
    watchdog_timer_ = this->create_wall_timer(
        std::chrono::seconds(3),
        std::bind(&LineExtractor::find_matches, this));
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

  // Time spacing
  double delta_t_ = 1.5; // [s]

  // Path to CSV files
  std::filesystem::path poses_file_path_;
  std::filesystem::path lines_file_path_;

  // Max acceptable match distance
  float match_distance_ = 30.0f;

  // File streams
  std::ofstream poses_file_stream_;
  std::ofstream lines_file_stream_;

  // Initialize timer variables
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  bool timer_expired_ = false;

  // Define callback for lines
  void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    // Reset watchdog timer
    watchdog_timer_->reset();

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

    // Store keylines and descriptors
    double t = static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    all_keylines_[t] = keylines;
    all_descriptors_[t] = descriptors;

    // Display image with detected lines
    // cv::imshow("Camera Frame", img);
    // cv::waitKey(1);
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

  // Perform line matching
  void find_matches()
  {
    if (timer_expired_) return;

    timer_expired_ = true;
    RCLCPP_INFO(this->get_logger(), "No more messages, performing line matching...");
    
    for (auto it = all_descriptors_.begin(); it != all_descriptors_.end(); ++it)
    {
      // First timestamp and descriptor
      double t1 = it->first;
      const cv::Mat &desc1 = it->second;
      const auto &keylines1 = all_keylines_[t1];

      auto it2 = all_descriptors_.lower_bound(t1 + delta_t_);
      if (it2 == all_descriptors_.end())
        continue;

      // Second timestamp and descriptor
      double t2 = it2->first;
      const cv::Mat &desc2 = it2->second;
      const auto &keylines2 = all_keylines_[t2];

      // Find matches
      std::vector<cv::DMatch> matches;
      lbdm_->match(desc1, desc2, matches);

      // Keep acceptable matches
      std::vector<cv::DMatch> acceptable_matches;
      for (int i = 0; i < (int)matches.size(); i++)
      {
        if (matches[i].distance < match_distance_)
          acceptable_matches.push_back(matches[i]);
      }

      // Extract lines corresponding to acceptable matches
      for (const auto &m : acceptable_matches)
      {
        const auto &kl1 = keylines1[m.queryIdx];
        const auto &kl2 = keylines2[m.trainIdx];

        // Get start and end points
        float startX1 = kl1.startPointX;
        float startY1 = kl1.startPointY;
        float endX1 = kl1.endPointX;
        float endY1 = kl1.endPointY;
        float startX2 = kl2.startPointX;
        float startY2 = kl2.startPointY;
        float endX2 = kl2.endPointX;
        float endY2 = kl2.endPointY;

        // Add to CSV file
        lines_file_stream_ << t1 << "," << startX1 << "," << startY1 << "," << endX1 << "," << endY1 << ","
                           << t2 << "," << startX2 << "," << startY2 << "," << endX2 << "," << endY2 << "\n";
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