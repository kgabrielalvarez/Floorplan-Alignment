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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <fstream>
#include <filesystem>

class TrajectoryRecorder : public rclcpp::Node
{
public:
  // Constructor
  TrajectoryRecorder(const std::string &filename)
      : Node("trajectory_recorder")
  {
    // Path to file
    std::filesystem::path file_path = std::filesystem::path(__FILE__).parent_path().parent_path() / "trajectories" / (filename + ".csv");

    // Open CSV file
    csv_file.open(file_path);
    if (!csv_file.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", file_path.c_str());
    }
    csv_file << "x,y,z,qx,qy,qz,qw\n";

    // Define subscription_
    subscription_ =
        this->create_subscription<nav_msgs::msg::Path>("/ov_msckf/pathimu", 10, std::bind(&TrajectoryRecorder::topic_callback, this, std::placeholders::_1));
  }

private:
  // Initialize subscription_
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_;

  // Initialize variable to keep track of number of poses
  size_t last_written_index_ = 0;

  // Initialize fstream
  std::ofstream csv_file;

  // Define callback
  void topic_callback(nav_msgs::msg::Path::SharedPtr msg)
  {
    const auto &poses = msg->poses;

    if (poses.size() <= last_written_index_)
    {
      return; // does nothing
    }

    for (size_t i = last_written_index_; i < poses.size(); ++i)
    {
      const auto &p = poses[i].pose;

      double x = p.position.x;
      double y = p.position.y;
      double z = p.position.z;

      double qx = p.orientation.x;
      double qy = p.orientation.y;
      double qz = p.orientation.z;
      double qw = p.orientation.w;

      RCLCPP_INFO(this->get_logger(), "x position: %f", x);

      csv_file << x << "," << y << "," << z << ","
               << qx << "," << qy << "," << qz << "," << qw << "\n";
    }

    last_written_index_ = poses.size();
    csv_file.flush();
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // Default file name for trajectory csv file
  std::string filename = "trajectory";

  // If trajectory name argument is passed assign it to filename
  if (argc > 1)
  {
    filename = argv[1];
  }

  // Spin the node and pass filename as an argument
  rclcpp::spin(std::make_shared<TrajectoryRecorder>(filename));
  rclcpp::shutdown();
  return 0;
}