#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rclcpp/serialization.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "opencv2/opencv.hpp"
#include "map"
#include <Eigen/Dense>
#include <iostream>

// Define Pose structure (rotation + translation)
struct Pose
{
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
};

class Reconstruct3d
{
public:
    // Constructor
    Reconstruct3d();

    // Function to read bag
    void readBag(const std::string &bag_path);

private:
    // Private variables
    std::map<double, Pose> poses_;
};