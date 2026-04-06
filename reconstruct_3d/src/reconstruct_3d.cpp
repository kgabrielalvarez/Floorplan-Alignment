// Include header file
#include "../include/reconstruct_3d/reconstruct_3d.hpp"

// Constructor
Reconstruct3d::Reconstruct3d() {}

// Read bag
void Reconstruct3d::readBag(const std::string &bag_path)
{
    // Open bag
    rosbag2_cpp::Reader reader;
    reader.open(bag_path);

    // Define serializers
    rclcpp::Serialization<nav_msgs::msg::Odometry> pose_serializer;

    // Read msgs
    while (reader.has_next())
    {
        auto bag_msg = reader.read_next();

        if (bag_msg->topic_name == "/ov_msckf/poseimu")
        {
            // Read serialized msg
            nav_msgs::msg::Odometry msg;
            rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
            pose_serializer.deserialize_message(&serialized, &msg);

            // Extract time, position, and orientation
            double time = rclcpp::Time(msg.header.stamp).seconds();
            auto p = msg.pose.pose.position;
            auto q = msg.pose.pose.orientation;

            // Define transformation matrix
            Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
            Eigen::Matrix3d R = quat.toRotationMatrix();
            Eigen::Vector3d t(p.x, p.y, p.z);
            poses_[time] = {R, t};

        }
    }
}