// Include header file
#include "../include/reconstruct_3d/reconstruct_3d.hpp"

int main(int argc, char **argv)
{
  // Get rosbag name
  if (argc < 2)
  {
    std::cout << "Missing bag name!\n";
    return 1;
  }

  // Define object
  Reconstruct3d reconstruct_3d;

  // Read poses and lines from bag
  std::string bag_path = argv[1];
  reconstruct_3d.readBag(bag_path);

  return 0;
}