#include "slam.h"
#include <fstream>
#include <iostream>

bool Slam::initialize() { return true; }

bool Slam::read_calib_data() {
  std::ifstream fin(dataset_path_ + "/calib.txt");
  std::string line;
  while (std::getline(fin, line)) {
    std::istringstream iss(line);
    std::string camera_name;
    if (!(iss >> camera_name)) {
      std::cout << "Invalid calib.txt format.\n";
      return false;
    }
    camera_name.pop_back();

    std::array<double, 12> raw_camera{};
    for (int i = 0; i < 12; ++i) {
      iss >> raw_camera[i];
    }

    cameras_.insert({camera_name, std::make_shared<Camera>(raw_camera)});
  }
  return !cameras_.empty();
}
