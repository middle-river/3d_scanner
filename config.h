#ifndef CONFIG_H_
#define CONFIG_H_

#define CHECK(cond, msg) do {if (!(cond)) {fprintf(stderr, "ERROR(%s:%d): %s\n", __FILE__, __LINE__, (msg)); exit(1);}} while (false);

#include <opencv2/core.hpp>

class Config {
 public:
  Config(const std::string &filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    CHECK(fs.isOpened(), "Cannot open the configuration file.");
    fs["cam_width"] >> cam_width;
    fs["cam_height"] >> cam_height;
    fs["cam_config"] >> cam_config;
    fs["pat_column"] >> pat_column;
    fs["pat_row"] >> pat_row;
    fs["cal_file"] >> cal_file;
    fs["det_threshold"] >> det_threshold;
    fs["det_window"] >> det_window;
    fs["focal_length"] >> focal_length;
    fs["las_angle"] >> las_angle;
    fs["las_steps"] >> las_steps;
    fs["las_distance"] >> las_distance;
    fs["z_max"] >> z_max;
    fs["cmp_iter"] >> cmp_iter;
    fs["cmp_cycle"] >> cmp_cycle;
    fs["cmp_file"] >> cmp_file;
  }

  int cam_width, cam_height;
  std::string cam_config;
  int pat_column, pat_row;
  std::string cal_file;
  unsigned char det_threshold;
  int det_window;
  double focal_length;
  double las_angle;
  int las_steps;
  double las_distance;
  double z_max;
  int cmp_iter;
  int cmp_cycle;
  std::string cmp_file;
};

#undef CHECK

#endif
