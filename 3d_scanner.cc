/*
  3D Laser Scanner

  2020-01-26  T. Nakagawa
*/

#define _USE_MATH_DEFINES
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pigpio.h>  
#include "camera.h"
#include "config.h"
#include "pcd_writer.h"

#define CHECK(cond, msg) do {if (!(cond)) {fprintf(stderr, "ERROR(%s:%d): %s\n", __FILE__, __LINE__, (msg)); exit(1);}} while (false);

class Calibration {
 public:
  Calibration(const std::string &filename) {
    if (filename.empty()) return;
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    cv::Mat camera_matrix;
    fs["camera_matrix"] >> camera_matrix;
    cv::Mat dist_coeffs;
    fs["dist_coeffs"] >> dist_coeffs;
    cv::Size image_size;
    fs["image_size"] >> image_size;
    cv::initUndistortRectifyMap(camera_matrix, dist_coeffs, cv::Mat(), camera_matrix, image_size, CV_32FC1, map_x_, map_y_);
  }

  cv::Mat Map(const cv::Mat &image) {
    if (map_x_.empty()) return image;
    cv::Mat result;
    cv::remap(image, result, map_x_, map_y_, cv::INTER_LINEAR);
    return result;
  }

 private:
  cv::Mat map_x_;
  cv::Mat map_y_;
};

class Peripheral {
 public:
  Peripheral() {
    CHECK(gpioInitialise() >= 0, "Failed to initialize GPIO.");
    gpioSetMode(PIN_LED, PI_OUTPUT);
    gpioSetMode(PIN_LASER, PI_OUTPUT);
    gpioSetMode(PIN_MOTOR_DIR, PI_OUTPUT);
    gpioSetMode(PIN_MOTOR_STEP, PI_OUTPUT);
    gpioWrite(PIN_LED, 0);
    gpioWrite(PIN_LASER, 0);
    gpioWrite(PIN_MOTOR_DIR, 0);
    gpioWrite(PIN_MOTOR_STEP, 0);
  }

  ~Peripheral() {
    gpioWrite(PIN_LED, 0);
    gpioWrite(PIN_LASER, 0);
    gpioWrite(PIN_MOTOR_DIR, 0);
    gpioWrite(PIN_MOTOR_STEP, 0);
    gpioTerminate();
  }

  void Led(bool on) {
    gpioWrite(PIN_LED, on ? 1 : 0);
  }

  void Laser(bool on) {
    gpioWrite(PIN_LASER, on ? 1 : 0);
  }

  void Rotate(int rot) {
    const int direction = (rot > 0) ? 1 : -1;
    gpioWrite(PIN_MOTOR_DIR, (direction + 1) / 2);
    for (int i = 0; i != rot; i += direction) {
      gpioWrite(PIN_MOTOR_STEP, 1);
      gpioDelay(WAIT);
      gpioWrite(PIN_MOTOR_STEP, 0);
      gpioDelay(WAIT);
    }
  }

 private:
  static constexpr unsigned int PIN_LED = 11;
  static constexpr unsigned int PIN_LASER = 17;
  static constexpr unsigned int PIN_MOTOR_DIR = 8;
  static constexpr unsigned int PIN_MOTOR_STEP = 7;
  static constexpr unsigned int WAIT = 100;  // Microseconds.
} Prhl;

void Capture(const Config &cfg, const std::string &filename) {
  Camera cam(cfg.cam_width, cfg.cam_height);
  cv::Mat image;
  cv::rotate(cam.Capture(), image, cv::ROTATE_90_CLOCKWISE);
  cv::imwrite(filename, image);
}

void Calibrate(const Config &cfg, const std::vector<std::string> &inputs, const std::string &filename) {
  std::vector<std::vector<cv::Point3f>> object_points;
  std::vector<std::vector<cv::Point2f>> image_points;
  const cv::Size pattern_size(cfg.pat_column, cfg.pat_row);
  const int pattern_num = cfg.pat_column * cfg.pat_row;
  cv::Size image_size;

  for (const std::string &input : inputs) {
    std::cerr << "Processing: " << input << "... ";
    const cv::Mat image = cv::imread(input);
    CHECK(!image.empty(), "Failed to read an image.");
    image_size = image.size();
    std::vector<cv::Point2f> corners;
    if (!cv::findChessboardCorners(image, pattern_size, corners)) {
      std::cerr << "NG!\n";
      continue;
    }
    std::cerr << "OK.\n";
    cv::drawChessboardCorners(image, pattern_size, corners, true);
    cv::imwrite(input.substr(0, input.rfind('.')) + "_cal.jpg", image);
    object_points.push_back(std::vector<cv::Point3f>(pattern_num));
    for (int i = 0; i < pattern_num; i++) {
      object_points.back()[i] = cv::Point3f(i / cfg.pat_column, i % cfg.pat_column, 0.0);
    }
    image_points.push_back(corners);
  }
  CHECK(image_points.size() == inputs.size(), "Failed to find corners.");

  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  std::vector<cv::Mat> rvecs, tvecs;
  cv::calibrateCamera(object_points, image_points, image_size, camera_matrix, dist_coeffs, rvecs, tvecs);

  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  CHECK (fs.isOpened(), "Failed to open the calibration file.");
  fs << "camera_matrix" << camera_matrix;
  fs << "dist_coeffs" << dist_coeffs;
  fs << "image_size" << image_size;
  fs.release();
}

void Adjust(const Config &cfg) {
  Camera cam(cfg.cam_width, cfg.cam_height);
  cam.Configure(cfg.cam_config);
  Calibration cal(cfg.cal_file);
  std::cerr << "Command: +N, -N, reset, on, off, snap, =<config>, exit\n";
  int rot = 0;
  while (true) {
    std::cerr << "rot=" << rot << " ? ";
    std::string command;
    if (!std::getline(std::cin, command)) break;
    if (command == "exit") {
      break;
    } else if (command[0] == '=') {
      cam.Configure(command.substr(1));
    } else if (command == "snap") {
      cv::Mat image = cam.Capture();
      const cv::Mat tmp = cal.Map(image);
      cv::rotate(tmp, image, cv::ROTATE_90_CLOCKWISE);
      cv::line(image, cv::Point(cfg.cam_height / 2, 0), cv::Point(cfg.cam_height / 2, cfg.cam_width - 1), cv::Scalar(255, 0, 0));
      cv::imwrite("adjust.jpg", image);
    } else if (command == "reset") {
      rot = 0;
    } else if (command == "on") {
      Prhl.Laser(true);
    } else if (command == "off") {
      Prhl.Laser(false);
    } else if (command[0] == '+' || command[0] == '-') {
      const int n = std::stoi(command);
      Prhl.Rotate(n);
      rot += n;
    } else {
      std::cerr << "ERROR\n";
    }
  }
  Prhl.Laser(false);
  std::cerr << "\n";
}

double FindLaserSpot(const Config &cfg, const cv::Mat &image, int h) {
  const cv::Vec3b *ptr = image.ptr<cv::Vec3b>(h);
  int r_max = 3 * cfg.det_threshold;
  int w_max = -1;
  int r = ptr[0][2] + ptr[1][2];  // Red channel (window size = 3).
  for (int w = 1; w < cfg.cam_height - 1; w++) {
    r += ptr[w + 1][2];
    if (r > r_max) {
      r_max = r;
      w_max = w;
    }
    r -= ptr[w - 1][2];
  }
  if (w_max < 0) return -1.0;

  const int window = std::min(cfg.det_window, std::min(w_max, cfg.cam_height - 1 - w_max));
  int numerator = 0;
  int denominator = 0;
  for (int w = w_max - window; w <= w_max + window; w++) {
    const unsigned char r = ptr[w][2];  // Red.
    numerator += w * r;
    denominator += r;
  }
  const double w = (double)numerator / denominator;
  return w;
}

cv::Point3d Triangulate(const Config &cfg, double w, double h, double las_angle) {
  const cv::Point3d spot(w, h, cfg.focal_length);
  const cv::Point3d las_normal(-std::tan(las_angle), 0.0, 1.0);
  const cv::Point3d las_point(-cfg.las_distance, 0.0, 0.0);
  const double l = las_normal.dot(las_point) / las_normal.dot(spot);
  const cv::Point3d point(l * spot.x, l * spot.y, l * spot.z);
  return point;
}

std::vector<double> Compensation(int cycle, const std::string &filename) {
  std::vector<double> cmp;
  if (filename.empty()) {
    cmp.assign(cycle, 0.0);
    return cmp;
  }
  FILE *fp = fopen(filename.c_str(), "r");
  CHECK(fp != NULL, "Failed to open the file.");
  double val;
  while (fscanf(fp, "%lf", &val) == 1) {
    cmp.push_back(val);
  }
  fclose(fp);
  CHECK((int)cmp.size() == cycle, "Invalid compensation data.");
  return cmp;
}

void Scan(const Config &cfg, const std::string &filename, int rot_begin, int rot_end) {
  CHECK(rot_begin <= rot_end, "Invalid rotation range.");
  std::cerr << "Initializing...\n";

  Camera cam(cfg.cam_width, cfg.cam_height);
  cam.Configure(cfg.cam_config);
  Calibration cal(cfg.cal_file);
  const std::vector<double> cmp = Compensation(cfg.cmp_cycle, cfg.cmp_file);

  // Move to the start point.
  Prhl.Rotate(rot_begin);

  // Scan.
  PCDWriter pcd(filename, cfg.cam_width * (rot_end - rot_begin + 1));
  Prhl.Laser(true);
  for (int rot = rot_begin; rot <= rot_end; rot++) {
    std::cerr << "\rrot=" << rot << "... ";

    // Capture an image.
    cv::Mat image = cam.Capture();

    // Undistort and tilt the image.
    const cv::Mat tmp = cal.Map(image);
    cv::rotate(tmp, image, cv::ROTATE_90_CLOCKWISE);

    // Triangulation.
    const double delta = cmp[((rot % cfg.cmp_cycle) + cfg.cmp_cycle) % cfg.cmp_cycle];
    const double angle = cfg.las_angle - 2.0 * M_PI * (rot + delta) / cfg.las_steps;
    std::vector<cv::Point3f> points(cfg.cam_width);
    for (int h = 0; h < cfg.cam_width; h++) {
      const double w = FindLaserSpot(cfg, image, h);
      cv::Point3f point;
      if (w >= 0.0) {
	point = static_cast<cv::Point3f>(Triangulate(cfg, w - cfg.cam_height / 2.0, h - cfg.cam_width / 2.0, angle));
	if (point.z <= 0.0 ||
	    point.z > cfg.z_max ||
	    std::abs(point.x / point.z) > cfg.cam_height / 2.0 / cfg.focal_length ||
	    std::abs(point.y / point.z) > cfg.cam_width / 2.0 / cfg.focal_length) {
	  point = cv::Point3f(0.0, 0.0, 0.0);  // Clipping.
	}
	// Convert the coordinate system (right-hand, Y-up).
	point.y *= -1.0;
	point.z *= -1.0;
      } else {
	point = cv::Point3f(0.0, 0.0, 0.0);  // No laser spot found.
      }
      points[h] = point;
    }

    // Output point crowds.
    pcd.Write(points);

    // Rotate.
    Prhl.Rotate(1);
  }
  Prhl.Laser(false);
  std::cerr << "\nFinalizing...\n";

  // Move to the origin.
  Prhl.Rotate(-rot_end - 1);
}

void Compensate(const Config &cfg, const std::string &filename) {
  std::cerr << "Compensating...\n";

  Camera cam(cfg.cam_width, cfg.cam_height);
  cam.Configure(cfg.cam_config);
  Calibration cal(cfg.cal_file);

  std::vector<double> ws(cfg.cmp_cycle + 1);
  Prhl.Laser(true);
  for (int iter = 0; iter < cfg.cmp_iter; iter++) {
    for (int rot = 0; rot <= cfg.cmp_cycle; rot++) {
      std::cerr << "\riter=" << iter << ", rot=" << rot << "... ";
      ws.clear();

      // Capture an image.
      cv::Mat image = cam.Capture();

      // Undistort and tilt the image.
      const cv::Mat tmp = cal.Map(image);
      cv::rotate(tmp, image, cv::ROTATE_90_CLOCKWISE);

      for (int j = -100; j <= 100; j++) {
	const double w = FindLaserSpot(cfg, image, cfg.cam_width / 2 + j);
	ws[rot] += w;
      }
      Prhl.Rotate(1);
    }
    Prhl.Rotate(-(cfg.cmp_cycle + 1));
  }
  Prhl.Laser(false);

  const double dd = (ws[cfg.cmp_cycle] - ws[0]) / cfg.cmp_cycle;
  FILE *fp = fopen(filename.c_str(), "w");
  CHECK(fp != NULL, "Failed to open the file.");
  for (int i = 0; i < cfg.cmp_cycle; i++) {
    const double d = ws[i] - ws[0];
    const double error = (d - dd * i) / dd;
    fprintf(fp, "%f\n", error);
  }
  fclose(fp);
  std::cerr << "done...\n";
}

int main(int argc, char **argv) {
  std::signal(SIGINT, exit);
  std::signal(SIGTERM, exit);

  if (argc < 3) {
    std::cerr << "usage: " << argv[0] << " <config file> <command> ...\n";
    std::cerr << "  <command>\n";
    std::cerr << "  capture <JPG file>\n";
    std::cerr << "  calibrate <XML file> <JPG file>...\n";
    std::cerr << "  adjust\n";
    std::cerr << "  scan <PCD file> <rot_begin> <rot_end>\n";
    std::cerr << "  compensate <TXT file>\n";
    return 1;
  }
  const Config cfg(argv[1]);
  const std::string command(argv[2]);
  Prhl.Led(true);

  if (command == "capture") {
    CHECK(argc == 4, "Invalid arguments.");
    Capture(cfg, argv[3]);
  } else if (command == "calibrate") {
    CHECK(argc > 4, "Invalid arguments.");
    std::vector<std::string> inputs;
    for (int i = 4; i < argc; i++) inputs.push_back(argv[i]);
    Calibrate(cfg, inputs, argv[3]);
  } else if (command == "adjust") {
    CHECK(argc == 3, "Invalid arguments.");
    Adjust(cfg);
  } else if (command == "scan") {
    CHECK(argc == 6, "Invalid arguments.");
    Scan(cfg, argv[3], std::atoi(argv[4]), std::atoi(argv[5]));
  } else if (command == "compensate") {
    Compensate(cfg, argv[3]);
  } else {
    CHECK(false, "Unknown command.");
  }
  Prhl.Led(false);

  return 0;
}
