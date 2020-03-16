#ifndef PCD_WRITER_H_
#define PCD_WRITER_H_

#define CHECK(cond, msg) do {if (!(cond)) {fprintf(stderr, "ERROR(%s:%d): %s\n", __FILE__, __LINE__, (msg)); exit(1);}} while (false);

#include <cstdio>
#include <opencv2/core.hpp>

class PCDWriter {
 public:
  PCDWriter(const std::string &filename, int size) {
    fp_ = std::fopen(filename.c_str(), "wb");
    CHECK(fp_ != NULL, "Failed to open the file.");
    size_ = size;
    fprintf(fp_, "# .PCD v.7 - Point Cloud Data file format\n");
    fprintf(fp_, "VERSION .7\n");
    fprintf(fp_, "FIELDS x y z\n");
    fprintf(fp_, "SIZE 4 4 4\n");
    fprintf(fp_, "TYPE F F F\n");
    fprintf(fp_, "COUNT 1 1 1\n");
    fprintf(fp_, "WIDTH %d\n", size);
    fprintf(fp_, "HEIGHT 1\n");
    fprintf(fp_, "VIEWPOINT 0 0 0 1 0 0 0\n");
    fprintf(fp_, "POINTS %d\n", size);
    fprintf(fp_, "DATA binary\n");
  }

  ~PCDWriter() {
    CHECK(size_ == 0, "Wrong size.");
    fclose(fp_);
  }

  void Write(const std::vector<cv::Point3f> &points) {
    for (const cv::Point3f &point : points) {
      const float data[3] = {point.x, point.y, point.z};
      fwrite(data, sizeof(float), 3, fp_);
      size_--;
    }
  }

 private:
  std::FILE *fp_;
  int size_;
};

#undef CHECK

#endif
