#include "functions.hpp"

class SIFTDetector {
public:
  SIFTDetector(Mat &);
  std::vector<Point2f> GetCornersOfMarkerInScene(Mat &img_scene);
private:
  cv::Ptr<Feature2D> f2d;
  Mat descriptors_marker;
  std::vector<KeyPoint> keypoints_marker;
  Mat img_marker;
};
