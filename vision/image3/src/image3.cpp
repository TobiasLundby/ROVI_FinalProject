#include "functions.hpp"
#include "SIFTDetector.hpp"



int main(int argc, char** argv){

  Mat img_scene;// = imread("marker_corny/marker_corny_01.png", CV_LOAD_IMAGE_COLOR);
  Mat img_marker = imread("Marker3.ppm", CV_LOAD_IMAGE_COLOR);

  if(img_marker.empty()){
      cout << "Can not open img_marker" << endl;
      return -1;
  }

  SIFTDetector siftdetector(img_marker);
  for(int i = 1; i < 20; i++){

    stringstream ss;
    ss << setw(2) << setfill('0') << i;
    std::string s = ss.str();
    std::cout << "opening: " << "marker_corny/marker_corny_" + s +  ".png" << std::endl;
    img_scene = imread("marker_corny/marker_corny_" + s +  ".png", CV_LOAD_IMAGE_COLOR);

    auto corners = siftdetector.GetCornersOfMarkerInScene(img_scene);

    Point2f center_point;
    if(!lineIntersection(corners[0], corners[2], corners[1], corners[3], center_point)){
      std::cout << "Could not find SIFT intersection" << std::endl;
    }

    circle(img_scene, corners[2], 30, Scalar( 0, 0, 255), 10);
    circle(img_scene, center_point, 30, Scalar( 0, 255, 0), 10);

    namedWindow( "Marker", CV_WINDOW_NORMAL );
    imshow( "Marker", img_scene );
    waitKey(0);
  }

  return 0;
}
