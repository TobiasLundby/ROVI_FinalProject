#include "functions.hpp"
#include "SIFTDetector.hpp"
#include <time.h>       /* time */
#include <sys/time.h>       /* time */


std::ofstream timing_data;

long long currentTimeUs()
// Timer function
{
    timeval current;
    gettimeofday(&current, 0);
    return (long long)current.tv_sec * 1000000L + current.tv_usec;
}



int main(int argc, char** argv){

  timing_data.open ("timing_data.csv");
  timing_data << std::setprecision(8) << "FrameNo,NoMB,NoMR,MB1_x,MB1_y,MB2_x,MB2_y,MB3_x,MB3_y,MR1_x,MR2_y,center_x,center_y,analysis_time" << "\n";
  timing_data.close();



  Mat img_scene;// = imread("marker_corny/marker_corny_01.png", CV_LOAD_IMAGE_COLOR);
  Mat img_marker = imread("Marker3.ppm", CV_LOAD_IMAGE_COLOR);

  if(img_marker.empty()){
      cout << "Can not open img_marker" << endl;
      return -1;
  }

  SIFTDetector siftdetector(img_marker);
  for(int i = 1; i < 52; i++){
    long long time_start = currentTimeUs();

    stringstream ss;
    ss << setw(2) << setfill('0') << i;
    std::string s = ss.str();
    std::cout << "opening: " << "marker_corny_hard/marker_corny_hard" + s +  ".png" << std::endl;
    img_scene = imread("marker_corny_hard/marker_corny_hard_" + s +  ".png", CV_LOAD_IMAGE_COLOR);

    auto corners = siftdetector.GetCornersOfMarkerInScene(img_scene);

    Point2f center_point;
    if(!lineIntersection(corners[0], corners[2], corners[1], corners[3], center_point)){
      std::cout << "Could not find SIFT intersection" << std::endl;
    }

    circle(img_scene, corners[2], 30, Scalar( 0, 0, 255), 10);
    circle(img_scene, center_point, 30, Scalar( 0, 255, 0), 10);

    long long time_end = currentTimeUs();
    timing_data.open ("timing_data.csv",std::fstream::app|std::fstream::out);

    timing_data << std::setprecision(8) << i << "," << 0 << "," << 0 << "," << corners[0].x  << "," << corners[0].y << "," << corners[1].x << "," << corners[1].y << "," << corners[2].x << "," << corners[2].y << "," << corners[3].x << "," << corners[3].y << "," << center_point.x << "," << center_point.y << "," << (time_end-time_start) << "\n";
    timing_data.close();

    namedWindow( "Marker", CV_WINDOW_NORMAL );
    imshow( "Marker", img_scene );
//    waitKey(0);
  }

  return 0;
}
