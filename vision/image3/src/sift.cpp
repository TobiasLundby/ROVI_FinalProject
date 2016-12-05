#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <vector>
using namespace std;

//#include "opencv2/xfeatures2d.hpp"
//#include "opencv2/features2d/features2d.hpp"
//#include <opencv2/nonfree/nonfree.hpp>
//#include "opencv2/features2d/features2d.hpp"
//#include "opencv2/nonfree/features2d.hpp"

#include <opencv2/opencv.hpp>
//#include "opencv2/features2d/features2d.hpp"
#include <opencv2/nonfree/nonfree.hpp>
#include "opencv2/xfeatures2d.hpp"
//#include <opencv2/nonfree/features2d.hpp>
//#include "opencv2/xfeatures2d.hpp"
//#include "opencv2/xfeatures2d/nonfree.hpp"
using namespace cv;
#define EPSILON 1E-5


//! Return the maximum of the provided numbers
static double maximum(double number1, double number2, double number3) {
    return std::max(std::max(number1, number2), number3);
}

static bool almostEqual(double number1, double number2) {
    return (std::abs(number1 - number2) <= (EPSILON * maximum(1.0, std::abs(number1), std::abs(number2))));
}

static bool lineIntersection(const cv::Point2f &a1, const cv::Point2f &b1, const cv::Point2f &a2,
                             const cv::Point2f &b2, cv::Point2f &intersection) {
    double A1 = b1.y - a1.y;
    double B1 = a1.x - b1.x;
    double C1 = (a1.x * A1) + (a1.y * B1);

    double A2 = b2.y - a2.y;
    double B2 = a2.x - b2.x;
    double C2 = (a2.x * A2) + (a2.y * B2);

    double det = (A1 * B2) - (A2 * B1);

    if (!almostEqual(det, 0)) {
        intersection.x = static_cast<float>(((C1 * B2) - (C2 * B1)) / (det));
        intersection.y = static_cast<float>(((C2 * A1) - (C1 * A2)) / (det));

        return true;
    }

    return false;
}


bool intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2,
                      Point2f &r)
{
    Point2f x = o2 - o1;
    Point2f d1 = p1 - o1;
    Point2f d2 = p2 - o2;


    float cross = d1.x*d2.y - d1.y*d2.x;
    if (abs(cross) < /*EPS*/1e-8)
        return false;

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    r = o1 + d1 * t1;
    return true;
}

int main(int argc, char** argv){
 initModule_nonfree();
 Mat img_scene, img_object;
 img_scene = imread("Marker3.ppm", CV_LOAD_IMAGE_COLOR);
 if(img_scene.empty()){
     cout << "can not open" << endl;
     return -1;
 }
 for(int i = 1; i < 20; i++){


 stringstream ss;
 ss << setw(2) << setfill('0') << i;
 string s = ss.str();

 std::cout << "opening: " << "marker_corny/marker_corny_" +s +  ".png" << std::endl;
 img_object = imread("marker_corny/marker_corny_" +s +  ".png", CV_LOAD_IMAGE_COLOR);
 if(img_object.empty()){
     cout << "can not open" << endl;
     return 0;
 }

  cv::Ptr<Feature2D> f2d = xfeatures2d::SIFT::create();

  //-- Step 1: Detect the keypoints:
  std::vector<KeyPoint> keypoints_object, keypoints_scene;
  f2d->detect( img_scene, keypoints_object );
  f2d->detect( img_object, keypoints_scene );

  //-- Step 2: Calculate descriptors (feature vectors)
  Mat descriptors_1, descriptors_2;
  f2d->compute( img_scene, keypoints_object, descriptors_1 );
  f2d->compute( img_object, keypoints_scene, descriptors_2 );

  //-- Step 3: Matching descriptor vectors using some matcher
  bool useBruteForceMatcher = false;
  std::vector< DMatch > matches;
  if (useBruteForceMatcher) {
   BFMatcher matcher;
   matcher.match( descriptors_1, descriptors_2, matches );
  } else {
   FlannBasedMatcher mathcer;
   mathcer.match( descriptors_1, descriptors_2, matches );
  }


  double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_1.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }

  printf("-- Max dist : %f \n", max_dist );
  printf("-- Min dist : %f \n", min_dist );


  //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
  //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
  //-- small)
  //-- PS.- radiusMatch can also be used here.
  std::vector< DMatch > good_matches;

  for( int i = 0; i < descriptors_1.rows; i++ )
  { if( matches[i].distance <= max(2*min_dist, 0.02) )
    { good_matches.push_back( matches[i]); }
  }

  //-- Draw only "good" matches
  Mat img_matches;
  drawMatches( img_scene, keypoints_object, img_object, keypoints_scene,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );


//  drawMatches(img_scene, keypoints_object, img_object, keypoints_scene, matches, img_matches);


//  Mat img_matches;

  //-- Show detected matches
  //namedWindow("Good Matches", CV_WINDOW_NORMAL);
  //imshow( "Good Matches", img_matches );



  //-- Localize the object
  std::vector<Point2f> obj;
  std::vector<Point2f> scene;

  for( int i = 0; i < good_matches.size(); i++ ){
    //-- Get the keypoints from the good matches
    obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
  }

  Mat H = findHomography( obj, scene, CV_RANSAC );

  //-- Get the corners from the image_1 ( the object to be "detected" )
  std::vector<Point2f> obj_corners(4);
  obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_scene.cols, 0 );
  obj_corners[2] = cvPoint( img_scene.cols, img_scene.rows ); obj_corners[3] = cvPoint( 0, img_scene.rows );
  std::vector<Point2f> scene_corners(4);

  perspectiveTransform( obj_corners, scene_corners, H);

  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
//  line( img_matches, scene_corners[0] + Point2f( img_scene.cols, 0), scene_corners[1] + Point2f( img_scene.cols, 0), Scalar(0, 255, 0), 4 );
//  line( img_matches, scene_corners[1] + Point2f( img_scene.cols, 0), scene_corners[2] + Point2f( img_scene.cols, 0), Scalar( 0, 255, 0), 4 );
//  line( img_matches, scene_corners[2] + Point2f( img_scene.cols, 0), scene_corners[3] + Point2f( img_scene.cols, 0), Scalar( 0, 255, 0), 4 );
//  line( img_matches, scene_corners[3] + Point2f( img_scene.cols, 0), scene_corners[0] + Point2f( img_scene.cols, 0), Scalar( 0, 255, 0), 4 );

  //-- Show detected matches
  //imshow( "Good Matches & Object detection", img_matches );


  Point2f center_point;
  std::cout << " found intersecting: " << lineIntersection(scene_corners[0], scene_corners[2], scene_corners[1], scene_corners[3], center_point) << std::endl;
  center_point += Point2f( img_scene.cols, 0);
  std::cout << "point center: " << center_point << std::endl;
  circle(img_matches, center_point, 30, Scalar( 0, 255, 0), 10);
 /// Create a window
// namedWindow( "input1", CV_WINDOW_NORMAL );
// namedWindow( "input2", CV_WINDOW_AUTOSIZE );
 namedWindow( "matches", CV_WINDOW_NORMAL );

// imshow("input1", img_scene);
 //imshow("input2", img_scene);
 imshow("matches", img_matches);

 waitKey(20);
}

 return 0;
}
