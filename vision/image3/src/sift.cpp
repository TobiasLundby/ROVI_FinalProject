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

int main(int argc, char** argv){
  initModule_nonfree();
 Mat img_1, img_2;
 img_1 = imread("cola.jpg", CV_LOAD_IMAGE_COLOR);
 if(img_1.empty())
 {
     cout << "can not open" << endl;
     return -1;
 }
 img_2 = imread("background.jpg", CV_LOAD_IMAGE_COLOR);
 if(img_2.empty())
 {
     cout << "can not open" << endl;
     return -1;
 }

  cv::Ptr<Feature2D> f2d = xfeatures2d::SURF::create();

  //-- Step 1: Detect the keypoints:
  std::vector<KeyPoint> keypoints_1, keypoints_2;    
  f2d->detect( img_1, keypoints_1 );
  f2d->detect( img_2, keypoints_2 );

  //-- Step 2: Calculate descriptors (feature vectors)    
  Mat descriptors_1, descriptors_2;    
  f2d->compute( img_1, keypoints_1, descriptors_1 );
  f2d->compute( img_2, keypoints_2, descriptors_2 );

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


  Mat img_matches;
  drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_matches);


 /// Create a window
 namedWindow( "input1", CV_WINDOW_NORMAL );
// namedWindow( "input2", CV_WINDOW_AUTOSIZE );
 namedWindow( "matches", CV_WINDOW_NORMAL );

 imshow("input1", img_1);
 //imshow("input2", img_1);
 imshow("matches", img_matches);

 waitKey();


 return 0;
}
