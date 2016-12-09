/*
 *  image1.cpp
 *  ROVI - Final Project - Visual Servoing
 *
 */

// Includes
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <iomanip>

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

// Namespaces
using namespace cv;

// Defines
#define HUE_ORANGE     11                      /* 0-22 */
#define HUE_YELLOW     30                      /* 22-38 */
#define HUE_GREEN      60                      /* 38-75 */
#define HUE_BLUE       100                      /* 75-130 */
#define HUE_BLUE_OWN   120                      /* 75-130 */
#define HUE_VIOLET     145                      /* 130-160 */
#define HUE_RED        160                      /* 160-179 */

// Global variables - only due to trackbar
Mat image;

int hsv_h_base        = HUE_BLUE_OWN;
int hsv_h_sensitivity = 25; // 5 for big green led and 24 for most other
int hsv_h_low         = hsv_h_base - hsv_h_sensitivity; //hsv_h_base - hsv_h_sensitivity;
int hsv_h_upper       = hsv_h_base + hsv_h_sensitivity;//hsv_h_base + hsv_h_sensitivity;
int hsv_s_low         = 30; //100;
int hsv_s_upper       = 255;
int hsv_v_low         = 30; //100;
int hsv_v_upper       = 255;

int dilate_color_iterations = 1; //  effect not tested

SimpleBlobDetector::Params params;
Ptr<SimpleBlobDetector> detector;

// void on_trackbar( int, void* )
void on_trackbar()
{

  Mat image_hsv, image_gray, image_masked;
  cvtColor(image, image_hsv,  COLOR_BGR2HSV); // Convert from BGR to HSV
  cvtColor(image, image_gray, COLOR_BGR2GRAY); //Convert the captured frame from BGR to GRAY
  //GaussianBlur(frame_gray, frame_gray_with_Gblur, Size(3, 3), 0); // Gaussian blur on gray frame, 1st arg: input frame; 2nd arg: output frame; 3rd arg: defines the blur radius; 4th arg: Gaussian kernel standard deviation in X direction, when this is 0 it is computed from the 3rd arg. Gaussaian blur is used since an example used this.

  Mat mask;
  inRange(image_hsv, Scalar(hsv_h_low,hsv_s_low,hsv_v_low), Scalar(hsv_h_upper, hsv_s_upper, hsv_v_upper), mask); // Find the areas which contain the color. 1st arg: inpur frame; 2nd arg: the lower HSV limits; 3rd arg: the upper HSV limits; 4th arg: the output mask.
  for (size_t i = 0; i < dilate_color_iterations; i++)
    dilate(mask, mask, Mat(), Point(-1,-1)); // Enhance the areas in the image
  image_gray.copyTo(image_masked, mask);

  // Detect blobs.
  std::vector<KeyPoint> keypoints;
  detector->detect( mask, keypoints);

  Mat image_circle;
  //image.copyTo(image_circle);
  image_circle = Mat::zeros(image.size(), image.type());
  if (keypoints.size()) {
    Scalar mm;
    std::vector< int > elements_first_search;
    std::vector< Scalar > elements_first_search_means;
    for (size_t i = 0; i < keypoints.size(); i++) {
      Mat mask_tmp = Mat::zeros(image_circle.size(), CV_8U);

      circle(mask_tmp, keypoints[i].pt, keypoints[i].size/2 * 1.20, Scalar::all(255), -1);
      circle(mask_tmp, keypoints[i].pt, keypoints[i].size/2, Scalar::all(0), -1);

      image.copyTo(image_circle, mask_tmp);

      mm = mean(image_hsv, mask_tmp);
      std::cout << mm[0] << "\t" << mm[1] << "\t" << mm[2] << std::endl;

      if ( (mm[0] > 60 and mm[0] < 95) ) {
        putText(image_circle,  patch::to_string(i), keypoints[i].pt,
        FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0,255,0), 1, CV_AA);
        elements_first_search.push_back(i);
        elements_first_search_means.push_back(mm);
      } else {
        putText(image_circle,  patch::to_string(i), keypoints[i].pt,
        FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0,0,255), 1, CV_AA);
      }
    }
    if (elements_first_search.size()) {
      int average_means_first_search_h = 0;
      int average_means_first_search_s = 0;
      int average_means_first_search_v = 0;
      int average_means_first_search_size = 0;
      for (size_t i = 0; i < elements_first_search_means.size(); i++) {
        average_means_first_search_h += elements_first_search_means.at(i)[0];
        average_means_first_search_s += elements_first_search_means.at(i)[1];
        average_means_first_search_v += elements_first_search_means.at(i)[2];
        average_means_first_search_size += keypoints[elements_first_search[i]].size;
      }
      average_means_first_search_h = average_means_first_search_h / elements_first_search_means.size();
      average_means_first_search_s = average_means_first_search_s / elements_first_search_means.size();
      average_means_first_search_v = average_means_first_search_v / elements_first_search_means.size();
      average_means_first_search_size = average_means_first_search_size / elements_first_search_means.size();
      std::cout << "Average h: " << average_means_first_search_h << std::endl;
      std::cout << "Average s: " << average_means_first_search_s << std::endl;
      std::cout << "Average v: " << average_means_first_search_v << std::endl;
      std::cout << "Average size: " << average_means_first_search_size << std::endl;

      int threashold_h = 10;
      int threashold_s = 25;
      int threashold_v = 35;
      int threashold_size = 25;

      std::cout << "cmp: " << average_means_first_search_size*(threashold_size/100.0) << std::endl;

      for (size_t i = 0; i < elements_first_search.size(); i++) {
        if ( (elements_first_search_means.at(i)[2] > average_means_first_search_v-threashold_v and elements_first_search_means.at(i)[2] < average_means_first_search_v+threashold_v)
          and (elements_first_search_means.at(i)[1] > average_means_first_search_s-threashold_s and elements_first_search_means.at(i)[1] < average_means_first_search_s+threashold_s)
          and (elements_first_search_means.at(i)[0] > average_means_first_search_h-threashold_h and elements_first_search_means.at(i)[0] < average_means_first_search_h+threashold_h)
          and (keypoints[elements_first_search[i]].size > average_means_first_search_size-average_means_first_search_size*(threashold_size/100.0) and keypoints[elements_first_search[i]].size < average_means_first_search_size+average_means_first_search_size*(threashold_size/100.0)) ) {
          putText(image_circle,  "O", keypoints[elements_first_search[i]].pt,
          FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0,0,255), 1, CV_AA);
        }
      }
    }
    //image.copyTo(image_circle, mask_tmp);
  }
  Mat im_with_keypoints;
  drawKeypoints( image, keypoints, im_with_keypoints, Scalar(0,127,127), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

  std::cout << "Keypoints: " << keypoints.size() << std::endl;
  if (keypoints.size()) {
    for (size_t i = 0; i < keypoints.size(); i++) {
      putText(im_with_keypoints,  patch::to_string(i), keypoints[i].pt,
      FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0,127,127), 1, CV_AA);
      std::cout << "Keypoint " << patch::to_string(i) << " has size " << patch::to_string(keypoints[i].size) << std::endl;
      line(im_with_keypoints, keypoints[i].pt, Point(keypoints[i].pt.x+(keypoints[i].size/2),keypoints[i].pt.y), cvScalar(0,127,255));
    }
  }
  //imshow("Display Image", image);
  imshow("Display Masked", image_masked);
  imshow("Display Keypoints", im_with_keypoints);
  imshow("Circle", image_circle);
}

int main(int argc, char **argv) {
  if ( argc != 2 )
  {
    printf("Use <Image_Path> to provide custom image\n");
    image = imread("Marker1.ppm", 1);
  } else {
    image = imread( argv[1], 1 );
  }
  if ( !image.data )
  {
    printf("No image data \n");
    return -1;
  }

  // All the parameters below are used for the blob detector - only some of them are used because motion blur introduces different shapes etc.
  // Change thresholds
  //params.minThreshold = 0;
  //params.maxThreshold = 100;
  params.blobColor = 255; // This parameter is used instead of the threadholds to detect the white color
  // Filter by Area.
  params.filterByArea = true;
  params.minArea = CV_PI*pow(30,2); // Radius 50px
  params.maxArea = CV_PI*pow(100,2); // Radius 100px
  // Filter by Circularity - we do not do this parameter due to motion blur
  params.filterByCircularity = true;
  params.minCircularity = 0.5;
  // Filter by Convexity - we do not use this parameter to ensure detection
  params.filterByConvexity = false;
  //params.minConvexity = 0.87;
  // Filter by Inertia - not used but it means "the inertial resistance of the blob to rotation about its principal axes"
  params.filterByInertia = false;
  //params.minInertiaRatio = 0.01;
  //params.maxInertiaRatio = 0.5;
  detector = SimpleBlobDetector::create(params); // Set up detector with params

  //namedWindow("Display Image", CV_WINDOW_AUTOSIZE );
  namedWindow("Display Masked", CV_WINDOW_AUTOSIZE );
  namedWindow("Display Keypoints", CV_WINDOW_AUTOSIZE );
  namedWindow("Trackbars", CV_WINDOW_AUTOSIZE );
  namedWindow("Circle", CV_WINDOW_AUTOSIZE );

  //createTrackbar( "Rho:", "Hough line detection", &hough_rho, hough_rho_max, on_trackbar );
  createTrackbar("HUE LOW", "Trackbars", &hsv_h_low, 255);
  createTrackbar("HUE UPPER", "Trackbars", &hsv_h_upper, 255);
  createTrackbar("SATURATION LOW", "Trackbars", &hsv_s_low, 255);
  createTrackbar("SATURATION UPPER", "Trackbars", &hsv_s_upper, 255);
  createTrackbar("VIBRANCE LOW", "Trackbars", &hsv_v_low, 255);
  createTrackbar("VIBRANCE UPPER", "Trackbars", &hsv_v_upper, 255);

  waitKey(3000);

  for (size_t i = 1; i <= 52; i++) {
    std::stringstream ss;
    ss << std::setw(2) << std::setfill('0') << i;
    std::string s = ss.str();

    std::string file_id = "marker_color";
    std::cout << "opening: " << file_id << "_" +s +  ".png" << std::endl;
    image = imread("../../sequences/" + file_id + "/" + file_id + "_" + s +  ".png", 1);
    on_trackbar();
    waitKey(0);
  }

  waitKey(0);
  return 0;
}
