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
#define HUE_VIOLET     145                      /* 130-160 */
#define HUE_RED        160                      /* 160-179 */

#define HUE_YELLOW_OWN 15                      /* 22-38 */
#define HUE_BLUE_OWN   120                      /* 75-130 */


// Global variables - only due to trackbar
Mat image;

int dilate_color_iterations = 1; //  effect not tested

SimpleBlobDetector::Params params;
Ptr<SimpleBlobDetector> detector;

// *** MARKER BLUE ***
int hsv_h_base_MB        = HUE_BLUE_OWN;
int hsv_h_sensitivity_MB = 25; // 5 for big green led and 24 for most other
int hsv_h_low_MB         = hsv_h_base_MB - hsv_h_sensitivity_MB; //hsv_h_base - hsv_h_sensitivity;
int hsv_h_upper_MB       = hsv_h_base_MB + hsv_h_sensitivity_MB;//hsv_h_base + hsv_h_sensitivity;
int hsv_s_low_MB         = 30; //100;
int hsv_s_upper_MB       = 255;
int hsv_v_low_MB         = 30; //100;
int hsv_v_upper_MB       = 255;

// *** MARKER BLUE ***
int hsv_h_base_MR        = HUE_YELLOW_OWN;
int hsv_h_sensitivity_MR = 15; // 5 for big green led and 24 for most other
int hsv_h_low_MR         = hsv_h_base_MR - hsv_h_sensitivity_MR; //hsv_h_base - hsv_h_sensitivity;
int hsv_h_upper_MR       = hsv_h_base_MR + hsv_h_sensitivity_MR;//hsv_h_base + hsv_h_sensitivity;
int hsv_s_low_MR         = 80; //100;
int hsv_s_upper_MR       = 230;
int hsv_v_low_MR         = 90; //100;
int hsv_v_upper_MR       = 210;

// void on_trackbar( int, void* )
void on_trackbar()
{
  // Create / convert images to color spaces for processing
  Mat image_hsv, image_gray;
  cvtColor(image, image_hsv,  COLOR_BGR2HSV); // Convert from BGR to HSV
  cvtColor(image, image_gray, COLOR_BGR2GRAY); //Convert the captured frame from BGR to GRAY
  //GaussianBlur(frame_gray, frame_gray_with_Gblur, Size(3, 3), 0); // Gaussian blur on gray frame, 1st arg: input frame; 2nd arg: output frame; 3rd arg: defines the blur radius; 4th arg: Gaussian kernel standard deviation in X direction, when this is 0 it is computed from the 3rd arg. Gaussaian blur is used since an example used this.

  // Mask blue parts
  Mat mask_MB, image_masked_MB;
  inRange(image_hsv, Scalar(hsv_h_low_MB, hsv_s_low_MB, hsv_v_low_MB), Scalar(hsv_h_upper_MB, hsv_s_upper_MB, hsv_v_upper_MB), mask_MB); // Find the areas which contain the color. 1st arg: inpur frame; 2nd arg: the lower HSV limits; 3rd arg: the upper HSV limits; 4th arg: the output mask.
  for (size_t i = 0; i < dilate_color_iterations; i++)
    dilate(mask_MB, mask_MB, Mat(), Point(-1,-1)); // Enhance the areas in the image
  image_gray.copyTo(image_masked_MB, mask_MB);

  // Detect blobs.
  std::vector<KeyPoint> keypoints_MB, keypoints_MB_final, keypoints_MR_final, keypoints_combined;
  detector->detect( mask_MB, keypoints_MB);

  // Filter found blobs
  Mat image_circle;
  image_circle = Mat::zeros(image.size(), image.type());
  if (keypoints_MB.size()) {
    Scalar mm;
    std::vector< int > elements_first_search;
    std::vector< Scalar > elements_first_search_means;
    for (size_t i = 0; i < keypoints_MB.size(); i++) {
      Mat mask_tmp = Mat::zeros(image_circle.size(), CV_8U);

      // Create mask area by two circles with different size within each other
      circle(mask_tmp, keypoints_MB[i].pt, keypoints_MB[i].size/2 * 1.20, Scalar::all(255), -1);
      circle(mask_tmp, keypoints_MB[i].pt, keypoints_MB[i].size/2, Scalar::all(0), -1);

      // Mask out from the image to get the glows / outline for each marker
      image.copyTo(image_circle, mask_tmp);

      mm = mean(image_hsv, mask_tmp); // Compute mean but only of mask

      if ( (mm[0] > 60 and mm[0] < 95) ) {
        putText(image_circle,  patch::to_string(i), keypoints_MB[i].pt,
        FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0,255,0), 1, CV_AA);
        elements_first_search.push_back(i);
        elements_first_search_means.push_back(mm);
      } else {
        putText(image_circle,  patch::to_string(i), keypoints_MB[i].pt,
        FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0,0,255), 1, CV_AA);
      }
    }
    // Filter markers 2nd run
    if (elements_first_search.size()) {
      // Compute average mean and avg marker size
      int average_means_first_search_h = 0;
      int average_means_first_search_s = 0;
      int average_means_first_search_v = 0;
      int average_means_first_search_size = 0;
      for (size_t i = 0; i < elements_first_search_means.size(); i++) {
        average_means_first_search_h += elements_first_search_means.at(i)[0];
        average_means_first_search_s += elements_first_search_means.at(i)[1];
        average_means_first_search_v += elements_first_search_means.at(i)[2];
        average_means_first_search_size += keypoints_MB[elements_first_search[i]].size;
      }
      average_means_first_search_h /= elements_first_search_means.size();
      average_means_first_search_s /= elements_first_search_means.size();
      average_means_first_search_v /= elements_first_search_means.size();
      average_means_first_search_size /= elements_first_search_means.size();

      // Threasholds for average filtering
      int threashold_h = 10;
      int threashold_s = 25;
      int threashold_v = 35;
      int threashold_size = 25;

      for (size_t i = 0; i < elements_first_search.size(); i++) {
        if ( (elements_first_search_means.at(i)[2] > average_means_first_search_v-threashold_v and elements_first_search_means.at(i)[2] < average_means_first_search_v+threashold_v)
          and (elements_first_search_means.at(i)[1] > average_means_first_search_s-threashold_s and elements_first_search_means.at(i)[1] < average_means_first_search_s+threashold_s)
          and (elements_first_search_means.at(i)[0] > average_means_first_search_h-threashold_h and elements_first_search_means.at(i)[0] < average_means_first_search_h+threashold_h)
          and (keypoints_MB[elements_first_search[i]].size > average_means_first_search_size-average_means_first_search_size*(threashold_size/100.0) and keypoints_MB[elements_first_search[i]].size < average_means_first_search_size+average_means_first_search_size*(threashold_size/100.0)) ) {
          putText(image_circle,  "O", keypoints_MB[elements_first_search[i]].pt,
          FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0,0,255), 1, CV_AA);
          // Save filtered keypoints for further processing
          keypoints_MB_final.push_back( keypoints_MB.at( elements_first_search.at(i) ));
          keypoints_combined.push_back( keypoints_MB.at( elements_first_search.at(i) ));
        }
      }
    }
  }

  // Std draw all the keypoints, not the filtered
  Mat im_with_keypoints;
  drawKeypoints( image, keypoints_MB_final, im_with_keypoints, Scalar(0,127,127), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
  // Draw radius
  if (keypoints_MB_final.size()) {
    for (size_t i = 0; i < keypoints_MB_final.size(); i++) {
      putText(im_with_keypoints,  patch::to_string(i), keypoints_MB_final[i].pt,
      FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0,127,127), 1, CV_AA);
      line(im_with_keypoints, keypoints_MB_final[i].pt, Point(keypoints_MB_final[i].pt.x+(keypoints_MB_final[i].size/2),keypoints_MB_final[i].pt.y), cvScalar(0,127,255));
    }
  }

  double centerX_MB = 0;
  double centerY_MB = 0;
  for (size_t i = 0; i < keypoints_MB_final.size(); i++) {
    centerX_MB += keypoints_MB_final[i].pt.x;
    centerY_MB += keypoints_MB_final[i].pt.y;
  }
  centerX_MB /= keypoints_MB_final.size();
  centerY_MB /= keypoints_MB_final.size();
  Point_<double> center_MB(centerX_MB,centerY_MB);
  circle(im_with_keypoints, center_MB, 2, Scalar(0,127,255), 2);


  // Shift hue values to color seperate better for red
  Mat image_hsv_shifted;
  image_hsv.copyTo(image_hsv_shifted);
  for(int y=0; y < image_hsv_shifted.rows; y++)
  {
    for(int x=0; x < image_hsv_shifted.cols; x++)
    {
      int tmp_color = image_hsv_shifted.at<Vec3b>(Point(x,y))[0];
      tmp_color += 60;
      if (tmp_color >= 360)
        tmp_color = tmp_color%360;
      image_hsv_shifted.at<Vec3b>(Point(x,y))[0] = tmp_color;
    }
  }
  Mat image_bgr_shifted;
  cvtColor(image_hsv_shifted, image_bgr_shifted,  COLOR_HSV2BGR); // Convert from BGR to HSV

  // Mask red parts
  Mat mask_MR, image_masked_MR;
  inRange(image_hsv, Scalar(hsv_h_low_MR, hsv_s_low_MR, hsv_v_low_MR), Scalar(hsv_h_upper_MR, hsv_s_upper_MR, hsv_v_upper_MR), mask_MR); // Find the areas which contain the color. 1st arg: inpur frame; 2nd arg: the lower HSV limits; 3rd arg: the upper HSV limits; 4th arg: the output mask.
  for (size_t i = 0; i < dilate_color_iterations; i++)
    dilate(mask_MR, mask_MR, Mat(), Point(-1,-1)); // Enhance the areas in the image
  image_gray.copyTo(image_masked_MR, mask_MR);

  // Detect blobs.
  std::vector<KeyPoint> keypoints_MR;
  detector->detect( mask_MR, keypoints_MR);

  // Filter keypoints
  double min_dist_MR;
  int min_dist_id;
  if (keypoints_MR.size()) {
    min_dist_MR = sqrt(pow(keypoints_MR.at(0).pt.x-center_MB.x,2)+pow(keypoints_MR.at(0).pt.y-center_MB.y,2));
    min_dist_id = 0;
    std::cout << "Dist is " << min_dist_MR << std::endl;
    //circle(im_with_keypoints, keypoints_MR.at(0).pt, 2, Scalar(0,127,127), 2);

    for (size_t i = 1; i < keypoints_MR.size(); i++) {
      double tmp_min_dist_MR = sqrt(pow(keypoints_MR.at(i).pt.x-center_MB.x,2)+pow(keypoints_MR.at(i).pt.y-center_MB.y,2));
      std::cout << "Dist is " << tmp_min_dist_MR << std::endl;
      if (tmp_min_dist_MR < min_dist_MR) {
        min_dist_id = i;
        min_dist_MR = tmp_min_dist_MR;
      }
    }
  }
  // Save keypoints
  keypoints_MR_final.push_back( keypoints_MR.at(min_dist_id) );
  keypoints_combined.push_back( keypoints_MR.at(min_dist_id) );
  drawKeypoints( image, keypoints_combined, im_with_keypoints, Scalar(0,127,127), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
  circle(im_with_keypoints, center_MB, 2, Scalar(0,127,255), 2);

  if (keypoints_MR_final.size()) {
    for (size_t i = 0; i < keypoints_MR.size(); i++) {
      putText(im_with_keypoints,  "red", keypoints_MR_final[i].pt,
      FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0,127,127), 1, CV_AA);
    }
  }
  if (keypoints_MB_final.size()) {
    for (size_t i = 0; i < keypoints_MB.size(); i++) {
      putText(im_with_keypoints,  "blue", keypoints_MB_final[i].pt,
      FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0,127,127), 1, CV_AA);
    }
  }

  std::cout << "Number of keypoints " << keypoints_combined.size() << std::endl;
  std::cout << "Number of red keypoints " << keypoints_MR.size() << std::endl;

  //imshow("Display Image", image);
  imshow("Display Masked", image_masked_MR);
  imshow("Display Keypoints", im_with_keypoints);
  //imshow("Circle", image_circle);

  //imshow("Shifted HSV", image_hsv_shifted);

  //imshow("Shifted BGR",image_bgr_shifted);
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
  //namedWindow("Shifted HSV", CV_WINDOW_AUTOSIZE );
  //namedWindow("Shifted BGR", CV_WINDOW_AUTOSIZE );

  //createTrackbar( "Rho:", "Hough line detection", &hough_rho, hough_rho_max, on_trackbar );
  createTrackbar("HUE LOW", "Trackbars", &hsv_h_low_MR, 255);
  createTrackbar("HUE UPPER", "Trackbars", &hsv_h_upper_MR, 255);
  createTrackbar("SATURATION LOW", "Trackbars", &hsv_s_low_MR, 255);
  createTrackbar("SATURATION UPPER", "Trackbars", &hsv_s_upper_MR, 255);
  createTrackbar("VIBRANCE LOW", "Trackbars", &hsv_v_low_MR, 255);
  createTrackbar("VIBRANCE UPPER", "Trackbars", &hsv_v_upper_MR, 255);

  waitKey(3000);

  for (size_t i = 1; i <= 52; i++) {
    std::stringstream ss;
    ss << std::setw(2) << std::setfill('0') << i;
    std::string s = ss.str();

    std::string file_id = "marker_color_hard";
    std::cout << "opening: " << file_id << "_" +s +  ".png" << std::endl;
    image = imread("../../sequences/" + file_id + "/" + file_id + "_" + s +  ".png", 1);
    on_trackbar();
    waitKey(0);
  }

  waitKey(0);
  return 0;
}
