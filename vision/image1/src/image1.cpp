
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
#include <fstream>
#include <iomanip>
#include <time.h>       /* time */
#include <sys/time.h>       /* time */

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

#define EPSILON 1E-5

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

// *** MARKER RED ***
int hsv_h_base_MR        = HUE_YELLOW_OWN;
int hsv_h_sensitivity_MR = 15; // 5 for big green led and 24 for most other
int hsv_h_low_MR         = hsv_h_base_MR - hsv_h_sensitivity_MR; //hsv_h_base - hsv_h_sensitivity;
int hsv_h_upper_MR       = hsv_h_base_MR + hsv_h_sensitivity_MR;//hsv_h_base + hsv_h_sensitivity;
int hsv_s_low_MR         = 80; //100;
int hsv_s_upper_MR       = 230;
int hsv_v_low_MR         = 90; //100;
int hsv_v_upper_MR       = 210;

std::ofstream timing_data;
int frame_no = 0;

long long currentTimeUs()
// Timer function
{
    timeval current;
    gettimeofday(&current, 0);
    return (long long)current.tv_sec * 1000000L + current.tv_usec;
}

double maximum(double number1, double number2, double number3) {
    return std::max(std::max(number1, number2), number3);
}

bool almostEqual(double number1, double number2) {
    return (std::abs(number1 - number2) <= (EPSILON * maximum(1.0, std::abs(number1), std::abs(number2))));
}

bool lineIntersection(const cv::Point2f &a1, const cv::Point2f &b1, const cv::Point2f &a2,
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

// void on_trackbar( int, void* )
void on_trackbar()
{
  long long time_start = currentTimeUs();
  //cvtColor(image, image, COLOR_BGR2RGB); // Convert from BGR to HSV
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

  Mat im_with_keypoints; // removed the define here!
  // drawKeypoints( image, keypoints_combined, im_with_keypoints, Scalar(0,127,127), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
  // imshow("test",im_with_keypoints);
  waitKey(0);
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
        //putText(image_circle,  patch::to_string(i), keypoints_MB[i].pt,
        //FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0,255,0), 1, CV_AA);
        elements_first_search.push_back(i);
        elements_first_search_means.push_back(mm);
      } else {
        //putText(image_circle,  patch::to_string(i), keypoints_MB[i].pt,
        //FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0,0,255), 1, CV_AA);
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
          //putText(image_circle,  "O", keypoints_MB[elements_first_search[i]].pt,
          //FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0,0,255), 1, CV_AA);
          // Save filtered keypoints for further processing
          keypoints_MB_final.push_back( keypoints_MB.at( elements_first_search.at(i) ));
          keypoints_combined.push_back( keypoints_MB.at( elements_first_search.at(i) ));
        }
      }
    }
  }

  // Calculate center of the blue markers
  double centerX_MB = 0;
  double centerY_MB = 0;
  for (size_t i = 0; i < keypoints_MB_final.size(); i++) {
    centerX_MB += keypoints_MB_final[i].pt.x;
    centerY_MB += keypoints_MB_final[i].pt.y;
  }
  centerX_MB /= keypoints_MB_final.size();
  centerY_MB /= keypoints_MB_final.size();
  Point_<double> center_MB(centerX_MB,centerY_MB);

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

//imshow("test",mask_MR);
  //waitKey(0);

  // Detect blobs.
  std::vector<KeyPoint> keypoints_MR;
  detector->detect( mask_MR, keypoints_MR);

  // Filter keypoints
  double min_dist_MR;
  int min_dist_id;
  if (keypoints_MR.size()) {
    min_dist_MR = sqrt(pow(keypoints_MR.at(0).pt.x-center_MB.x,2)+pow(keypoints_MR.at(0).pt.y-center_MB.y,2));
    min_dist_id = 0;
    //std::cout << "Dist is " << min_dist_MR << std::endl;
    //circle(im_with_keypoints, keypoints_MR.at(0).pt, 2, Scalar(0,127,127), 2);

    for (size_t i = 1; i < keypoints_MR.size(); i++) {
      double tmp_min_dist_MR = sqrt(pow(keypoints_MR.at(i).pt.x-center_MB.x,2)+pow(keypoints_MR.at(i).pt.y-center_MB.y,2));
      //std::cout << "Dist is " << tmp_min_dist_MR << std::endl;
      if (tmp_min_dist_MR < min_dist_MR) {
        min_dist_id = i;
        min_dist_MR = tmp_min_dist_MR;
      }
    }
    // Save keypoints
    keypoints_MR_final.push_back( keypoints_MR.at(min_dist_id) );
    keypoints_combined.push_back( keypoints_MR.at(min_dist_id) );
  }

  //im_with_keypoints; // removed the define here!
  drawKeypoints( image, keypoints_combined, im_with_keypoints, Scalar(0,127,127), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
  //circle(im_with_keypoints, center_MB, 2, Scalar(0,127,255), 2);

  // Sort markers in the right order
  int MB1_id = 0;
  int MB2_id = 0;
  int MB3_id = 0;


  if (keypoints_MR_final.size() == 1) {
    int MB1_x = keypoints_MB_final.at(0).pt.x;
    int MB2_y = keypoints_MB_final.at(0).pt.y;
    int MB3_x = keypoints_MB_final.at(0).pt.x;
    int MB3_y = keypoints_MB_final.at(0).pt.y;
    if (keypoints_MR_final.at(0).pt.x <= center_MB.x
        and keypoints_MR_final.at(0).pt.y <= center_MB.y) { // Red Marker is at left top
      if (keypoints_MB_final.at(MB1_id).pt.y > keypoints_MR_final.at(0).pt.y) { // Match blue marker 1
        MB1_id = 1;
      }
      for (size_t i = 0; i < keypoints_MB_final.size(); i++) {
        if (i != MB1_id) {
          if (keypoints_MB_final.at(i).pt.x <= keypoints_MB_final.at(MB1_id).pt.x) {
            if (keypoints_MB_final.at(i).pt.y > keypoints_MR_final.at(0).pt.y) {
              MB1_id = i;
            }
          }
        }
      }
      if (MB1_id == MB2_id) { // Match blue marker 2
        MB2_id = 1;
      }
      for (size_t i = 0; i < keypoints_MB_final.size(); i++) {
        if (i != MB1_id and i != MB2_id) {
          if (keypoints_MB_final.at(i).pt.y <= keypoints_MB_final.at(MB2_id).pt.y) {
            MB2_id = i;
          }
        }
      }
      for (size_t i = 1; i < keypoints_MB_final.size(); i++) { // Match blue marker 3
        if (keypoints_MB_final.at(i).pt.x > keypoints_MB_final.at(MB1_id).pt.x
            and keypoints_MB_final.at(i).pt.y > keypoints_MB_final.at(MB2_id).pt.y) {
          MB3_id = i;
        }
      }
    } else if (keypoints_MR_final.at(0).pt.x <= center_MB.x
        and keypoints_MR_final.at(0).pt.y > center_MB.y) { // Red Marker is at left bottom
      if (keypoints_MB_final.at(MB1_id).pt.x > keypoints_MR_final.at(0).pt.x) { // Match blue marker 1
        MB1_id = 1;
      }
      for (size_t i = 0; i < keypoints_MB_final.size(); i++) {
        if (i != MB1_id) {
          if (keypoints_MB_final.at(i).pt.y >= keypoints_MB_final.at(MB1_id).pt.y) {
            if (keypoints_MB_final.at(i).pt.x > keypoints_MR_final.at(0).pt.x) {
              MB1_id = i;
            }
          }
        }
      }
      if (MB1_id == MB2_id) { // Match blue marker 2
        MB2_id = 1;
      }
      for (size_t i = 0; i < keypoints_MB_final.size(); i++) {
        if (i != MB1_id and i != MB2_id) {
          if (keypoints_MB_final.at(i).pt.x <= keypoints_MB_final.at(MB2_id).pt.x) {
            MB2_id = i;
          }
        }
      }
      for (size_t i = 1; i < keypoints_MB_final.size(); i++) { // Match blue marker 3
        if (keypoints_MB_final.at(i).pt.x > keypoints_MB_final.at(MB2_id).pt.x
            and keypoints_MB_final.at(i).pt.y < keypoints_MB_final.at(MB1_id).pt.y) {
          MB3_id = i;
        }
      }
    } else if (keypoints_MR_final.at(0).pt.x > center_MB.x
        and keypoints_MR_final.at(0).pt.y <= center_MB.y) { // Red Marker is at right top
      if (keypoints_MB_final.at(MB1_id).pt.x < keypoints_MR_final.at(0).pt.x) { // Match blue marker 1
        MB1_id = 1;
      }
      for (size_t i = 0; i < keypoints_MB_final.size(); i++) {
        if (i != MB1_id) {
          if (keypoints_MB_final.at(i).pt.y <= keypoints_MB_final.at(MB1_id).pt.y) {
            if (keypoints_MB_final.at(i).pt.x < keypoints_MR_final.at(0).pt.x) {
              MB1_id = i;
            }
          }
        }
      }
      if (MB1_id == MB2_id) { // Match blue marker 2
        MB2_id = 1;
      }
      for (size_t i = 0; i < keypoints_MB_final.size(); i++) {
        if (i != MB1_id and i != MB2_id) {
          if (keypoints_MB_final.at(i).pt.x >= keypoints_MB_final.at(MB2_id).pt.x) {
            MB2_id = i;
          }
        }
      }
      for (size_t i = 1; i < keypoints_MB_final.size(); i++) { // Match blue marker 3
        if (keypoints_MB_final.at(i).pt.x < keypoints_MB_final.at(MB2_id).pt.x
            and keypoints_MB_final.at(i).pt.y > keypoints_MB_final.at(MB1_id).pt.y) {
          MB3_id = i;
        }
      }
    } else if (keypoints_MR_final.at(0).pt.x > center_MB.x
        and keypoints_MR_final.at(0).pt.y > center_MB.y) { // Red Marker is at right bottom
      if (keypoints_MB_final.at(MB1_id).pt.y > keypoints_MR_final.at(0).pt.y) { // Match blue marker 1
        MB1_id = 1;
      }
      for (size_t i = 0; i < keypoints_MB_final.size(); i++) {
        if (i != MB1_id) {
          if (keypoints_MB_final.at(i).pt.x >= keypoints_MB_final.at(MB1_id).pt.x) {
            if (keypoints_MB_final.at(i).pt.y < keypoints_MR_final.at(0).pt.y) {
              MB1_id = i;
            }
          }
        }
      }
      if (MB1_id == MB2_id) { // Match blue marker 2
        MB2_id = 1;
      }
      for (size_t i = 0; i < keypoints_MB_final.size(); i++) {
        if (i != MB1_id and i != MB2_id) {
          if (keypoints_MB_final.at(i).pt.y >= keypoints_MB_final.at(MB2_id).pt.y) {
            MB2_id = i;
          }
        }
      }
      for (size_t i = 1; i < keypoints_MB_final.size(); i++) { // Match blue marker 3
        if (keypoints_MB_final.at(i).pt.x < keypoints_MB_final.at(MB1_id).pt.x
            and keypoints_MB_final.at(i).pt.y < keypoints_MB_final.at(MB2_id).pt.y) {
          MB3_id = i;
        }
      }
    }
  }

  std::cout << "Number of keypoints \t\t" << keypoints_combined.size() << std::endl;
  std::cout << "Number of red keypoints \t" << keypoints_MR_final.size() << std::endl;
  std::cout << "Number of blue keypoints \t" << keypoints_MB_final.size() << std::endl;

  std::vector< Point2f > output_points; // Elements: 1=MB1, 2=MB2, 3=MB3, 4=MB4, 5=center

  if (keypoints_MB_final.size() == 3 and keypoints_MR_final.size() == 1) {
    putText(im_with_keypoints,  "MB1", keypoints_MB_final[MB1_id].pt,
    FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0,127,127), 1, CV_AA);
    putText(im_with_keypoints,  "MB2", keypoints_MB_final[MB2_id].pt,
    FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0,127,127), 1, CV_AA);
    putText(im_with_keypoints,  "MB3", keypoints_MB_final[MB3_id].pt,
    FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0,127,127), 1, CV_AA);
    putText(im_with_keypoints,  "MR1", keypoints_MR_final[0].pt,
    FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0,127,127), 1, CV_AA);

    //circle(im_with_keypoints, center_MB, 2, Scalar(0,127,255), 2);

    Point2f tmp_point;
    tmp_point = keypoints_MB_final.at(MB1_id).pt;
    output_points.push_back(tmp_point);
    tmp_point = keypoints_MB_final.at(MB2_id).pt;
    output_points.push_back(tmp_point);
    tmp_point = keypoints_MB_final.at(MB3_id).pt;
    output_points.push_back(tmp_point);
    tmp_point = keypoints_MR_final.at(0).pt;
    output_points.push_back(tmp_point);

    Point2f center_point;
    if(!lineIntersection(output_points[0], output_points[1], output_points[3], output_points[2], center_point)){
      std::cout << "Could not find intersection" << std::endl;
    } else {
      circle(im_with_keypoints, center_point, 2, Scalar(127,0,0), 2);
    }
    output_points.push_back(center_point);

    // Save data
    // timing_data << setprecision(8) << << "\n";
    long long time_end = currentTimeUs();
    timing_data.open ("timing_data.csv",std::fstream::app|std::fstream::out);
    timing_data << std::setprecision(8) << frame_no << "," << keypoints_MB_final.size() << "," << keypoints_MR_final.size() << "," << output_points[0].x  << "," << output_points[0].y << "," << output_points[1].x << "," << output_points[1].y << "," << output_points[2].x << "," << output_points[2].y << "," << output_points[3].x << "," << output_points[3].y << "," << output_points[4].x << "," << output_points[4].y << "," << (time_end-time_start) << "\n";    timing_data.close();
  }

  // Point2f center_point;
  // if(!lineIntersection(corners[0], corners[2], corners[1], corners[3], center_point)){
  //   std::cout << "Could not find SIFT intersection" << std::endl;
  // }

  //imshow("Display Image", image);
  //imshow("Display Masked", image_masked_MR);
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
  //namedWindow("Display Masked", CV_WINDOW_AUTOSIZE );
  namedWindow("Display Keypoints", CV_WINDOW_AUTOSIZE );
  //namedWindow("Trackbars", CV_WINDOW_AUTOSIZE );
  //namedWindow("Circle", CV_WINDOW_AUTOSIZE );
  //namedWindow("Shifted HSV", CV_WINDOW_AUTOSIZE );
  //namedWindow("Shifted BGR", CV_WINDOW_AUTOSIZE );

  //createTrackbar( "Rho:", "Hough line detection", &hough_rho, hough_rho_max, on_trackbar );
  //createTrackbar("HUE LOW", "Trackbars", &hsv_h_low_MR, 255);
  //createTrackbar("HUE UPPER", "Trackbars", &hsv_h_upper_MR, 255);
  //createTrackbar("SATURATION LOW", "Trackbars", &hsv_s_low_MR, 255);
  //createTrackbar("SATURATION UPPER", "Trackbars", &hsv_s_upper_MR, 255);
  //createTrackbar("VIBRANCE LOW", "Trackbars", &hsv_v_low_MR, 255);
  //createTrackbar("VIBRANCE UPPER", "Trackbars", &hsv_v_upper_MR, 255);

  waitKey(3000);

  timing_data.open ("timing_data.csv");
  timing_data << std::setprecision(8) << "FrameNo,NoMB,NoMR,MB1_x,MB1_y,MB2_x,MB2_y,MB3_x,MB3_y,MR1_x,MR2_y,center_x,center_y,analysis_time" << "\n";
  timing_data.close();


  for (size_t i = 1; i <= 52; i++) {
    frame_no = i;
    std::stringstream ss;
    ss << std::setw(2) << std::setfill('0') << i;
    std::string s = ss.str();

    std::string file_id = "marker_color";
    std::cout << "opening: " << file_id << "_" +s +  ".png" << std::endl;
    image = imread("../../sequences/" + file_id + "/" + file_id + "_" + s +  ".png", 1);
    on_trackbar();
    //waitKey(0);
  }

  // image = imread("from-camera.png", 1);
  // while(true){
  //   on_trackbar();
  //   waitKey(0);
  // }

  waitKey(0);
  return 0;
}
