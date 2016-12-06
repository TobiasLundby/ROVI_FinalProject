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
int hsv_h_sensitivity = 20; // 5 for big green led and 24 for most other
int hsv_h_low         = hsv_h_base - hsv_h_sensitivity; //hsv_h_base - hsv_h_sensitivity;
int hsv_h_upper       = hsv_h_base + hsv_h_sensitivity;//hsv_h_base + hsv_h_sensitivity;
int hsv_s_low         = 50; //100;
int hsv_s_upper       = 255;
int hsv_v_low         = 50; //100;
int hsv_v_upper       = 255;

int dilate_color_iterations = 0; //  effect not tested

SimpleBlobDetector::Params params;
Ptr<SimpleBlobDetector> detector;

void on_trackbar( int, void* )
{

  Mat image_hsv, image_gray, image_masked;
  cvtColor(image, image_hsv,  COLOR_BGR2HSV); // Convert from BGR to HSV
  cvtColor(image, image_gray, COLOR_BGR2GRAY); //Convert the captured frame from BGR to GRAY
  //GaussianBlur(frame_gray, frame_gray_with_Gblur, Size(3, 3), 0); // Gaussian blur on gray frame, 1st arg: input frame; 2nd arg: output frame; 3rd arg: defines the blur radius; 4th arg: Gaussian kernel standard deviation in X direction, when this is 0 it is computed from the 3rd arg. Gaussaian blur is used since an example used this.

  Mat mask;
  inRange(image_hsv, Scalar(hsv_h_low,hsv_s_low,hsv_v_low), Scalar(hsv_h_upper, hsv_s_upper, hsv_v_upper), mask); // Find the areas which contain the color. 1st arg: inpur frame; 2nd arg: the lower HSV limits; 3rd arg: the upper HSV limits; 4th arg: the output mask.
  for (size_t i = 0; i < dilate_color_iterations; i++)
    dilate(mask, mask, Mat(), Point(-1,-1)); // Enhance the red areas in the image
  image_gray.copyTo(image_masked, mask);

  // Detect blobs.
  std::vector<KeyPoint> keypoints;
  detector->detect( mask, keypoints);

  Mat im_with_keypoints;
  drawKeypoints( image, keypoints, im_with_keypoints, Scalar(0,127,127), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

  std::cout << "Keypoints: " << keypoints.size() << std::endl;
  if (keypoints.size()) {
    for (size_t i = 0; i < keypoints.size(); i++) {
      putText(im_with_keypoints,  patch::to_string(i), keypoints[i].pt,
      FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0,127,127), 1, CV_AA);
    }
  }
  imshow("Display Image", image);
  imshow("Display Masked", image_masked);
  imshow("Display Keypoints", im_with_keypoints);
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
  params.filterByArea = false;
  //params.minArea = 5;
  //params.maxArea = 600;
  // Filter by Circularity - we do not do this parameter due to motion blur
  params.filterByCircularity = false;
  //params.minCircularity = 0.5;
  // Filter by Convexity - we do not use this parameter to ensure detection
  params.filterByConvexity = false;
  //params.minConvexity = 0.87;
  // Filter by Inertia - not used but it means "the inertial resistance of the blob to rotation about its principal axes"
  params.filterByInertia = false;
  //params.minInertiaRatio = 0.01;
  //params.maxInertiaRatio = 0.5;
  detector = SimpleBlobDetector::create(params); // Set up detector with params

  namedWindow("Display Image", CV_WINDOW_AUTOSIZE );
  namedWindow("Display Masked", CV_WINDOW_AUTOSIZE );
  namedWindow("Display Keypoints", CV_WINDOW_AUTOSIZE );
  namedWindow("Trackbars", CV_WINDOW_AUTOSIZE );

  //createTrackbar( "Rho:", "Hough line detection", &hough_rho, hough_rho_max, on_trackbar );
  createTrackbar("HUE LOW", "Trackbars", &hsv_h_low, 255, on_trackbar);
  createTrackbar("HUE UPPER", "Trackbars", &hsv_h_upper, 255, on_trackbar);
  createTrackbar("SATURATION LOW", "Trackbars", &hsv_s_low, 255, on_trackbar);
  createTrackbar("SATURATION UPPER", "Trackbars", &hsv_s_upper, 255, on_trackbar);
  createTrackbar("VIBRANCE LOW", "Trackbars", &hsv_v_low, 255, on_trackbar);
  createTrackbar("VIBRANCE UPPER", "Trackbars", &hsv_v_upper, 255, on_trackbar);

  waitKey(0);
  return 0;
}
