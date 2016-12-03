/*
 *  image1.cpp
 *  ROVI - Final Project - Visual Servoing
 *
 */

// Includes
#include <stdio.h>
#include <opencv2/opencv.hpp>

// Namespaces
using namespace cv;

// Defines
#define HUE_ORANGE     11                      /* 0-22 */
#define HUE_YELLOW     30                      /* 22-38 */
#define HUE_GREEN      60                      /* 38-75 */
#define HUE_BLUE       100                      /* 75-130 */
#define HUE_VIOLET     145                      /* 130-160 */
#define HUE_RED        160                      /* 160-179 */

// Global variables - only due to trackbar
Mat image;

void on_trackbar( int, void* )
{
  int hsv_h_base        = HUE_BLUE;
  int hsv_h_sensitivity = 15; // 5 for big green led and 24 for most other
  int hsv_h_low         = hsv_h_base - hsv_h_sensitivity; //hsv_h_base - hsv_h_sensitivity;
  int hsv_h_upper       = hsv_h_base + hsv_h_sensitivity;//hsv_h_base + hsv_h_sensitivity;
  int hsv_s_low         = 50; //100;
  int hsv_s_upper       = 255;
  int hsv_v_low         = 50; //100;
  int hsv_v_upper       = 255;

  Mat image_hsv, image_red;
  cvtColor(image, image_hsv, COLOR_BGR2HSV); // Convert from BGR to HSV
  Mat mask;
  inRange(image_hsv, Scalar(hsv_h_low,hsv_s_low,hsv_v_low), Scalar(hsv_h_upper, hsv_s_upper, hsv_v_upper), mask); // Find the areas which contain the color. 1st arg: inpur frame; 2nd arg: the lower HSV limits; 3rd arg: the upper HSV limits; 4th arg: the output mask.

  image.copyTo(image_red, mask);

  imshow("Display Image", image);
  imshow("Display masked", image_red);
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

  namedWindow("Display Image", CV_WINDOW_AUTOSIZE );
  namedWindow("Display Masked", CV_WINDOW_AUTOSIZE );
  namedWindow("Trackbars", CV_WINDOW_AUTOSIZE );

  createTrackbar( "Rho:", "Hough line detection", &hough_rho, hough_rho_max, on_trackbar );
  createTrackbar("HUE GREEN LOW", "Trackbars", &hsv_h_low, 255);
  createTrackbar("HUE GREEN UPPER", "Trackbars", &hsv_h_upper, 255);
  createTrackbar("SATURATION LOW", "Trackbars", &hsv_s_low, 255);
  createTrackbar("SATURATION UPPER", "Trackbars", &hsv_s_upper, 255);
  createTrackbar("VIBRANCE LOW", "Trackbars", &hsv_v_low, 255);
  createTrackbar("VIBRANCE UPPER", "Trackbars", &hsv_v_upper, 255);

  waitKey(0);
  return 0;
}
