#include <iostream>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <vector>
using namespace std;

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "opencv2/xfeatures2d.hpp"

using namespace cv;
#define EPSILON 1E-5

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

//! Return the maximum of the provided numbers
double maximum(double number1, double number2, double number3);
bool almostEqual(double number1, double number2);
bool lineIntersection(const cv::Point2f &a1, const cv::Point2f &b1, const cv::Point2f &a2, const cv::Point2f &b2, cv::Point2f &intersection);
#endif
