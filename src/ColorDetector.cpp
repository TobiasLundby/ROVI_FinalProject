#include "ColorDetector.hpp"

ColorDetector::ColorDetector(){
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
}

std::vector<Point2f> ColorDetector::FindMarker(Mat &image) {
  // Create / convert images to color spaces for processing
  Mat image_hsv, image_gray;
  cvtColor(image, image_hsv,  COLOR_BGR2HSV); // Convert from BGR to HSV
  cvtColor(image, image_gray, COLOR_BGR2GRAY); //Convert the captured frame from BGR to GRAY

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
      std::cout << "Mean (" << i << "): " << mm[0] << std::endl;
      if ( (mm[0] > 60 and mm[0] < 95) ) {
        elements_first_search.push_back(i);
        elements_first_search_means.push_back(mm);
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
      int threashold_s = 40;
      int threashold_v = 40;
      int threashold_size = 25;

      std::cout << "avg h: " << average_means_first_search_h << std::endl;
      std::cout << "avg s: " << average_means_first_search_s << std::endl;
      std::cout << "avg v: " << average_means_first_search_v << std::endl;
      std::cout << "avg size: " << average_means_first_search_size << std::endl;

      for (size_t i = 0; i < elements_first_search.size(); i++) {
        std::cout << "avg (" << i << ")h: " << elements_first_search_means.at(i)[0] << std::endl;
        std::cout << "avg (" << i << ")s: " << elements_first_search_means.at(i)[1] << std::endl;
        std::cout << "avg (" << i << ")v: " << elements_first_search_means.at(i)[2] << std::endl;
        std::cout << "avg (" << i << ")size: " << keypoints_MB[elements_first_search[i]].size << std::endl << std::endl;
        if ( (elements_first_search_means.at(i)[2] > average_means_first_search_v-threashold_v and elements_first_search_means.at(i)[2] < average_means_first_search_v+threashold_v)
          and (elements_first_search_means.at(i)[1] > average_means_first_search_s-threashold_s and elements_first_search_means.at(i)[1] < average_means_first_search_s+threashold_s)
          and (elements_first_search_means.at(i)[0] > average_means_first_search_h-threashold_h and elements_first_search_means.at(i)[0] < average_means_first_search_h+threashold_h)
          and (keypoints_MB[elements_first_search[i]].size > average_means_first_search_size-average_means_first_search_size*(threashold_size/100.0) and keypoints_MB[elements_first_search[i]].size < average_means_first_search_size+average_means_first_search_size*(threashold_size/100.0)) ) {
          // Save filtered keypoints for further processing
          keypoints_MB_final.push_back( keypoints_MB.at( elements_first_search.at(i) ));
          keypoints_combined.push_back( keypoints_MB.at( elements_first_search.at(i) ));
        }
      }
    }
  } else {
    std::cout << "Not enough detections, line 103 ColorDetector.cpp" << std::endl;
  }

  // Calculate center of the blue markers
  double centerX_MB = 0;
  double centerY_MB = 0;
  if (keypoints_MB_final.size()) {
    for (size_t i = 0; i < keypoints_MB_final.size(); i++) {
      centerX_MB += keypoints_MB_final[i].pt.x;
      centerY_MB += keypoints_MB_final[i].pt.y;
    }
    centerX_MB /= keypoints_MB_final.size();
    centerY_MB /= keypoints_MB_final.size();
  } else {
    std::cout << "Not enough detections, line 115 ColorDetector.cpp" << std::endl;
  }
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


  // Detect blobs.
  std::vector<KeyPoint> keypoints_MR;
  detector->detect( mask_MR, keypoints_MR);

  // Filter keypoints
  double min_dist_MR;
  int min_dist_id;
  if (keypoints_MR.size()) {
    min_dist_MR = sqrt(pow(keypoints_MR.at(0).pt.x-center_MB.x,2)+pow(keypoints_MR.at(0).pt.y-center_MB.y,2));
    min_dist_id = 0;

    for (size_t i = 1; i < keypoints_MR.size(); i++) {
      double tmp_min_dist_MR = sqrt(pow(keypoints_MR.at(i).pt.x-center_MB.x,2)+pow(keypoints_MR.at(i).pt.y-center_MB.y,2));
      if (tmp_min_dist_MR < min_dist_MR) {
        min_dist_id = i;
        min_dist_MR = tmp_min_dist_MR;
      }
    }

    // Save keypoints
    keypoints_MR_final.push_back( keypoints_MR.at(min_dist_id) );
    keypoints_combined.push_back( keypoints_MR.at(min_dist_id) );
  } else {
    std::cout << "Not enough detections, line 165 ColorDetector.cpp" << std::endl;
  }




  // Sort markers in the right order
  int MB1_id = 0;
  int MB2_id = 0;
  int MB3_id = 0;

  if (keypoints_MR_final.size() == 1 and keypoints_MB_final.size() == 3) {
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
  } else {
    std::cout << "Not enough detections, line 303 ColorDetector.cpp" << std::endl;
  }

  std::vector< Point2f > output_points; // Elements: 1=MB1, 2=MB2, 3=MB3, 4=MB4

  if (keypoints_MB_final.size() == 3 and keypoints_MR_final.size() == 1) {
    Point2f tmp_point;
    tmp_point = keypoints_MB_final.at(MB1_id).pt;
    output_points.push_back(tmp_point);
    tmp_point = keypoints_MB_final.at(MB2_id).pt;
    output_points.push_back(tmp_point);
    tmp_point = keypoints_MB_final.at(MB3_id).pt;
    output_points.push_back(tmp_point);
    tmp_point = keypoints_MR_final.at(0).pt;
    output_points.push_back(tmp_point);
  } else {
    std::cout << "Not enough detections, line 325 ColorDetector.cpp" << std::endl;
  }

  return output_points;
}
