#include "SIFTDetector.hpp"


SIFTDetector::SIFTDetector(Mat &img_marker){
  std::cout << "Enter constructor" << std::endl;
  this->img_marker = img_marker;
  f2d = xfeatures2d::SIFT::create();

  //-- Step 1: Detect the keypoints:
  f2d->detect( img_marker, keypoints_marker );
  std::cout << "exit detect" << std::endl;

  //-- Step 2: Calculate descriptors (feature vectors)
  f2d->compute( img_marker, keypoints_marker, descriptors_marker );
  std::cout << "exit compute" << std::endl;
}

std::vector<Point2f> SIFTDetector::GetCornersOfMarkerInScene(Mat &img_scene){
  //-- Step 1: Detect the keypoints:
  std::vector<KeyPoint> keypoints_scene;
  f2d->detect( img_scene, keypoints_scene );

  //-- Step 2: Calculate descriptors (feature vectors)
  Mat descriptors_scene;
  f2d->compute( img_scene, keypoints_scene, descriptors_scene );

  //-- Step 3: Matching descriptor vectors using some matcher
  bool useBruteForceMatcher = false;
  std::vector< DMatch > matches;
  if (useBruteForceMatcher) {
   BFMatcher matcher;
   matcher.match( descriptors_marker, descriptors_scene, matches );
  } else {
   FlannBasedMatcher mathcer;
   mathcer.match( descriptors_marker, descriptors_scene, matches );
  }

  double max_dist = 0; double min_dist = 100;

  //-- Step 4: Filter out best keypoints
  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_marker.rows; i++ ){
    double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }
/*
  printf("-- Max dist : %f \n", max_dist );
  printf("-- Min dist : %f \n", min_dist );
*/

  //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
  //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
  //-- small)
  std::vector< DMatch > good_matches;
  for( int i = 0; i < descriptors_marker.rows; i++ ){
    if( matches[i].distance <= max(3*min_dist, 0.025) ){
       good_matches.push_back( matches[i]);
    }
  }

  std::cout << "good matches: " << good_matches.size() << std::endl;
  if(good_matches.size() == 0){
    std::cout << "returning, good_matches: 0" << std::endl;
    std::vector<Point2f> out;
    return out;
  }
  Mat img_matches;
  drawMatches( img_marker, keypoints_marker, img_scene, keypoints_scene,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  //-- Localize the marker
  std::vector<cv::Point2f> obj;
  std::vector<cv::Point2f> scene;

  for( int i = 0; i < good_matches.size(); i++ ){
    //-- Get the keypoints from the good matches
    obj.push_back( keypoints_marker[ good_matches[i].queryIdx ].pt );
    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
  }

  Mat H = findHomography( obj, scene, CV_RANSAC );

  //-- Get the corners from the marker ( the marker to be "detected" )
  std::vector<Point2f> obj_corners(4);
  obj_corners[0] = cvPoint( 0, 0);
  obj_corners[1] = cvPoint( img_marker.cols, 0 );
  obj_corners[2] = cvPoint( img_marker.cols, img_marker.rows );
  obj_corners[3] = cvPoint( 0, img_marker.rows );
  std::vector<Point2f> scene_corners(4);

  perspectiveTransform( obj_corners, scene_corners, H);

  //-- Draw lines between the corners (the mapped marker in the scene - image_2 )
  //line( img_scene, scene_corners[0], scene_corners[1], Scalar(0, 255, 0), 4 );
//  line( img_matches, scene_corners[1] + Point2f( img_scene.cols, 0), scene_corners[2] + Point2f( img_scene.cols, 0), Scalar( 0, 255, 0), 4 );
//  line( img_matches, scene_corners[2] + Point2f( img_scene.cols, 0), scene_corners[3] + Point2f( img_scene.cols, 0), Scalar( 0, 255, 0), 4 );
//  line( img_matches, scene_corners[3] + Point2f( img_scene.cols, 0), scene_corners[0] + Point2f( img_scene.cols, 0), Scalar( 0, 255, 0), 4 );


  return scene_corners;

/*
  Point2f center_point;
  std::cout << " found intersecting: " << lineIntersection(scene_corners[0], scene_corners[2], scene_corners[1], scene_corners[3], center_point) << std::endl;
  center_point += Point2f( img_scene.cols, 0);
  std::cout << "point center: " << center_point << std::endl;
  circle(img_matches, center_point, 30, Scalar( 0, 255, 0), 10);
  namedWindow( "matches", CV_WINDOW_NORMAL );
*/
}
