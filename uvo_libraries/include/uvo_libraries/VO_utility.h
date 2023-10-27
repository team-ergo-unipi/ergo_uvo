#ifndef VO_UTILITY_H
#define VO_UTILITY_H


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////// LIBRARIES //////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "ros/ros.h"
#include "uvo_libraries/math_utility.h"
#include <opencv2/xfeatures2d/nonfree.hpp>
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Bool.h"
using namespace cv::xfeatures2d;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////// VARIABLES //////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


Mat R_left, t_left, R_right, t_right;

std::vector<double> data;
int rows, cols; 

// MONO CAMERA PARAMETERS
double fx, fy, ccx, ccy; // INTRINSICS
double k1, k2, p1, p2;   // DISTORTION

// STEREO CAMERA PARAMETERS
double fx_left, fy_left, ccx_left, ccy_left, fx_right, fy_right, ccx_right, ccy_right;  // INTRINSICS
double k1_left, k2_left, p1_left, p2_left, k1_right, k2_right, p1_right, p2_right;      // DISTORTION   				

// NODE PARAMETERS
int NODE_FREQ;

// PREPROCESSING PARAMETERS
int DESIRED_WIDTH;
bool CLAHE_CORRECTION;
int CLIP_LIMIT;

// VO PARAMETERS
int DISTANCE;

// FEATURES DETECTION METHOD
string FEATURE_DETECTOR;

int    ESSENTIAL_OUTLIER_METHOD;
double ESSENTIAL_MAX_ITERS;
double ESSENTIAL_CONFIDENCE;
double ESSENTIAL_THRESHOLD;

int    HOMOGRAPHY_OUTLIER_METHOD;
double HOMOGRAPHY_MAX_ITERS;
double HOMOGRAPHY_CONFIDENCE;
double HOMOGRAPHY_THRESHOLD;
double HOMOGRAPHY_DISTANCE;

double VPF_THRESHOLD;
double REPROJECTION_TOLERANCE;
double LOWE_RATIO_THRESHOLD;

int MIN_NUM_FEATURES;
int MIN_NUM_3DPOINTS;
int MIN_NUM_INLIERS;

// PNP RANSAC PARAMETERS
int ITERATIONS_COUNT;
double REPROJECTION_ERROR_THRESHOLD;
double CONFIDENCE;
bool USE_EXTRINSIC_GUESS;
int PNP_METHOD_FLAG;
    
// VISUALIZATION PARAMETERS
int FPS;
bool SHOW_MATCHES;

// SURF PARAMETERS
int  SURF_MIN_HESSIAN;
int  SURF_OCTAVES_NUMBER;
int  SURF_OCTAVES_LAYERS;
bool SURF_EXTENDED;
bool SURF_UPRIGHT;

bool use_essential = true;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////// FUNCTIONS //////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Mat compute_projection_matrix(const Mat& R, const Mat& t, const Mat& cameraIntrinsic);
double compute_scale_factor(float distance, const Mat& world_points);
Mat convert_3Dpoints_camera(const Mat& points_to_convert, const Mat& R_to_from, const Mat& t_to_from);
Mat convert_from_homogeneous_coords(const Mat& points4d);
void detect_features(Mat img, vector<KeyPoint> &keypoints, Mat &descriptors);
void estimate_relative_pose(vector<Point2f> keypoints1_conv, vector<Point2f> keypoints2_conv, Mat cameraMatrix, Mat &R_currCam_prevCam, Mat &t_currCam_prevCam, vector<Point2f> &inliers1, vector<Point2f> &inliers2, vector<DMatch> &inlier_matches, bool &success);
void extract_3Dpoints(vector<Point2f> keypoints1_conv, vector<Point2f> keypoints2_conv, Mat R1, Mat t1, Mat R2, Mat t2, Mat cameraMatrix1, Mat cameraMatrix2, Mat points4D, Mat& very_good_cam1_points, Mat& very_good_indexes);
void extract_3Dpoints_and_reprojection(vector<Point2f> keypoints1_conv, vector<Point2f> keypoints2_conv, Mat R1, Mat t1, Mat R2, Mat t2, Mat cameraMatrix1, Mat cameraMatrix2, Mat points4D, Mat& very_good_cam1_points, Mat& very_good_indexes, vector<double>& reprojection_errors_vector);
void extract_inliers(const vector<Point2f>& keypoints1_conv, const vector<Point2f>& keypoints2_conv, const Mat& mask, vector<Point2f>& inliers1, vector<Point2f>& inliers2, vector<DMatch>& inlier_matches);
Mat get_image(const Mat& current_img, const Mat& cameraMatrix, const Mat& distortionCoeff, const Mat& newCamMatrix);
void get_mono_camera_parameters(ros::NodeHandle node_obj, string CAMERA_NAME);
void get_stereo_camera_parameters(ros::NodeHandle node_obj, string CAMERA_NAME);
void get_VO_parameters(ros::NodeHandle node_obj);
void match_features(vector<KeyPoint> keypoints1, vector<KeyPoint> keypoints2, Mat descriptors1, Mat descriptors2, vector<DMatch> &matches);
void match_features(vector<KeyPoint> keypoints1, vector<KeyPoint> keypoints2, Mat descriptors1, Mat descriptors2, vector<DMatch> &matches, vector<Point2f> &keypoints1_conv, vector<Point2f> &keypoints2_conv);
int recover_pose_homography(Mat H, vector<Point2f> inliers1, vector<Point2f> inliers2, Mat cameraMatrix, Mat& R, Mat& t);
void resize_camera_matrix(Mat original_image, Mat& cameraMatrix, Mat distortionCoeff, Mat& newCamMatrix);
vector<double> reproject_errors(const Mat& world_points, const Mat& R, const Mat& t, const Mat& cameraMatrix, const vector<Point2f>& img_points);
void select_desired_descriptors(const Mat& descriptors, Mat& descriptors_desired, const Mat& indexes);
void select_desired_keypoints(const vector<KeyPoint>& keypoints, vector<KeyPoint>& keypoints_desired, const Mat& indexes);
bool select_estimation_method(const vector<Point2f>& keypoints1_conv, const vector<Point2f>& keypoints2_conv);
Mat show_matches(vector<KeyPoint> kpoints1, vector<KeyPoint> kpoints2, vector<DMatch> matches, Mat img1, Mat img2);

#endif