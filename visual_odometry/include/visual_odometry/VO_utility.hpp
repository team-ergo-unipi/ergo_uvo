// ######################################################################################################################################################## //
// ######################################################################################################################################################## //
//                                                                                                                                                          //
//                                  This file contains all the functions exploited within the visual odometry node                                          //       
//                                                                                                                                                          //
// -------------------------------------------------------------------------------------------------------------------------------------------------------- //
//                            Author:  Francesco Ruscio       Email: francesco.ruscio@phd.unipi.it     Date:  06/06/2022                                    //
//                            Author:  Simone Tani            Email: simone.tani@phd.unipi.it          Date:  06/06/2022                                    //
//                                                                                                                                                          //
// ######################################################################################################################################################## //
// ######################################################################################################################################################## //

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////// LIBRARIES //////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "ros/ros.h"
#include <visual_odometry/math_utility.hpp>        
#include <opencv2/xfeatures2d/nonfree.hpp>
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int64.h"
using namespace cv::xfeatures2d;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////// VARIABLES //////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Mat body_camera_matrix;
std::vector<double> data;
int rows, cols; 

// MONO CAMERA PARAMETERS
double fx, fy, ccx, ccy; // INTRINSICS
double k1, k2, p1, p2;   // DISTORTION

// STEREO CAMERA PARAMETERS
double fx_left, fy_left, ccx_left, ccy_left, fx_right, fy_right, ccx_right, ccy_right;  // INTRINSICS
double k1_left, k2_left, p1_left, p2_left, k1_right, k2_right, p1_right, p2_right;      // DISTORTION
double baseline;

// NODE PARAMETERS
int NODE_FREQ;

// PREPROCESSING PARAMETERS
int DESIRED_WIDTH;
bool CLAHE_CORRECTION;
int CLIP_LIMIT;

// VO PARAMETERS
int DISTANCE;
int SKIPPED_IMGS;
bool FEATURES_TRACKING;
double ESSENTIAL_MAX_ITERS;
double ESSENTIAL_CONFIDENCE;
double ESSENTIAL_THRESHOLD;
double HOMOGRAPHY_MAX_ITERS;
double HOMOGRAPHY_CONFIDENCE;
double HOMOGRAPHY_THRESHOLD;
double HOMOGRAPHY_DISTANCE;
double VPF_THRESHOLD;
double REPROJECTION_TOLERANCE;
int MIN_NUM_FEATURES;
int MIN_NUM_TRACKED_FEATURES;
int MIN_NUM_3DPOINTS;
int MIN_NUM_INLIERS;

// PNP RANSAC PARAMETERS
int ITERATIONS_COUNT;
double REPROJECTION_ERROR_THRESHOLD;
double CONFIDENCE;
bool USE_EXTRINSIC_GUESS;
    
// VISUALIZATION PARAMETERS
int FPS;
bool SHOW_MATCHES;

// SURF PARAMETERS
int SURF_MIN_HESSIAN;
int SURF_OCTAVES_NUMBER;
int SURF_OCTAVES_LAYERS;
bool SURF_EXTENDED;
bool SURF_UPRIGHT;
bool LOWE_RATIO;
double SURF_RATIO_THRESHOLD;

// RELATIVE POSE ESTIMATION METHOD
enum rel_pose_method {ESSENTIAL, HOMOGRAPHY};
rel_pose_method GLOBAL_ESTIMATE_METHOD = ESSENTIAL;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////// FUNCTIONS //////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool check_if_moving(vector<Point2f> keypoints1_conv, vector<Point2f> keypoints2_conv);
vector<Mat> compute_absolute_pose(Mat R_world_prevCam, Mat prevCam_world_position, Mat R_currCam_prevCam, Mat t_currCam_prevCam, double SF, Mat world_points);
geometry_msgs::Vector3 compute_angular_velocity(geometry_msgs::Vector3 rpy, Mat R, double deltaT, Mat Rbc);
Mat compute_projection_matrix(Mat R, Mat t, Mat cameraIntrinsic);
double compute_scale_factor(float distance, Mat worldPoints);
Mat convert_3Dpoints_camera(Mat points_to_convert, Mat R_to_from, Mat t_to_from);
Mat convert_from_homogeneous_coords(Mat points4d);
Mat desired_resize(Mat img);
void detect_features(Mat img, vector<KeyPoint> &keypoints, Mat &descriptors);
void estimate_relative_pose(vector<Point2f> keypoints1_conv, vector<Point2f> keypoints2_conv, Mat cameraMatrix, Mat &R_currCam_prevCam, Mat &t_currCam_prevCam, vector<Point2f> &inliers1, vector<Point2f> &inliers2, vector<DMatch> &inlier_matches, bool &success);
geometry_msgs::Twist estimate_twist(ros::Duration deltaT, Mat R_currCam_prevCam, Mat t_currCam_prevCam, double SF, geometry_msgs::Vector3 estimate_rpy, Mat R_body_currCam);
Mat extract_3Dpoints(vector<Point2f> keypoints1_conv, vector<Point2f> keypoints2_conv, Mat R1, Mat t1, Mat R2, Mat t2, Mat cameraMatrix1, Mat cameraMatrix2, Mat points4d);
void extract_inliers(vector<Point2f> keypoints1_conv, vector<Point2f> keypoints2_conv, Mat mask, vector<Point2f> &inliers1, vector<Point2f> &inliers2, vector<DMatch> &inlier_matches);
Mat get_image(Mat current_img, Mat cameraMatrix, Mat distortionCoeff, Mat newCamMatrix);
void get_mono_camera_parameters(ros::NodeHandle node_obj, string CAMERA_NAME);
void get_stereo_camera_parameters(ros::NodeHandle node_obj, string CAMERA_NAME);
void get_VO_parameters(ros::NodeHandle node_obj);
void match_features(vector<KeyPoint> keypoints1, vector<KeyPoint> keypoints2, Mat descriptors1, Mat descriptors2, vector<DMatch> &matches, vector<Point2f> &keypoints1_conv, vector<Point2f> &keypoints2_conv);
void match_features(vector<KeyPoint> keypoints1, vector<KeyPoint> keypoints2, Mat descriptors1, Mat descriptors2, vector<DMatch> &matches);
int recover_pose_homography(Mat H, vector<Point2f> inliers1, vector<Point2f> inliers2, Mat cameraMatrix, Mat& R, Mat& t);
void resize_camera_matrix(Mat original_image, Mat& cameraMatrix, Mat distortionCoeff, Mat& newCamMatrix);
vector<double> reproject_errors(Mat world_points, Mat R, Mat t, Mat cameraMatrix, vector<Point2f> img_points);
void select_desired_descriptors(const Mat &descriptors, Mat &descriptors_desired, const Mat &indexes);
void select_desired_keypoints(vector<KeyPoint> keypoints, vector<KeyPoint> &keypoints_desired, Mat indexes);
Mat show_matches(vector<KeyPoint> kpoints1, vector<KeyPoint> kpoints2, vector<DMatch> matches, Mat img1, Mat img2);
void track_features(Mat img_1, Mat img_2, vector<KeyPoint> keypoints1, vector<KeyPoint> &keypoints2, Mat descriptors1, Mat &descriptors2, vector<DMatch> &matches);
void track_features(Mat img_1, Mat img_2, vector<KeyPoint> &keypoints1, vector<KeyPoint> &keypoints2, Mat descriptors1, Mat &descriptors2, vector<DMatch> &matches, vector<Point2f> &keypoints1_conv, vector<Point2f> &keypoints2_conv);


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


bool check_if_moving(vector<Point2f> keypoints1_conv, vector<Point2f> keypoints2_conv)
{
    int n_kP = keypoints1_conv.size();
    vector<double> pixelTrasl(n_kP);

    for(int i = 0; i < n_kP; i++)
    {
        pixelTrasl[i] = sqrt(powf(keypoints1_conv[i].x - keypoints2_conv[i].x, 2.0) + powf(keypoints1_conv[i].y - keypoints2_conv[i].y, 2.0));
    }

    if(compute_median(pixelTrasl) < DISTANCE){return false;}
    else{return true;}
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


vector<Mat> compute_absolute_pose(Mat R_world_prevCam, Mat prevCam_world_position, Mat R_currCam_prevCam, Mat t_currCam_prevCam, double SF, Mat world_points)
{
    vector<Mat> absPose; 

    Mat R_world_currCam = R_world_prevCam * R_currCam_prevCam.t();
    Mat currCam_world_position = transform_coordinates(- SF * R_currCam_prevCam.t() * t_currCam_prevCam, R_world_prevCam, prevCam_world_position);

    if(world_points.cols >= MIN_NUM_3DPOINTS)
    {

        Mat world_pointsW = Mat::zeros(world_points.rows, world_points.cols, CV_64F);
        for(int i = 0; i < world_points.cols; i++)
        {
            Mat local_WP = transform_coordinates(SF * world_points.col(i), R_world_currCam, currCam_world_position);
            for(int j = 0; j < world_points.rows; j++){world_pointsW.at<double>(j, i) = local_WP.at<double>(j);}
        }

        absPose = {currCam_world_position, R_world_currCam, world_pointsW};

    }

    else{absPose = {currCam_world_position, R_world_currCam};}
    
    return absPose;
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


geometry_msgs::Vector3 compute_angular_velocity(geometry_msgs::Vector3 rpy, Mat R, double deltaT, Mat Rbc)
{
    //deltaRPY
    geometry_msgs::Vector3 rpy_dot = from_rotation_matrix_to_euler_angles(R);
    //rpy_dot
    rpy_dot.x /= deltaT;
    rpy_dot.y /= deltaT;
    rpy_dot.z /= deltaT;

    Mat J = compute_jacobian(rpy);
    Mat M = Rbc * J.inv();

    //Angular Velocity
    geometry_msgs::Vector3 w;

    //w = Rbc*J2(rpy)^-1*rpy_dot
    w.x = M.at<double>(0,0)*rpy_dot.x + M.at<double>(0,1)*rpy_dot.y + M.at<double>(0,2)*rpy_dot.z;
    w.y = M.at<double>(1,0)*rpy_dot.x + M.at<double>(1,1)*rpy_dot.y + M.at<double>(1,2)*rpy_dot.z;
    w.z = M.at<double>(2,0)*rpy_dot.x + M.at<double>(2,1)*rpy_dot.y + M.at<double>(2,2)*rpy_dot.z;

    return w;
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


Mat compute_projection_matrix(Mat R, Mat t, Mat cameraIntrinsic)
{
    Mat R_t;
    hconcat(R, t, R_t);

    return cameraIntrinsic * R_t; 
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


double compute_scale_factor(float distance, Mat world_points)
{
    vector<double> z_vector;
    world_points.row(2).copyTo(z_vector);

    double Zmedian;
    Zmedian = compute_median(z_vector);

    return distance / Zmedian;  
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


Mat convert_3Dpoints_camera(Mat points_to_convert, Mat R_to_from, Mat t_to_from)
{
    Mat converted_points;
    int i = 0;
    for(i; i < points_to_convert.rows; i++)
    {
        Mat points_to_convert_row = (points_to_convert.row(i));
        Mat converted_points_row = transform_coordinates(points_to_convert_row.t(), R_to_from, t_to_from).t();    

        if (converted_points_row.at<double>(2) > 0)
        {
            converted_points.push_back(points_to_convert_row);
        }                     
    }

    if(!converted_points.empty()){return converted_points.t();}
    else{return converted_points;}
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


Mat convert_from_homogeneous_coords(Mat points4d)
{
    CV_Assert((points4d.rows == 4) && (points4d.type() == CV_32F));

    int cols = points4d.cols;
    Mat points3d(3, cols, points4d.type());
    for(int i = 0; i < cols; i++)
    {
        points3d.col(i) = (points4d.col(i).rowRange(0, 3))/(points4d.col(i).at<float>(3));
    }

    return points3d;    
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


Mat desired_resize(Mat img)
{
    int original_width  = img.cols;
    int original_height = img.rows;
    double ratio        = (double)original_width / (double)DESIRED_WIDTH;
    int desired_height  = (int)(original_height / ratio);

    if( (original_width != DESIRED_WIDTH) || (original_height != desired_height) )
    {
        Mat resized_img;
        resize(img, resized_img, Size(DESIRED_WIDTH, desired_height), 0, 0, INTER_AREA);

        return resized_img;
    }
    else {return img;}
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


void detect_features(Mat img, vector<KeyPoint> &keypoints, Mat &descriptors)
{
    Ptr<SURF> detector = SURF::create(SURF_MIN_HESSIAN, SURF_OCTAVES_NUMBER, SURF_OCTAVES_LAYERS, SURF_EXTENDED, SURF_UPRIGHT);
    detector->detectAndCompute(img, noArray(), keypoints, descriptors);
    ROS_INFO("FEATURES EXTRACTED - CURR. IMAGE: %lu", keypoints.size());
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


void estimate_relative_pose(vector<Point2f> keypoints1_conv, vector<Point2f> keypoints2_conv, Mat cameraMatrix, Mat &R_currCam_prevCam, Mat &t_currCam_prevCam, vector<Point2f> &inliers1, vector<Point2f> &inliers2, vector<DMatch> &inlier_matches, bool &success)
{
    // METHOD USED WITHIN THE FUNCTION
    rel_pose_method LOCAL_ESTIMATE_METHOD = GLOBAL_ESTIMATE_METHOD;

    bool estimate_completed = false;
    bool switch_method      = false;
    success                 = false;

    Mat ESSENTIAL_MASK, HOMOGRAPHY_MASK;
    int numInliers  = 0;
    int validInlier = 0;
    float validPointFraction = 0.0;

    while(!estimate_completed)
    {

        switch (LOCAL_ESTIMATE_METHOD)
        {

            case ESSENTIAL:
            {

                Mat E = findEssentialMat(keypoints1_conv, keypoints2_conv, cameraMatrix, LMEDS, ESSENTIAL_CONFIDENCE, ESSENTIAL_THRESHOLD, ESSENTIAL_MAX_ITERS, ESSENTIAL_MASK);

                numInliers = countNonZero(ESSENTIAL_MASK);
                ROS_INFO("INLIERS AFTER ESSENTIAL MATRIX COMPUTATION: %d", numInliers);

                if(numInliers < MIN_NUM_INLIERS)
                {
                    ROS_WARN("NUMBER OF INLIERS AFTER ESSENTIAL MATRIX COMPUTATION IS TOO LOW");
                    success = false;         
                }
                else
                {

                    extract_inliers(keypoints1_conv, keypoints2_conv, ESSENTIAL_MASK, inliers1, inliers2, inlier_matches);

                    ROS_INFO("MATCHES AFTER ESSENTIAL MATRIX COMPUTATION: %lu", inliers1.size());

                    validInlier = recoverPose(E, inliers1, inliers2, cameraMatrix, R_currCam_prevCam, t_currCam_prevCam);
                    ROS_INFO("MATCHES AFTER RECOVERING POSE: %d", validInlier);

                    validPointFraction = (float) validInlier / inliers1.size();
                    ROS_INFO("VALID POINT FRACTION: %f", validPointFraction);

                    if(validPointFraction > VPF_THRESHOLD)
                    {
                        success = true;  
                    }
                    else
                    {
                        ROS_WARN("VALID POINT FRACTION IS TOO LOW");
                        success = false;
                    }
                }


                if (!success)
                {
                    if (switch_method){estimate_completed = true;}
                    else
                    {
                        switch_method = true;
                        ROS_WARN("###### SWITCHING METHOD TO HOMOGRAPHY ######");
                    }
                    LOCAL_ESTIMATE_METHOD  = HOMOGRAPHY;
                    GLOBAL_ESTIMATE_METHOD = HOMOGRAPHY;
                }
                else{estimate_completed = true;}

            break;}
            

        /////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////     
        /////////////////////////////////////////////////////////////////////////////////////////////


            case HOMOGRAPHY:

                Mat H = findHomography(keypoints1_conv, keypoints2_conv, RANSAC, HOMOGRAPHY_THRESHOLD, HOMOGRAPHY_MASK, HOMOGRAPHY_MAX_ITERS, HOMOGRAPHY_CONFIDENCE);

                numInliers = countNonZero(HOMOGRAPHY_MASK);
                ROS_INFO("INLIERS AFTER HOMOGRAPHY COMPUTATION: %d", numInliers);

                if(numInliers < MIN_NUM_INLIERS)
                {
                    ROS_WARN("NUMBER OF INLIERS AFTER HOMOGRAPHY COMPUTATION IS TOO LOW");
                    success = false;      
                }
                else
                {

                    extract_inliers(keypoints1_conv, keypoints2_conv, HOMOGRAPHY_MASK, inliers1, inliers2, inlier_matches);

                    ROS_INFO("MATCHES AFTER HOMOGRAPHY COMPUTATION: %lu", inliers1.size());

                    validInlier = recover_pose_homography(H, inliers1, inliers2, cameraMatrix, R_currCam_prevCam, t_currCam_prevCam);
                    ROS_INFO("MATCHES AFTER RECOVERING POSE: %d", validInlier);

                    validPointFraction = (float) validInlier/inliers1.size();
                    ROS_INFO("VALID POINT FRACTION: %f", validPointFraction);

                    if(validPointFraction > VPF_THRESHOLD)
                    {
                        success = true;
                    }
                    else
                    {
                        ROS_WARN("VALID POINT FRACTION IS TOO LOW");
                        success = false;
                    }
                }   


                if (!success)
                {
                    if (switch_method){estimate_completed = true;}
                    else
                    {
                        ROS_WARN("###### SWITCHING METHOD TO ESSENTIAL ######");
                        switch_method = true;
                    }
                    LOCAL_ESTIMATE_METHOD  = ESSENTIAL;
                    GLOBAL_ESTIMATE_METHOD = ESSENTIAL;
                }
                else{estimate_completed = true;}

            break;

        }

    }

}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


geometry_msgs::Twist estimate_twist(ros::Duration deltaT, Mat R_currCam_prevCam, Mat t_currCam_prevCam, double SF, geometry_msgs::Vector3 estimate_rpy, Mat R_body_currCam)
{
    geometry_msgs::Twist estimate_twist;

    // COMPUTE LINEAR VELOCITY IN BODY FRAME
    Mat estimate_vel = R_body_currCam * (- SF * R_currCam_prevCam.t() * t_currCam_prevCam / deltaT.toSec());
    estimate_twist.linear = from_mat_to_vector_type(estimate_vel);

    // COMPUTE ANGULAR VELOCITY
    estimate_twist.angular = compute_angular_velocity(estimate_rpy, R_currCam_prevCam.t(), deltaT.toSec(), R_body_currCam);

    return estimate_twist;
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


void extract_3Dpoints(vector<Point2f> keypoints1_conv, vector<Point2f> keypoints2_conv, Mat R1, Mat t1, Mat R2, Mat t2, Mat cameraMatrix1, Mat cameraMatrix2, Mat points4D, Mat& very_good_cam1_points, Mat& very_good_indexes)
{    
    Mat points3D, cam1_points, good_cam1_points, good_indexes;
    convertPointsFromHomogeneous(points4D.t(), points3D);
    points3D.convertTo(points3D, CV_64F);
    int row_index = 0;

    for (int i = 0; row_index < points3D.rows; i = i + 3) 
    {
        Mat current_row;
        current_row = (Mat1d(1, 3) << points3D.at<double>(i), points3D.at<double>(i + 1), points3D.at<double>(i + 2)); 
        cam1_points.push_back(current_row);
        row_index++;
    }
    
    if(cam1_points.rows >= MIN_NUM_3DPOINTS)
    {
        // COMPUTE REPROJECTION ERROR FOR BOTH THE TWO FRAMES AND EXTRACT THE MEAN VALUE
        vector<double> reproject_cam1 = reproject_errors(cam1_points, R1, t1, cameraMatrix1, keypoints1_conv);
        vector<double> reproject_cam2 = reproject_errors(cam1_points, R2, t2, cameraMatrix2, keypoints2_conv);  
        vector<double> reproject_mean(reproject_cam2.size());

        for(int i = 0; i < cam1_points.rows; i++)
        {
            Mat cam1_points_row = cam1_points.row(i);
            reproject_mean[i] = (reproject_cam1[i] + reproject_cam2[i]) / 2.0;
            if((reproject_mean[i] < REPROJECTION_TOLERANCE) && (cam1_points_row.at<double>(2) > 0))
            {
                good_indexes.push_back(i);
                good_cam1_points.push_back(cam1_points_row);
            }

        }
    }

    if(good_cam1_points.rows >= MIN_NUM_3DPOINTS)
    {

        vector<double> mean_var = compute_mean_and_variance(good_cam1_points.col(2));

        for(int i = 0; i < good_cam1_points.rows; i++)
        {
            Mat good_cam1_points_row = good_cam1_points.row(i);
            if( (good_cam1_points_row.at<double>(2) <= mean_var[0] + 3.0*sqrt(mean_var[1])) && (good_cam1_points_row.at<double>(2) >= mean_var[0] - 3.0*sqrt(mean_var[1])) )
            {
                very_good_indexes.push_back(good_indexes.at<int>(i));
                very_good_cam1_points.push_back(good_cam1_points_row);
            }
        }
    }
}
 

// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


void extract_inliers(vector<Point2f> keypoints1_conv, vector<Point2f> keypoints2_conv, Mat mask, vector<Point2f> &inliers1, vector<Point2f> &inliers2, vector<DMatch> &inlier_matches)
{
    int numInliers = countNonZero(mask);
    inlier_matches.resize(numInliers);

    inliers1 = keypoints1_conv;
    inliers1.resize(numInliers);

    inliers2 = keypoints2_conv;
    inliers2.resize(numInliers);

    int inlierCount = 0;
    for(int i = 0; i < mask.rows; i++) {
      if((int) mask.at<uchar>(i,0) != 0){
        inliers1[inlierCount]                 = keypoints1_conv.at(i);
        inliers2[inlierCount]                 = keypoints2_conv.at(i);
        inlier_matches[inlierCount].queryIdx  = inlierCount;
        inlier_matches[inlierCount].trainIdx  = inlierCount;
        inlierCount++;
      }
    }
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


Mat get_image(Mat current_img, Mat cameraMatrix, Mat distortionCoeff, Mat newCamMatrix)
{
    int original_width  = current_img.cols;
    int original_height = current_img.rows;
    double ratio        = (double)original_width / (double)DESIRED_WIDTH;
    int desired_height  = (int)(original_height / ratio);

    Mat gray_img;
    cvtColor(current_img, gray_img, COLOR_RGB2GRAY);

    if( (original_width != DESIRED_WIDTH) || (original_height != desired_height) )
    {
        resize(gray_img, gray_img, Size(DESIRED_WIDTH, desired_height), 0, 0, INTER_AREA);
    }


    Mat undistorted_image;
    undistort(gray_img, undistorted_image, cameraMatrix, distortionCoeff, newCamMatrix);


    if(CLAHE_CORRECTION)
    {
        Ptr<CLAHE> clahe = createCLAHE();
        clahe->setClipLimit(CLIP_LIMIT);

        Mat clahe_img;
        clahe->apply(undistorted_image, undistorted_image);
    }
    
 
    return undistorted_image;
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


void get_mono_camera_parameters(ros::NodeHandle node_obj, string CAMERA_NAME)
{
    //Intrinsic Matrix
    node_obj.getParam("/" + CAMERA_NAME + "/camera_intrinsic/fx", fx);
    node_obj.getParam("/" + CAMERA_NAME + "/camera_intrinsic/fy", fy);
    node_obj.getParam("/" + CAMERA_NAME + "/camera_intrinsic/ccx", ccx);
    node_obj.getParam("/" + CAMERA_NAME + "/camera_intrinsic/ccy", ccy);

    //Distortion Coefficients
    node_obj.getParam("/" + CAMERA_NAME + "/distortion_coefficient/radial/k1", k1);
    node_obj.getParam("/" + CAMERA_NAME + "/distortion_coefficient/radial/k2", k2);
    node_obj.getParam("/" + CAMERA_NAME + "/distortion_coefficient/tangential/p1", p1);
    node_obj.getParam("/" + CAMERA_NAME + "/distortion_coefficient/tangential/p2", p2);

    // READING MATRIX FROM BODY FRAME TO CAMERA FRAME
    node_obj.getParam("/" + CAMERA_NAME + "/body_camera_matrix/data", data);
    node_obj.getParam("/" + CAMERA_NAME + "/body_camera_matrix/rows", rows);
    node_obj.getParam("/" + CAMERA_NAME + "/body_camera_matrix/cols", cols);
    cv::Mat(rows, cols, CV_64F, data.data()).copyTo(body_camera_matrix);
}

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////   

void get_stereo_camera_parameters(ros::NodeHandle node_obj, string CAMERA_NAME)
{
    // READING INTRINSIC MATRIX
    node_obj.getParam("/" + CAMERA_NAME + "/camera_intrinsic_left/fx", fx_left);
    node_obj.getParam("/" + CAMERA_NAME + "/camera_intrinsic_left/fy", fy_left);
    node_obj.getParam("/" + CAMERA_NAME + "/camera_intrinsic_left/ccx", ccx_left);
    node_obj.getParam("/" + CAMERA_NAME + "/camera_intrinsic_left/ccy", ccy_left);
    node_obj.getParam("/" + CAMERA_NAME + "/camera_intrinsic_right/fx", fx_right);
    node_obj.getParam("/" + CAMERA_NAME + "/camera_intrinsic_right/fy", fy_right);
    node_obj.getParam("/" + CAMERA_NAME + "/camera_intrinsic_right/ccx", ccx_right);
    node_obj.getParam("/" + CAMERA_NAME + "/camera_intrinsic_right/ccy", ccy_right);

    // READING BASELINE VALUE BETWEEN THE TWO CAMERAS
    node_obj.getParam("/" + CAMERA_NAME + "/baseline", baseline);

    // READING DISTORTION COEFFICIENTS
    node_obj.getParam("/" + CAMERA_NAME + "/distortion_coefficient_left/radial/k1", k1_left);
    node_obj.getParam("/" + CAMERA_NAME + "/distortion_coefficient_left/radial/k2", k2_left);
    node_obj.getParam("/" + CAMERA_NAME + "/distortion_coefficient_left/tangential/p1", p1_left);
    node_obj.getParam("/" + CAMERA_NAME + "/distortion_coefficient_left/tangential/p2", p2_left);
    node_obj.getParam("/" + CAMERA_NAME + "/distortion_coefficient_right/radial/k1", k1_right);
    node_obj.getParam("/" + CAMERA_NAME + "/distortion_coefficient_right/radial/k2", k2_right);
    node_obj.getParam("/" + CAMERA_NAME + "/distortion_coefficient_right/tangential/p1", p1_right);
    node_obj.getParam("/" + CAMERA_NAME + "/distortion_coefficient_right/tangential/p2", p2_right);

    // READING MATRIX FROM BODY FRAME TO CAMERA FRAME
    node_obj.getParam("/" + CAMERA_NAME + "/body_camera_matrix/data", data);
    node_obj.getParam("/" + CAMERA_NAME + "/body_camera_matrix/rows", rows);
    node_obj.getParam("/" + CAMERA_NAME + "/body_camera_matrix/cols", cols);
    cv::Mat(rows, cols, CV_64F, data.data()).copyTo(body_camera_matrix);
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


void get_VO_parameters(ros::NodeHandle node_obj)
{
    // Node params
    node_obj.getParam("/node_freq", NODE_FREQ);

    // Preprocessing params
    node_obj.getParam("/preprocessing/desired_width", DESIRED_WIDTH);
    node_obj.getParam("/preprocessing/clahe", CLAHE_CORRECTION);
    node_obj.getParam("/preprocessing/clip_limit", CLIP_LIMIT);

    // VO params
    node_obj.getParam("/vo_params/distance", DISTANCE);
    node_obj.getParam("/vo_params/max_skipped_imgs", SKIPPED_IMGS);
    node_obj.getParam("/vo_params/feature_tracking", FEATURES_TRACKING);
    node_obj.getParam("/vo_params/essential_max_iters", ESSENTIAL_MAX_ITERS);
    node_obj.getParam("/vo_params/essential_confidence", ESSENTIAL_CONFIDENCE);
    node_obj.getParam("/vo_params/essential_threshold", ESSENTIAL_THRESHOLD);
    node_obj.getParam("/vo_params/homography_max_iters", HOMOGRAPHY_MAX_ITERS);
    node_obj.getParam("/vo_params/homography_confidence", HOMOGRAPHY_CONFIDENCE);
    node_obj.getParam("/vo_params/homography_threshold", HOMOGRAPHY_THRESHOLD);
    node_obj.getParam("/vo_params/homography_distance", HOMOGRAPHY_DISTANCE);
    node_obj.getParam("/vo_params/valid_point_fraction", VPF_THRESHOLD);
    node_obj.getParam("/vo_params/reprojection_threshold", REPROJECTION_TOLERANCE);
    node_obj.getParam("/vo_params/min_num_features", MIN_NUM_FEATURES);
    node_obj.getParam("/vo_params/min_num_tracked_features", MIN_NUM_TRACKED_FEATURES);
    node_obj.getParam("/vo_params/min_num_3Dpoints", MIN_NUM_3DPOINTS);
    node_obj.getParam("/vo_params/min_num_inliers", MIN_NUM_INLIERS);

    // PnP params
    node_obj.getParam("/vo_params/iterations_count", ITERATIONS_COUNT);
    node_obj.getParam("/vo_params/reprojection_error", REPROJECTION_ERROR_THRESHOLD);
    node_obj.getParam("/vo_params/confidence", CONFIDENCE);
    node_obj.getParam("/vo_params/use_extrinsic_guess", USE_EXTRINSIC_GUESS);
    
    // Visualization params
    node_obj.getParam("/visualization/fps", FPS);
    node_obj.getParam("/visualization/show_match", SHOW_MATCHES);

    // SURF params
    node_obj.getParam("/surf_params/min_hessian", SURF_MIN_HESSIAN);
    node_obj.getParam("/surf_params/n_octaves", SURF_OCTAVES_NUMBER);
    node_obj.getParam("/surf_params/n_octave_layers", SURF_OCTAVES_LAYERS);
    node_obj.getParam("/surf_params/extended", SURF_EXTENDED);
    node_obj.getParam("/surf_params/upright", SURF_UPRIGHT);
    node_obj.getParam("/surf_params/lowe_ratio", LOWE_RATIO);
    node_obj.getParam("/surf_params/ratio_test", SURF_RATIO_THRESHOLD);
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


void match_features(vector<KeyPoint> keypoints1, vector<KeyPoint> keypoints2, Mat descriptors1, Mat descriptors2, vector<DMatch> &matches)
{
    if (LOWE_RATIO)
    {
        vector< vector< DMatch > > knn_matches;
        float ratio_thresh = SURF_RATIO_THRESHOLD;

        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
        matcher->knnMatch(descriptors1, descriptors2, knn_matches, 2);

        ROS_INFO("MATCHES BEFORE LOWE'S RATIO: %lu", knn_matches.size());

        size_t i = 0;
        for (i; i < knn_matches.size(); i++) 
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                matches.push_back(knn_matches[i][0]);
            }
        }

        ROS_INFO("MATCHES AFTER LOWE'S RATIO: %lu", matches.size());
    }

    else
    {
        vector< DMatch > bf_matches;
        BFMatcher brue_force_matcher = BFMatcher(NORM_L2, true);
        brue_force_matcher.match(descriptors1, descriptors2, bf_matches);

        ROS_INFO("MATCHES: %lu", bf_matches.size());

        size_t i = 0;
        for (i; i < bf_matches.size(); i++) 
        {
            matches.push_back(bf_matches[i]);  
        }

    }
}


// ############################################################ //
// ############################################################ //
// ############################################################ //


void match_features(vector<KeyPoint> keypoints1, vector<KeyPoint> keypoints2, Mat descriptors1, Mat descriptors2, vector<DMatch> &matches, vector<Point2f> &keypoints1_conv, vector<Point2f> &keypoints2_conv)
{
    if (LOWE_RATIO)
    {
        vector< vector< DMatch > > knn_matches;
        float ratio_thresh = SURF_RATIO_THRESHOLD;

        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
        matcher->knnMatch(descriptors1, descriptors2, knn_matches, 2);

        ROS_INFO("MATCHES BEFORE LOWE'S RATIO: %lu", knn_matches.size());

        size_t i = 0;
        for (i; i < knn_matches.size(); i++) 
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                matches.push_back(knn_matches[i][0]);
                keypoints1_conv.push_back(keypoints1[knn_matches[i][0].queryIdx].pt);
                keypoints2_conv.push_back(keypoints2[knn_matches[i][0].trainIdx].pt);
            }
        }

        ROS_INFO("MATCHES AFTER LOWE'S RATIO: %lu", matches.size());
    }

    else
    {
        vector< DMatch > bf_matches;
        BFMatcher brue_force_matcher = BFMatcher(NORM_L2, true);
        brue_force_matcher.match(descriptors1, descriptors2, bf_matches);

        ROS_INFO("MATCHES: %lu", bf_matches.size());

        size_t i = 0;
        for (i; i < bf_matches.size(); i++) 
        {
            matches.push_back(bf_matches[i]);
            keypoints1_conv.push_back(keypoints1[bf_matches[i].queryIdx].pt);
            keypoints2_conv.push_back(keypoints2[bf_matches[i].trainIdx].pt);    
        }

    }
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


int recover_pose_homography(Mat H, vector<Point2f> inliers1, vector<Point2f> inliers2, Mat cameraMatrix, Mat& R, Mat& t)
{
    // DECOMPOSE HOMOGRAPHY MATRIX
    vector<Mat> R_candidates, t_candidates;
    int solutions = decomposeHomographyMat(H, cameraMatrix, R_candidates, t_candidates, noArray());

    Mat eye_m = (Mat1d(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    Mat zero_v = (Mat1d(3, 1) << 0, 0, 0);
    Mat proj_std = compute_projection_matrix(eye_m, zero_v, cameraMatrix);

    vector<Mat> triangulateCandidates(solutions);
    vector<int> n_goodTP(solutions);
    

    for(int i = 0; i < solutions; i++)
    {
        triangulatePoints(proj_std, compute_projection_matrix(R_candidates[i], t_candidates[i], cameraMatrix), inliers1, inliers2, triangulateCandidates[i]);
        triangulateCandidates[i] = convert_from_homogeneous_coords(triangulateCandidates[i]);


        for(int j = 0; j < triangulateCandidates[i].cols; j++)
        {
            Mat triangulateCandidates_col = triangulateCandidates[i].col(j);
            bool good_condition = ((triangulateCandidates_col.at<double>(2) > 0) && (triangulateCandidates_col.at<double>(2) < HOMOGRAPHY_DISTANCE));

            if(good_condition)
            {
                n_goodTP[i]++;
            }
        }
    }

  
    int idx_max = max_element(n_goodTP.begin(), n_goodTP.end()) - n_goodTP.begin();

    double translation_norm = sqrt(t_candidates[idx_max].at<double>(0) * t_candidates[idx_max].at<double>(0) + t_candidates[idx_max].at<double>(1) * t_candidates[idx_max].at<double>(1) + t_candidates[idx_max].at<double>(2) * t_candidates[idx_max].at<double>(2));
    R = R_candidates[idx_max];
    t = t_candidates[idx_max] / translation_norm;

    return n_goodTP[idx_max];
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


vector<double> reproject_errors(Mat world_points, Mat R, Mat t, Mat cameraMatrix, vector<Point2f> img_points)
{
    Mat reprojected_points;
    Mat Rvec;
    Rodrigues(R, Rvec);

    // REPROJECT 3D POINTS IN THE IMAGE PLANE
    projectPoints(world_points, Rvec, t, cameraMatrix, noArray(), reprojected_points); 

    // COMPUTE REPROJECTION ERROR
    vector<double> reproject_err(img_points.size());
    for(int i = 0; i < reprojected_points.rows; i++)
    {
        reproject_err[i] = sqrt(pow(img_points[i].x - reprojected_points.at<double>(i, 0), 2.0) + pow(img_points[i].y - reprojected_points.at<double>(i, 1), 2.0));
    }

    return reproject_err;
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


void resize_camera_matrix(Mat original_image, Mat& cameraMatrix, Mat distortionCoeff, Mat& newCamMatrix)
{
    int original_width  = original_image.cols;
    int original_height = original_image.rows;
    double ratio        = (double)original_width / (double)DESIRED_WIDTH;
    int desired_height  = (int)(original_height / ratio);

    double skew_ = cameraMatrix.at<double>(0, 1);
    cameraMatrix = cameraMatrix / ratio;
    cameraMatrix.at<double>(0, 1) = skew_; 
    cameraMatrix.at<double>(2, 2) = 1;

    newCamMatrix = getOptimalNewCameraMatrix(cameraMatrix, distortionCoeff, Size(DESIRED_WIDTH,desired_height), 0, Size(DESIRED_WIDTH,desired_height), 0);
}  


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


void select_desired_descriptors(const Mat &descriptors, Mat &descriptors_desired, const Mat &indexes)
{
    for (int i = 0; i < indexes.rows; i++)
    {
        descriptors_desired.push_back(descriptors.row(indexes.at<int>(i)));
    }
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


void select_desired_keypoints(vector<KeyPoint> keypoints, vector<KeyPoint> &keypoints_desired, Mat indexes)
{
    for (int i = 0; i < indexes.rows; i++) 
    {   
        keypoints_desired.push_back(keypoints[indexes.at<int>(i)]);
    }
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


Mat show_matches(vector<KeyPoint> kpoints1, vector<KeyPoint> kpoints2, vector<DMatch> matches, Mat img1, Mat img2)
{
    Mat img_matches;
    drawMatches( img1, kpoints1, img2, kpoints2, matches, img_matches, Scalar::all(-1),
                 Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    return img_matches;
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


void track_features(Mat img_1, Mat img_2, vector<KeyPoint> keypoints1, vector<KeyPoint> &keypoints2, Mat descriptors1, Mat &descriptors2, vector<DMatch> &matches)
{ 
    vector<uchar> status;
    vector<float> err;                    
    Size winSize=Size(151,151);                                                                                              
    TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);

    vector<Point2f> keypoints1_conv, keypoints2_conv;
    KeyPoint::convert(keypoints1, keypoints1_conv, vector<int>());
    
    calcOpticalFlowPyrLK(img_1, img_2, keypoints1_conv, keypoints2_conv, status, err, winSize, 3, termcrit, 0, 0.001);

    Mat good_status_vector;

    for( int i=0; i<status.size(); i++)
    {
        Point2f pt2 = keypoints2_conv.at(i);
        descriptors2.push_back(descriptors1.row(i));

        if ((status[i] == 1) && (pt2.x>=0) && (pt2.x<=img_1.cols) && (pt2.y<=img_1.rows) && (pt2.y>=0))
        {
            good_status_vector.push_back(i);
        }  
    }

    KeyPoint::convert(keypoints2_conv, keypoints2);
    int good_status_number = good_status_vector.rows;
    matches.resize(good_status_number);

    for(int i = 0; i < good_status_number; i++)
    {
        matches[i].queryIdx  = good_status_vector.at<int>(i);
        matches[i].trainIdx  = good_status_vector.at<int>(i);         
    }

    ROS_INFO("FEATURES TRACKED - CURR. IMAGE: %d", good_status_number);
}


// ############################################################ //
// ############################################################ //
// ############################################################ //


void track_features(Mat img_1, Mat img_2, vector<KeyPoint> &keypoints1, vector<KeyPoint> &keypoints2, Mat descriptors1, Mat &descriptors2, vector<DMatch> &matches, vector<Point2f> &keypoints1_conv, vector<Point2f> &keypoints2_conv)
{ 
    vector<uchar> status;
    vector<float> err;                    
    Size winSize=Size(41,41);                                                                                             
    TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.5);

    vector<Point2f> total_keypoints1_conv, total_keypoints2_conv;
    KeyPoint::convert(keypoints1, total_keypoints1_conv, vector<int>());
    
    calcOpticalFlowPyrLK(img_1, img_2, total_keypoints1_conv, total_keypoints2_conv, status, err, winSize, 3, termcrit);

    for( int i=0; i<status.size(); i++)
    {
        Point2f pt1 = total_keypoints1_conv.at(i);
        Point2f pt2 = total_keypoints2_conv.at(i);
        if ((status[i] == 1) && (pt2.x>=0) && (pt2.x<=img_1.cols) && (pt2.y<=img_1.rows) && (pt2.y>=0))
        {
            keypoints1_conv.push_back(pt1);
            keypoints2_conv.push_back(pt2);
            descriptors2.push_back(descriptors1.row(i));
        }
    }

    keypoints1.clear();
    KeyPoint::convert(keypoints1_conv, keypoints1);
    KeyPoint::convert(keypoints2_conv, keypoints2);

    matches.resize(keypoints2_conv.size());
    
    for(int i = 0; i < keypoints2_conv.size(); i++)
    {
        matches[i].queryIdx  = i;
        matches[i].trainIdx  = i;       
    }

    ROS_INFO("FEATURES TRACKED - CURR. IMAGE: %lu", keypoints2.size());
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //

