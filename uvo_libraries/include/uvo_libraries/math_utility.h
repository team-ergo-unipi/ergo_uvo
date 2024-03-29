#ifndef MATH_UTILITY_H
#define MATH_UTILITY_H


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////// LIBRARIES //////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <opencv2/opencv.hpp> 
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>

using namespace std;
using namespace cv;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////// FUNCTIONS //////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void check_rotation_matrix(Mat &matrix);
Mat compute_jacobian(geometry_msgs::Vector3 rpy);
vector<double> compute_mean_and_variance(Mat v);
double compute_median(vector<double> v);
Mat cross_product(Mat first_vector, Mat second_vector);
geometry_msgs::Vector3 from_mat_to_vector_type(Mat pos_mat);
Mat from_euler_angles_to_quaternion(double roll, double pitch, double yaw);
Mat from_ros_to_cv_image(const sensor_msgs::CompressedImage::ConstPtr& image);
geometry_msgs::Vector3 from_rotation_matrix_to_euler_angles(Mat &R);
Mat from_vector_to_mat_type(geometry_msgs::Vector3 vec);
Mat from_vector_to_skew_matrix(geometry_msgs::Vector3 vec);
bool isRotationMatrix(Mat &R);
geometry_msgs::Vector3 ll2ne(geometry_msgs::Vector3 ll0, geometry_msgs::Vector3 ll);
geometry_msgs::Vector3 lld2ned(geometry_msgs::Vector3 lld0, geometry_msgs::Vector3 lld);
geometry_msgs::Vector3 ne2ll(const geometry_msgs::Vector3 ll0, const geometry_msgs::Vector3 ne);
Mat rotx(double angle);
Mat roty(double angle);
Mat rotz(double angle);
Mat transform_coordinates(Mat vector, Mat R, Mat t); 
double wrap2pi(double angle);

#endif