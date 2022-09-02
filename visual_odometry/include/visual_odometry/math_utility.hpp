// ######################################################################################################################################################## //
// ######################################################################################################################################################## //
//                                                                                                                                                          //
//                                  This file contains all the math functions exploited within the visual odometry node                                     //                                                                     
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
geometry_msgs::Vector3 from_mat_to_vector_type(Mat pos_mat);
Mat from_ros_to_cv_image(sensor_msgs::CompressedImage image);
geometry_msgs::Vector3 from_rotation_matrix_to_euler_angles(Mat &R);
Mat from_vector_to_mat_type(geometry_msgs::Vector3 vec);
Mat from_vector_to_skew_matrix(geometry_msgs::Vector3 vec);
bool isRotationMatrix(Mat &R);
geometry_msgs::Vector3 ll2ne(geometry_msgs::Vector3 ll0, geometry_msgs::Vector3 ll);
geometry_msgs::Vector3 lld2ned(geometry_msgs::Vector3 lld0, geometry_msgs::Vector3 lld);
Mat rotx(double angle);
Mat roty(double angle);
Mat rotz(double angle);
Mat transform_coordinates(Mat vector, Mat R, Mat t); 
double wrap2pi(double angle);


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


void check_rotation_matrix(Mat &matrix)
{
    if(!isRotationMatrix(matrix))
    {
        Mat w, U, Vt;
        SVDecomp(matrix, w, U, Vt);
        matrix = U * Vt;
    }
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


Mat compute_jacobian(geometry_msgs::Vector3 rpy)
{
    return (Mat1d(3, 3) << 1, sin(rpy.x)*tan(rpy.y), cos(rpy.x)*tan(rpy.y), 0, cos(rpy.x), -sin(rpy.x), 0, sin(rpy.x)/cos(rpy.y), cos(rpy.x)/cos(rpy.y));
}

// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


vector<double> compute_mean_and_variance(Mat v)
{
    CV_Assert((v.cols == 1) && (v.type() == CV_64F));
    int N = v.rows;
    int i = 0;

    double mean = 0.0;
    for(i; i < N; i++)
    {
        mean += v.at<double>(i)/N;
    }

    double variance = 0.0;
    
    for(i = 0; i < N; i++)
    {
        variance += pow(v.at<double>(i) - mean, 2.0)/N;
    }
    
    vector<double> statistic_values {mean, variance};
    
    return statistic_values;
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


double compute_median(vector<double> v)
{
  size_t size = v.size();

  if (size == 0)
  {
    ROS_ERROR("Empy vector");
  }

  else
  {
    sort(v.begin(), v.end());
    if (size % 2 == 0)
      return (v[size / 2.0 - 1] + v[size / 2.0]) / 2.0;

    else 
      return v[size / 2.0];
  }
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


geometry_msgs::Vector3 from_mat_to_vector_type(Mat pos_mat)
{
    geometry_msgs::Vector3 pos;
    pos.x = pos_mat.at<double>(0);
    pos.y = pos_mat.at<double>(1);
    pos.z = pos_mat.at<double>(2);

    return pos;
} 


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


Mat from_ros_to_cv_image(const sensor_msgs::CompressedImage::ConstPtr& image)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(image);
    }

    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    if (image->format.find("bayer") != std::string::npos)
    {
        cvtColor(cv_ptr->image, cv_ptr->image, COLOR_BayerBGGR2BGR);
    }

    return cv_ptr->image;
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


geometry_msgs::Vector3 from_rotation_matrix_to_euler_angles(Mat &R)
{
    check_rotation_matrix(R);

    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6;

    geometry_msgs::Vector3 angles;

    if (!singular)
    {
        angles.x = atan2(R.at<double>(2,1), R.at<double>(2,2));
        angles.y = atan2(-R.at<double>(2,0), sy);
        angles.z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        ROS_ERROR("ROTATION MATRIX IS CLOSE TO SINGULARITY!!!");
        angles.x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        angles.y = atan2(-R.at<double>(2,0), sy);
        angles.z = 0;
    }
    return angles;
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


Mat from_vector_to_mat_type(geometry_msgs::Vector3 vec)
{
    return (Mat1d(3, 1) << vec.x, vec.y, vec.z);
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


Mat from_vector_to_skew_matrix(geometry_msgs::Vector3 vec)
{
    return (Mat1d(3, 3) << 0, vec.z, -vec.y, -vec.z, 0, vec.x, vec.y, -vec.x, 0);
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


bool isRotationMatrix(Mat &R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());

    return  norm(I, shouldBeIdentity) < 1e-6;
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


geometry_msgs::Vector3 ll2ne(geometry_msgs::Vector3 ll0, geometry_msgs::Vector3 ll)
{      
  double lat0 = ll0.x;
  double lon0 = ll0.y;
  double lat  = ll.x;
  double lon  = ll.y;

  geometry_msgs::Vector3 ne;

  lat   = lat * M_PI/180;
  lon   = lon * M_PI/180;
  lat0  = lat0 * M_PI/180;
  lon0  = lon0 * M_PI/180;

  double dlat = lat - lat0;
  double dlon = lon - lon0;

  double a = 6378137.0;
  double f = 1 / 298.257223563;
  double Rn = a / sqrt(1 - (2 * f - f * f) * sin(lat0) * sin(lat0));
  double Rm = Rn * (1 - (2 * f - f * f)) / (1 - (2 * f - f * f) * sin(lat0) * sin(lat0));

  ne.x = dlat / atan2(1, Rm);
  ne.y = dlon / atan2(1, Rn * cos(lat0));

  return ne;      
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


geometry_msgs::Vector3 lld2ned(geometry_msgs::Vector3 lld0, geometry_msgs::Vector3 lld) 
{
    geometry_msgs::Vector3 ned;
    geometry_msgs::Vector3 ne;

    ne = ll2ne(lld0, lld);

    ned.x = ne.x;
    ned.y = ne.y;
    ned.z = lld.z - lld0.z;

    return ned;
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


Mat rotx(double angle)
{
    return (Mat1d(3, 3) << 1, 0, 0, 0, cos(angle), -sin(angle), 0, sin(angle), cos(angle));
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


Mat roty(double angle)
{
    return (Mat1d(3, 3) << cos(angle), 0, sin(angle), 0, 1, 0, -sin(angle), 0, cos(angle));
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


Mat rotz(double angle)
{
    return (Mat1d(3, 3) << cos(angle), -sin(angle), 0, sin(angle), cos(angle), 0, 0, 0, 1);
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


Mat transform_coordinates(Mat vector, Mat R, Mat t)
{
    return R*vector + t;
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


double wrap2pi(double angle)
{
    return atan2(sin(angle), cos(angle));
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //










