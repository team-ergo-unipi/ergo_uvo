// ######################################################################################################################################################## //
// ######################################################################################################################################################## //
//                                                                                                                                                          //
//                                                       The stereo visual odometry node                                                                    //                                              
//                                                                                                                                                          //
// -------------------------------------------------------------------------------------------------------------------------------------------------------- //
//                            Author:  Francesco Ruscio       Email: francesco.ruscio@phd.unipi.it     Date:  06/06/2022                                    //
//                            Author:  Simone Tani            Email: simone.tani@phd.unipi.it          Date:  06/06/2022                                    //
//                                                                                                                                                          //
// ######################################################################################################################################################## //
// ######################################################################################################################################################## //


#include <visual_odometry/VO_utility.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// SUPPORT FUNCTION
void stereo_output_computation(geometry_msgs::Vector3Stamped estimated_rpy, geometry_msgs::Vector3Stamped estimated_position_NED, 
                      geometry_msgs::Vector3Stamped estimated_linear_velocity_body, geometry_msgs::Vector3Stamped estimated_angular_velocity_body, 
                      Mat R_world_currCam, Mat R_body_cam, ros::Duration deltaT, Mat R_prevCam_currCam, Mat t_prevCam_currCam,
                      Mat world_points, Mat good_currCam_points, Mat world_position, Mat R_world_body, std_msgs::Bool successful_estimate);

// MSGS
std_msgs::Header img_header;

// SUPPORT VARIABLES
Mat camera_left, camera_right;
bool first_img = false;
bool new_imgs_available = false;
string CAMERA_NAME;

// OUTPUT VARIABLES
geometry_msgs::Vector3Stamped estimated_linear_velocity_body, estimated_position_NED, estimated_rpy, estimated_angular_velocity_body;
std_msgs::Bool successful_estimate; // VARIABLE FOR CHECKING SUCCESS/FAIL

// PUBLISHERS
ros::Publisher pub_estimated_angular_vel, pub_estimated_linear_vel, pub_estimated_rpy, pub_estimated_pos, pub_validity, pub_inliers, pub_camera_pointcloud, pub_world_pointcloud;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void camera_imgs_callback(const sensor_msgs::CompressedImage::ConstPtr& left_image, const sensor_msgs::CompressedImage::ConstPtr& right_image)
{
    camera_left         = from_ros_to_cv_image(left_image);
    camera_right        = from_ros_to_cv_image(right_image);
    img_header          = left_image->header;
    first_img           = true;
    new_imgs_available  = true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_VO");
    ros::NodeHandle node_obj;

    pub_estimated_angular_vel   = node_obj.advertise<geometry_msgs::Vector3Stamped>("/estimated_angular_vel_stereoVO", 10);
    pub_estimated_linear_vel    = node_obj.advertise<geometry_msgs::Vector3Stamped>("/estimated_linear_vel_stereoVO", 10);
    pub_estimated_rpy           = node_obj.advertise<geometry_msgs::Vector3Stamped>("/estimated_rpy_stereoVO", 10);
    pub_estimated_pos           = node_obj.advertise<geometry_msgs::Vector3Stamped>("/estimated_pos_stereoVO", 10);
    pub_validity                = node_obj.advertise<std_msgs::Bool>("/validity_stereoVO", 10);
    pub_camera_pointcloud       = node_obj.advertise<sensor_msgs::PointCloud2>("/camera_points_stereoVO", 10);
    pub_world_pointcloud        = node_obj.advertise<sensor_msgs::PointCloud2>("/world_points_stereoVO", 10);

    message_filters::Subscriber<sensor_msgs::CompressedImage> sub_cameraSX(node_obj, "/image_left/compressed", 1);
    message_filters::Subscriber<sensor_msgs::CompressedImage> sub_cameraDX(node_obj, "/image_right/compressed", 1);

    // APPROXIMATE TIME SYNCRHRONIZER
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;
    sync_.reset(new Sync(MySyncPolicy(10), sub_cameraSX, sub_cameraDX));
    sync_->registerCallback(boost::bind(&camera_imgs_callback, _1, _2));

    // GET PARAMETERS
    node_obj.getParam("/camera_name", CAMERA_NAME);
    get_VO_parameters(node_obj);
    get_stereo_camera_parameters(node_obj, CAMERA_NAME);

    ros::Rate loop_rate(NODE_FREQ);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////// WAITING FOR THE FIRST SENSORS MSGS ///////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    while(!first_img && ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_WARN(" ###################################################################################################################### ");
    ROS_WARN(" ############################################# STARTING VISUAL ODOMETRY NODE ########################################## ");
    ROS_WARN(" ###################################################################################################################### ");

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////// LOADING CAMERA PARAMETERS FROM YAML FILE ///////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    Mat camera_matrix_left   = (Mat1d(3, 3) << fx_left, 0, ccx_left, 0, fy_left, ccy_left, 0, 0, 1);     
    Mat camera_matrix_right  = (Mat1d(3, 3) << fx_right, 0, ccx_right, 0, fy_right, ccy_right, 0, 0, 1);

    Mat distortion_left     = (Mat1d(1, 4) << k1_left, k2_left, p1_left, p2_left);                             
    Mat distortion_right    = (Mat1d(1, 4) << k1_right, k2_right, p1_right, p2_right);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////// STEREO PARAMETERS FOR TRIANGULATION //////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // WE ASSUME THE LEFT CAMERA AS THE WORLD FRAME
    Mat R_left  = (Mat1d(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    Mat t_left  = (Mat1d(3, 1) << 0, 0, 0);

    // ASSUMING LEFT CAMERA AS THE WORLD FRAME, THE RIGHT CAMERA RESULT TO BE ONLY TRANSLATED BY THE BASELINE VALUE
    Mat R_right = (Mat1d(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    Mat t_right = (Mat1d(3, 1) << -baseline, 0, 0);
    
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////// INITIALIZATION OF VARIABLES FOR THE MAIN LOOP ///////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    Mat R_body_cam = body_camera_matrix;

    // INITIALIZATION OF GLOBAL ORIENTATION AND POSITION
    Mat R_world_body    = Mat::eye(3, 3, CV_64F);
    Mat R_world_currCam = R_world_body * R_body_cam;
    Mat world_position  = (Mat1d(3, 1) << 0, 0, 0);
    Mat angular_vector  = Mat::zeros(3, 1, CV_64F); 

    //  INITIALIZATION OF RELATIVE ORIENTATION AND RELATIVE TRANSLATION
    Mat previous_R_prevCam_currCam  = Mat::eye(3, 3, CV_64F);
    Mat previous_t_prevCam_currCam  = Mat::zeros(3, 1, CV_64F);
    Mat R_currCam_prevCam_Vec       = Mat::zeros(3, 1, CV_64FC1); 
    Mat R_currCam_prevCam           = Mat::eye(3, 3, CV_64FC1);           
    Mat R_prevCam_currCam           = Mat::eye(3, 3, CV_64FC1);           
    Mat t_currCam_prevCam           = Mat::zeros(3, 1, CV_64FC1);            
    Mat t_prevCam_currCam           = Mat::zeros(3, 1, CV_64FC1);
    Mat distCoeffs                  = Mat::zeros(4, 1, CV_64FC1);  // PnP distortion coefficients          

    // TIME VARIABLES
    ros::Time prev_time, curr_time;
    ros::Duration deltaT, previous_deltaT;
    previous_deltaT = ros::Duration(1.0);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////// READING THE FIRST TWO IMAGES /////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    Mat new_camera_matrix_left, new_camera_matrix_right;
    resize_camera_matrix(camera_left, camera_matrix_left, distortion_left, new_camera_matrix_left);
    resize_camera_matrix(camera_right, camera_matrix_right, distortion_right, new_camera_matrix_right);

    Mat left_projection_matrix  = compute_projection_matrix(R_left, t_left, new_camera_matrix_left); 
    Mat right_projection_matrix = compute_projection_matrix(R_right, t_right, new_camera_matrix_right); 

    prev_time           = img_header.stamp;
    Mat prev_left_img   = get_image(camera_left, camera_matrix_left, distortion_left, new_camera_matrix_left);
    Mat prev_right_img  = get_image(camera_right, camera_matrix_right, distortion_right, new_camera_matrix_right);
    new_imgs_available          = false;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////// PROCESSING THE FIRST COUPLE OF IMAGES /////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    vector<KeyPoint> prev_left_keypoints, prev_right_keypoints;
    Mat prev_left_descr, prev_right_descr;
    detect_features(prev_left_img, prev_left_keypoints, prev_left_descr);
    detect_features(prev_right_img, prev_right_keypoints, prev_right_descr);

    vector<DMatch> results_match_prev;
    match_features(prev_left_keypoints, prev_right_keypoints, prev_left_descr, prev_right_descr, results_match_prev);
    
    if(SHOW_MATCHES)
    {
        Mat img_matches_stereo = show_matches(prev_left_keypoints, prev_right_keypoints, results_match_prev, prev_left_img, prev_right_img);
        imshow("MATCHING STEREO PAIR", img_matches_stereo);
        waitKey(FPS);
    }

    Mat prev_stereo_match_indexPairs_left, prev_stereo_match_indexPairs_right;
    for (size_t i = 0; i < results_match_prev.size(); i++)
    {
        prev_stereo_match_indexPairs_left.push_back(results_match_prev[i].queryIdx);
        prev_stereo_match_indexPairs_right.push_back(results_match_prev[i].trainIdx);
    }

    Mat prev_left_descr_after_stereo_match, prev_right_descr_after_stereo_match;
    vector<KeyPoint> prev_left_keypoints_after_stereo_match, prev_right_keypoints_after_stereo_match;
    select_desired_descriptors(prev_left_descr, prev_left_descr_after_stereo_match, prev_stereo_match_indexPairs_left);
    select_desired_descriptors(prev_right_descr, prev_right_descr_after_stereo_match, prev_stereo_match_indexPairs_right);
    select_desired_keypoints(prev_left_keypoints, prev_left_keypoints_after_stereo_match, prev_stereo_match_indexPairs_left);
    select_desired_keypoints(prev_right_keypoints, prev_right_keypoints_after_stereo_match, prev_stereo_match_indexPairs_right);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////// MAIN LOOP OF THE STEREO VISUAL ODOMETRY NODE /////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        if (new_imgs_available)
        {
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////// PROCESSING THE CURRENT COUPLE OF IMAGES ////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            ROS_WARN(" < ------------------- PROCESSING STEREO PAIR ------------------- > ");

            curr_time = img_header.stamp;
            deltaT = curr_time - prev_time;

            Mat curr_left_img       = get_image(camera_left, camera_matrix_left, distortion_left, new_camera_matrix_left);
            Mat curr_right_img      = get_image(camera_right, camera_matrix_right, distortion_right, new_camera_matrix_right);
            new_imgs_available              = false;

            vector<KeyPoint> curr_left_keypoints, curr_right_keypoints;
            Mat curr_left_descr, curr_right_descr;
            detect_features(curr_left_img, curr_left_keypoints, curr_left_descr);
            detect_features(curr_right_img, curr_right_keypoints, curr_right_descr);

            vector<DMatch> results_match_curr;
            match_features(curr_left_keypoints, curr_right_keypoints, curr_left_descr, curr_right_descr, results_match_curr);
            
            if(SHOW_MATCHES)
            {
                Mat img_matches_stereo = show_matches(curr_left_keypoints, curr_right_keypoints, results_match_curr, curr_left_img, curr_right_img);
                imshow("MATCHING STEREO PAIR", img_matches_stereo);
                waitKey(FPS);
            }
             

            Mat curr_stereo_match_indexPairs_left, curr_stereo_match_indexPairs_right;
            for (size_t i = 0; i < results_match_curr.size(); i++)
            {
                curr_stereo_match_indexPairs_left.push_back(results_match_curr[i].queryIdx);
                curr_stereo_match_indexPairs_right.push_back(results_match_curr[i].trainIdx);
            }

            Mat curr_left_descr_after_stereo_match, curr_right_descr_after_stereo_match;
            vector<KeyPoint> curr_left_keypoints_after_stereo_match, curr_right_keypoints_after_stereo_match;
            select_desired_descriptors(curr_left_descr, curr_left_descr_after_stereo_match, curr_stereo_match_indexPairs_left);
            select_desired_descriptors(curr_right_descr, curr_right_descr_after_stereo_match, curr_stereo_match_indexPairs_right);
            select_desired_keypoints(curr_left_keypoints, curr_left_keypoints_after_stereo_match, curr_stereo_match_indexPairs_left);
            select_desired_keypoints(curr_right_keypoints, curr_right_keypoints_after_stereo_match, curr_stereo_match_indexPairs_right);

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////// PROCESSING PREVIOUS AND CURRENT LEFT IMAGES //////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            ROS_WARN(" < ------------------- TRIANGULAR MATCHING ------------------- > ");

            vector< DMatch > results_match_prev_curr;
            
            // FEATURE TRACKING
            if(FEATURES_TRACKING)
            {
                ROS_INFO("FEATURES - PREV. IMAGE: %lu", prev_left_keypoints_after_stereo_match.size());

                curr_left_keypoints.clear();
                curr_left_descr.release();

                track_features(prev_left_img, curr_left_img, prev_left_keypoints_after_stereo_match, curr_left_keypoints, prev_left_descr_after_stereo_match, curr_left_descr, results_match_prev_curr);
            }

            // FEATURE MATCHING
            else
            {   
                match_features(prev_left_keypoints_after_stereo_match, curr_left_keypoints, prev_left_descr_after_stereo_match, curr_left_descr, results_match_prev_curr);
            }



            if(SHOW_MATCHES)
            {
                Mat img_matches_left = show_matches(prev_left_keypoints_after_stereo_match, curr_left_keypoints, results_match_prev_curr, prev_left_img, curr_left_img);
                imshow("TRAINGULAR MATCHING", img_matches_left);
                waitKey(FPS);
            }



            Mat prev_curr_match_indexPairs_left, prev_curr_match_indexPairs_right;
            for (size_t i = 0; i < results_match_prev_curr.size(); i++)
            {
                prev_curr_match_indexPairs_left.push_back(results_match_prev_curr[i].queryIdx);
                prev_curr_match_indexPairs_right.push_back(results_match_prev_curr[i].trainIdx);  
            }

            vector<KeyPoint> prev_left_keypoints_after_triangular_match, prev_right_keypoints_after_triangular_match, curr_left_keypoints_after_triangular_match;
            // 
            select_desired_keypoints(prev_left_keypoints_after_stereo_match, prev_left_keypoints_after_triangular_match, prev_curr_match_indexPairs_left);
            select_desired_keypoints(prev_right_keypoints_after_stereo_match, prev_right_keypoints_after_triangular_match, prev_curr_match_indexPairs_left);
            select_desired_keypoints(curr_left_keypoints, curr_left_keypoints_after_triangular_match, prev_curr_match_indexPairs_right);

            cout << "TAGLIA PREV LEFT AFTER TRIANGULAR MATCH: " << prev_left_keypoints_after_triangular_match.size() << endl;
            cout << "TAGLIA CURR LEFT AFTER TRIANGULAR MATCH: " << curr_left_keypoints_after_triangular_match.size() << endl;

            vector<Point2f> prev_left_keypoints_after_triangular_match_converted, prev_right_keypoints_after_triangular_match_converted;
            KeyPoint::convert(prev_left_keypoints_after_triangular_match, prev_left_keypoints_after_triangular_match_converted, vector<int>());
            KeyPoint::convert(prev_right_keypoints_after_triangular_match, prev_right_keypoints_after_triangular_match_converted, vector<int>());

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////// TRIANGULATION OF 2D POINTS OF THE PREVIOUS IMAGES //////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            
            Mat good_prevCam_points, points4D, good_indexes, good_currCam_points, world_points;

            if(results_match_prev_curr.size() > MIN_NUM_FEATURES)
            {
                // TRIANGULATION IS DONE BETWEEN PREVIOUS LEFT CAMERA AND PREVIOUS RIGHT CAMERA, THUS:
                // AT THIS LEVEL, 3D POINTS ARE EXPRESSED IN THE FRAME ASSOCIATED TO THE PREVIOUS LEFT CAMERA
                triangulatePoints(left_projection_matrix, right_projection_matrix, prev_left_keypoints_after_triangular_match_converted, prev_right_keypoints_after_triangular_match_converted, points4D);
                extract_3Dpoints(prev_left_keypoints_after_triangular_match_converted, prev_right_keypoints_after_triangular_match_converted, R_left, t_left, R_right, t_right, new_camera_matrix_left, new_camera_matrix_right, points4D, good_prevCam_points, good_indexes);
                

                if(good_prevCam_points.rows > MIN_NUM_3DPOINTS)
                {
                    vector<KeyPoint> good_curr_left_keypoints_after_triangular_match;
                    select_desired_keypoints(curr_left_keypoints_after_triangular_match, good_curr_left_keypoints_after_triangular_match, good_indexes);
                    vector<Point2f> curr_left_keypoints_after_triangular_match_converted;
                    KeyPoint::convert(good_curr_left_keypoints_after_triangular_match, curr_left_keypoints_after_triangular_match_converted);
                    
                    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    ////////////////////////////// EXTRACTION OF ROTATION AND TRANSLATION - PNP METHOD ////////////////////////////////////
                    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
               
                    Mat inliers_idx;
                     
                    int flags = SOLVEPNP_EPNP;

                    cv::solvePnPRansac(good_prevCam_points, curr_left_keypoints_after_triangular_match_converted, new_camera_matrix_left, distCoeffs, R_currCam_prevCam_Vec, t_currCam_prevCam,
                                        USE_EXTRINSIC_GUESS, ITERATIONS_COUNT, REPROJECTION_ERROR_THRESHOLD, CONFIDENCE, inliers_idx, flags);
                    

                    vector<DMatch> inlier_matches = results_match_prev_curr;
                    inlier_matches.resize(inliers_idx.rows);
                    
                    for(int i = 0; i < inliers_idx.rows; i++)
                    {
                        inlier_matches[i].queryIdx  = results_match_prev_curr[inliers_idx.at<int>(i)].queryIdx;
                        inlier_matches[i].trainIdx  = results_match_prev_curr[inliers_idx.at<int>(i)].trainIdx;      
                    }

                    if(SHOW_MATCHES)
                    {
                        Mat img_matches_inliers = show_matches(prev_left_keypoints_after_stereo_match, curr_left_keypoints, inlier_matches, prev_left_img, curr_left_img);
                        imshow("MATCHING AFTER RANSAC", img_matches_inliers);
                        waitKey(FPS);
                    }

                    if(inliers_idx.rows < MIN_NUM_INLIERS)
                    {
                        ROS_WARN("NUMBER OF INLIERS AFTER PNP IS TOO LOW - ASSUMING CONSTANT MOTION");
                        t_prevCam_currCam = previous_t_prevCam_currCam * deltaT.toSec() / previous_deltaT.toSec();
                        angular_vector    = from_vector_to_mat_type(estimated_angular_velocity_body.vector);
                        R_prevCam_currCam = (deltaT.toSec() * from_vector_to_skew_matrix(from_mat_to_vector_type(R_body_cam.t() * angular_vector)) + Mat::eye(3, 3, CV_64F)) * previous_R_prevCam_currCam;
                        check_rotation_matrix(R_prevCam_currCam);
                        successful_estimate.data    = 0;         
                    }
                    else
                    {
                       Rodrigues(R_currCam_prevCam_Vec, R_currCam_prevCam);
                        R_prevCam_currCam = R_currCam_prevCam.t();                                       
                        t_prevCam_currCam = - R_currCam_prevCam.t() * t_currCam_prevCam;

                        successful_estimate.data    = 1;
                        good_currCam_points = convert_3Dpoints_camera(good_prevCam_points, R_currCam_prevCam, t_currCam_prevCam); 
                    }
                }

                else
                {
                    ROS_WARN("NOT ENOUGH TRIANGULATED POINTS - ASSUMING CONSTANT MOTION");
                    t_prevCam_currCam = previous_t_prevCam_currCam * deltaT.toSec() / previous_deltaT.toSec();
                    angular_vector    = from_vector_to_mat_type(estimated_angular_velocity_body.vector);
                    R_prevCam_currCam = (deltaT.toSec() * from_vector_to_skew_matrix(from_mat_to_vector_type(R_body_cam.t() * angular_vector)) + Mat::eye(3, 3, CV_64F)) * previous_R_prevCam_currCam;
                    check_rotation_matrix(R_prevCam_currCam);

                    successful_estimate.data    = 0;
                }

            }
            else
            {
                ROS_WARN("NUMBER OF FEATURES AFTER TRIANGULAR MATCHING IS TOO LOW - ASSUMING CONSTANT MOTION");
                t_prevCam_currCam = previous_t_prevCam_currCam * deltaT.toSec() / previous_deltaT.toSec();
                angular_vector    = from_vector_to_mat_type(estimated_angular_velocity_body.vector);
                R_prevCam_currCam = (deltaT.toSec() * from_vector_to_skew_matrix(from_mat_to_vector_type(R_body_cam.t() * angular_vector)) + Mat::eye(3, 3, CV_64F)) * previous_R_prevCam_currCam;
                check_rotation_matrix(R_prevCam_currCam);

                successful_estimate.data    = 0;
            }        

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////// CREATING AND PUBLISHING OUTPUT ////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            vector<Mat> absPose = compute_absolute_pose(R_world_currCam, world_position, R_currCam_prevCam, t_currCam_prevCam, 1.0, good_currCam_points);
            world_position      = absPose[0];
            R_world_currCam     = absPose[1];
            if(good_currCam_points.cols >= MIN_NUM_3DPOINTS){world_points = absPose[2];}

            R_world_body    = R_world_body * R_body_cam * R_prevCam_currCam * R_body_cam.t();

            stereo_output_computation(estimated_rpy, estimated_position_NED, estimated_linear_velocity_body, estimated_angular_velocity_body, 
                R_world_currCam, R_body_cam, deltaT, R_prevCam_currCam, t_prevCam_currCam, world_points, good_currCam_points,
                world_position, R_world_body, successful_estimate);

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ///////////////////////////////////// UPDATING VARIABLES FOR THE NEXT ITERATION ///////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            prev_left_img   = curr_left_img.clone();
            prev_right_img  = curr_right_img.clone();
            prev_time       = curr_time;

            prev_left_keypoints_after_stereo_match.clear();
            prev_right_keypoints_after_stereo_match.clear();
            prev_left_keypoints_after_stereo_match = curr_left_keypoints_after_stereo_match;
            prev_right_keypoints_after_stereo_match = curr_right_keypoints_after_stereo_match;

            prev_left_descr_after_stereo_match = curr_left_descr_after_stereo_match.clone();
            prev_right_descr_after_stereo_match = curr_left_descr_after_stereo_match.clone();

            previous_t_prevCam_currCam  = t_prevCam_currCam.clone();
            previous_R_prevCam_currCam  = R_prevCam_currCam.clone();
            previous_deltaT             = deltaT;

            ROS_WARN(" ###################################################################################################################### ");
            ROS_WARN(" #################################################### ITERATION ENDED ################################################# ");
            ROS_WARN(" ######################################################################################################################\n ");
        }
    }
    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void stereo_output_computation(geometry_msgs::Vector3Stamped estimated_rpy, geometry_msgs::Vector3Stamped estimated_position_NED, 
                      geometry_msgs::Vector3Stamped estimated_linear_velocity_body, geometry_msgs::Vector3Stamped estimated_angular_velocity_body, 
                      Mat R_world_currCam, Mat R_body_cam, ros::Duration deltaT, Mat R_prevCam_currCam, Mat t_prevCam_currCam,
                      Mat world_points, Mat good_currCam_points, Mat world_position, Mat R_world_body, std_msgs::Bool successful_estimate)
{
    estimated_rpy.header.stamp.sec                      = ros::Time::now().toSec();
    estimated_position_NED.header.stamp.sec             = ros::Time::now().toSec();
    estimated_linear_velocity_body.header.stamp.sec     = ros::Time::now().toSec();
    estimated_angular_velocity_body.header.stamp.sec    = ros::Time::now().toSec();

    Mat linear_velocity_camera                          = (Mat1d(3, 1) << t_prevCam_currCam.at<double>(0,0) / deltaT.toSec(), t_prevCam_currCam.at<double>(1,0) / deltaT.toSec(), t_prevCam_currCam.at<double>(2,0) / deltaT.toSec());
    Mat linear_velocity_body                            = linear_velocity_camera.t() * R_body_cam.t();

    estimated_linear_velocity_body.vector.x             = linear_velocity_body.at<double>(0,0);
    estimated_linear_velocity_body.vector.y             = linear_velocity_body.at<double>(0,1);
    estimated_linear_velocity_body.vector.z             = linear_velocity_body.at<double>(0,2);

    estimated_position_NED.vector.x                     = world_position.at<double>(0,0);
    estimated_position_NED.vector.y                     = world_position.at<double>(0,1);
    estimated_position_NED.vector.z                     = world_position.at<double>(0,2);
    
    estimated_rpy.vector                                = from_rotation_matrix_to_euler_angles(R_world_body);
    estimated_angular_velocity_body.vector              = compute_angular_velocity(estimated_rpy.vector, R_prevCam_currCam, deltaT.toSec(), R_body_cam);


    pcl::PointCloud<pcl::PointXYZ> camera_cloud;           // CLOUD OBJECT
    sensor_msgs::PointCloud2 wp_camera_cloud;              // PC2 OBJECT
    if(!good_currCam_points.empty())
    {
        camera_cloud.points.resize(good_currCam_points.cols);
        for(int i = 0; i < good_currCam_points.cols; i++)
        {
            camera_cloud.points[i].x = good_currCam_points.at<double>(0,i);
            camera_cloud.points[i].y = good_currCam_points.at<double>(1,i);
            camera_cloud.points[i].z = good_currCam_points.at<double>(2,i);
        }
    }
    pcl::toROSMsg(camera_cloud, wp_camera_cloud);
    wp_camera_cloud.header.frame_id = "zeno/cameraleft_link_optical"; // TO USE OCTOMAP


    pcl::PointCloud<pcl::PointXYZ> world_cloud;            // CLOUD OBJECT
    sensor_msgs::PointCloud2 wp_world_cloud;               // PC2 OBJECT
    if(!world_points.empty())
    {
        world_cloud.points.resize(world_points.cols);
        for(int i = 0; i < world_points.cols; i++)
        {
            world_cloud.points[i].x = world_points.at<double>(0,i);
            world_cloud.points[i].y = world_points.at<double>(1,i);
            world_cloud.points[i].z = world_points.at<double>(2,i);
        }
    }
    pcl::toROSMsg(world_cloud, wp_world_cloud);
    wp_world_cloud.header.frame_id = "world_points";


    // PUBLISHING
    pub_estimated_rpy.publish(estimated_rpy);
    pub_estimated_pos.publish(estimated_position_NED);
    pub_estimated_linear_vel.publish(estimated_linear_velocity_body);
    pub_estimated_angular_vel.publish(estimated_angular_velocity_body);
    pub_validity.publish(successful_estimate);
    pub_camera_pointcloud.publish(wp_camera_cloud);
    pub_world_pointcloud.publish(wp_world_cloud);

}




