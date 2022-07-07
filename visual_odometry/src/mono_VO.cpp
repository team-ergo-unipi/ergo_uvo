// ######################################################################################################################################################## //
// ######################################################################################################################################################## //
//                                                                                                                                                          //
//                                                    The monocular visual odometry node                                                                    //                                          
//                                                                                                                                                          //
// -------------------------------------------------------------------------------------------------------------------------------------------------------- //
//                            Author:  Francesco Ruscio       Email: francesco.ruscio@phd.unipi.it     Date:  06/06/2022                                    //
//                            Author:  Simone Tani            Email: simone.tani@phd.unipi.it          Date:  06/06/2022                                    //
//                                                                                                                                                          //
// ######################################################################################################################################################## //
// ######################################################################################################################################################## //


#include <visual_odometry/VO_utility.hpp>


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// SUPPORT FUNCTION
void mono_output_computation(geometry_msgs::Vector3Stamped estimated_rpy, geometry_msgs::Vector3Stamped estimated_position_NED, 
                      geometry_msgs::Vector3Stamped estimated_linear_velocity_body, geometry_msgs::Vector3Stamped estimated_angular_velocity_body,
                      Mat R_world_currCam, Mat R_body_cam, ros::Duration deltaT, Mat R_currCam_prevCam, Mat t_currCam_prevCam,
                      double SF, Mat world_points, Mat world_position, std_msgs::Bool successful_estimate);

// MSGS
std_msgs::Header img_header;
std_msgs::Float64 altitude;

// SUPPORT VARIABLES
Mat camera_img;
bool first_img              = false;
bool new_img_available      = false;
int skipped_imgs          = 0;
vector<double> SF_vector;
vector<double> distance_vector;
string CAMERA_NAME;

// OUTPUT VARIABLES
geometry_msgs::Vector3Stamped estimated_linear_velocity_body, estimated_position_NED, estimated_rpy, estimated_angular_velocity_body;
std_msgs::Bool successful_estimate; // VARIABLE FOR CHECKING SUCCESS/FAIL

// PUBLISHERS
ros::Publisher pub_estimated_angular_vel, pub_estimated_linear_vel, pub_estimated_rpy, pub_estimated_pos, pub_validity, pub_inliers, pub_pointcloud, pub_estimation_method;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void camera_imgs_callback(const sensor_msgs::CompressedImage::ConstPtr& msg)
{   
    camera_img          = from_ros_to_cv_image(msg);
    img_header          = msg->header;
    first_img           = true;
    new_img_available   = true; 
}


void altitude_callback(const std_msgs::Float64::ConstPtr& msg)
{
    altitude.data    = msg->data;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mono_VO");
    ros::NodeHandle node_obj; 

    pub_estimated_angular_vel   = node_obj.advertise<geometry_msgs::Vector3Stamped>("/estimated_angular_vel_monoVO", 10);
    pub_estimated_linear_vel    = node_obj.advertise<geometry_msgs::Vector3Stamped>("/estimated_linear_vel_monoVO", 10);
    pub_estimated_rpy           = node_obj.advertise<geometry_msgs::Vector3Stamped>("/estimated_rpy_monoVO", 10);
    pub_estimated_pos           = node_obj.advertise<geometry_msgs::Vector3Stamped>("/estimated_pos_monoVO", 10);
    pub_validity                = node_obj.advertise<std_msgs::Bool>("/validity_monoVO", 10);
    pub_pointcloud              = node_obj.advertise<sensor_msgs::PointCloud2>("/world_points_monoVO", 10);
    pub_inliers                 = node_obj.advertise<std_msgs::Int64>("/inliers_monoVO", 10);
    pub_estimation_method       = node_obj.advertise<std_msgs::Int64>("/estimation_method_monoVO", 10);
    
    ros::Subscriber sub_camera_imgs = node_obj.subscribe("/image/compressed", 1, camera_imgs_callback);
    ros::Subscriber sub_altitude    = node_obj.subscribe("/altitude", 1, altitude_callback);

    // GET PARAMETERS
    ros::param::get("/camera_name", CAMERA_NAME);
    get_VO_parameters(node_obj);
    get_mono_camera_parameters(node_obj, CAMERA_NAME);

    ros::Rate loop_rate(NODE_FREQ);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////// WAITING FOR THE FIRST IMAGE  /////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    while(!first_img && ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_GREEN_STREAM(" ###################################################################################################################### ");
    ROS_GREEN_STREAM(" ############################################# STARTING VISUAL ODOMETRY NODE ########################################## ");
    ROS_GREEN_STREAM(" ######################################################################################################################\n ");

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////// LOADING CAMERA PARAMETERS FROM YAML FILE ///////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    Mat distortion_coefficients = (Mat1d(1, 4) << k1, k2, p1, p2);
    Mat camera_matrix    = (Mat1d(3, 3) << fx, 0, ccx, 0, fy, ccy, 0, 0, 1);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////// INITIALIZATION OF VARIABLES FOR THE MAIN LOOP ///////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    Mat R_body_cam = body_camera_matrix;  // ROTATION MATRIX FROM CAMERA FRAME TO BODY FRAME
    
    // INITIALIZATION OF GLOBAL ORIENTATION AND POSITION
    Mat R_world_body    = Mat::eye(3, 3, CV_64F);
    Mat R_world_currCam = R_world_body * R_body_cam;
    Mat world_position  = Mat::zeros(3, 1, CV_64F);
    Mat angular_vector  = Mat::zeros(3, 1, CV_64F);

    // INITIALIZATION OF RELATIVE ORIENTATION AND RELATIVE TRANSLATION
    Mat R_currCam_prevCam = Mat::eye(3, 3, CV_64F); 
    Mat t_currCam_prevCam = Mat::zeros(3, 1, CV_64F);
    Mat previous_R_currCam_prevCam = Mat::eye(3, 3, CV_64F);  
    Mat previous_t_currCam_prevCam = Mat::zeros(3, 1, CV_64F);

    // PREVIOUS PROJECTION MATRIX
    Mat R_prev = Mat::eye(3, 3, CV_64F);
    Mat t_prev = Mat::zeros(3, 1, CV_64F);

    // SUCCESS VARIABLES
    bool fail_detection = false; // IF TRUE FEATURES NUMBER IS < THRESHOLD
    bool success; 

    // SCALE FACTOR VARIABLES
    double SF = 1.0; // SCALE FACTOR default value
    SF_vector.push_back(SF);
    float distance = 1.0; // ALTITUDE default value
    
    // IMGS & PCL VARIABLES
    Mat prev_img, curr_img;
    vector<KeyPoint> prev_keypoints;
    Mat prev_descriptors;
    Mat world_points;   // WORLD POINTS IN {W} COORDINATES

    // TIME VARIABLES
    ros::Time prev_time, curr_time;
    ros::Duration deltaT, previous_deltaT;
    previous_deltaT = ros::Duration(1.0);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////// READING THE FIRST IMAGE /////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    Mat new_camera_matrix;
    resize_camera_matrix(camera_img, camera_matrix, distortion_coefficients, new_camera_matrix);

    Mat prev_projection_matrix = compute_projection_matrix(R_prev, t_prev, new_camera_matrix);
    Mat curr_projection_matrix = prev_projection_matrix.clone();

    prev_time               = img_header.stamp;
    prev_img                = get_image(camera_img, camera_matrix, distortion_coefficients, new_camera_matrix);
    new_img_available     = false;

    detect_features(prev_img, prev_keypoints, prev_descriptors);

    // Border values used to reset feature tracking
    float min_w = (1.0/4.0)*prev_img.cols;
    float max_w = (3.0/4.0)*prev_img.cols;
    float min_h = (1.0/4.0)*prev_img.rows;
    float max_h = (3.0/4.0)*prev_img.rows;
    
    while(ros::ok())
    {
        ros::spinOnce();    
        loop_rate.sleep();

        if(new_img_available)
        {
        
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////// PROCESSING THE CURRENT IMAGE //////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            curr_time               = img_header.stamp;
            curr_img                = get_image(camera_img, camera_matrix, distortion_coefficients, new_camera_matrix);

            new_img_available       = false;
            deltaT                  = curr_time - prev_time;

            vector<KeyPoint> curr_keypoints;
            vector<Point2f> prev_keypoints_conv, curr_keypoints_conv, prev_inliers, curr_inliers;
            vector<DMatch> matches, inlier_matches;
            Mat curr_descriptors;

             // GET LAST AVAILABLE ALTITUDE DATA
            distance = altitude.data;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////// FEATURE CORRESPONDENCES /////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            

            // FEATURE TRACKING
            if(FEATURES_TRACKING)
            {
                ROS_INFO("FEATURES - PREV. IMAGE: %lu", prev_keypoints.size());
                track_features(prev_img, curr_img, prev_keypoints, curr_keypoints, prev_descriptors, curr_descriptors, matches, prev_keypoints_conv, curr_keypoints_conv);
            }

            // FEATURE MATCHING
            else
            {   
                detect_features(curr_img, curr_keypoints, curr_descriptors);
                match_features(prev_keypoints, curr_keypoints, prev_descriptors, curr_descriptors, matches, prev_keypoints_conv, curr_keypoints_conv);
            }

            if(SHOW_MATCHES)
            {
                Mat img_matches = show_matches(prev_keypoints, curr_keypoints, matches, prev_img, curr_img);
                imshow("MATCHING PRE-RANSAC", img_matches);
                waitKey(FPS);
            }

            // CHECK THE NUMBER OF FEATURE CORRESPONDECES
            if(matches.size() < MIN_NUM_FEATURES)
            {
                ROS_YELLOW_STREAM("NUMBER OF FEATURES IS TOO LOW. SKIP ITERATION!");
                skipped_imgs = skipped_imgs + 1;
                if(skipped_imgs < SKIPPED_IMGS){continue;}
                else
                {
                    ROS_RED_STREAM("TOO MANY IMAGES SKIPPED!");
                    skipped_imgs              = 0;

                    // Constant Motion Model
                    t_currCam_prevCam           = previous_t_currCam_prevCam * deltaT.toSec() / previous_deltaT.toSec();
                    R_currCam_prevCam           = (deltaT.toSec() * from_vector_to_skew_matrix(from_mat_to_vector_type(R_body_cam.t() * angular_vector)) + Mat::eye(3, 3, CV_64F)) * previous_R_currCam_prevCam;

                    previous_t_currCam_prevCam  = t_currCam_prevCam.clone();
                    previous_R_currCam_prevCam  = R_currCam_prevCam.clone();
                    previous_deltaT             = deltaT;
                    prev_img                    = curr_img.clone();
                    prev_keypoints              = curr_keypoints;
                    prev_descriptors            = curr_descriptors.clone();
                    prev_time                   = curr_time;
                    continue;
                }
            }     

            // CHECK CONSECUTIVE IMAGES BASELINE
            if(!check_if_moving(prev_keypoints_conv, curr_keypoints_conv))
            {
                ROS_YELLOW_STREAM("BASELINE IS TOO LOW. SKIPPING IMAGE!");
                continue;
            } 

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////// EXTRACTING RELATIVE POSE //////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            estimate_relative_pose(prev_keypoints_conv, curr_keypoints_conv, new_camera_matrix, R_currCam_prevCam, t_currCam_prevCam, prev_inliers, curr_inliers, inlier_matches, success);

            std_msgs::Int64 inliers_number;
            inliers_number.data = curr_inliers.size();
            pub_inliers.publish(inliers_number);

            if(SHOW_MATCHES)
            {
                vector<KeyPoint> prev_kpoint_inliers, curr_kpoint_inliers;
                KeyPoint::convert(prev_keypoints_conv, prev_kpoint_inliers);
                KeyPoint::convert(curr_keypoints_conv, curr_kpoint_inliers);
                Mat img_matches_inliers = show_matches(prev_kpoint_inliers, curr_kpoint_inliers, inlier_matches, prev_img, curr_img);
                imshow("MATCHING AFTER RANSAC", img_matches_inliers);
                waitKey(FPS);
            }
            


            if(success)
            {
                successful_estimate.data    = 1;

                std_msgs::Int64 estimation_method;
                estimation_method.data = GLOBAL_ESTIMATE_METHOD;
                pub_estimation_method.publish(estimation_method);              
            }

            else
            {
                ROS_YELLOW_STREAM("FAIL DURING RECOVER POSE- ASSUMING CONSTANT MOTION");
                successful_estimate.data    = 0;

                // Constant Motion Model
                t_currCam_prevCam = previous_t_currCam_prevCam * deltaT.toSec() / previous_deltaT.toSec();
                angular_vector    = from_vector_to_mat_type(estimated_angular_velocity_body.vector);
                R_currCam_prevCam = (deltaT.toSec() * from_vector_to_skew_matrix(from_mat_to_vector_type(R_body_cam.t() * angular_vector)) + Mat::eye(3, 3, CV_64F)) * previous_R_currCam_prevCam;
                check_rotation_matrix(R_currCam_prevCam);               
            }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////// TRIANGULATION AND SCALE FACTOR ////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            Mat good_currCam_points;
            if(success)
            {
                Mat prevCamPoints4d, good_indexes, good_prevCam_points;
                curr_projection_matrix = compute_projection_matrix(R_currCam_prevCam, t_currCam_prevCam, new_camera_matrix);
                triangulatePoints(prev_projection_matrix, curr_projection_matrix, prev_inliers, curr_inliers, prevCamPoints4d);
                extract_3Dpoints(prev_inliers, curr_inliers, R_prev, t_prev, R_currCam_prevCam, t_currCam_prevCam, new_camera_matrix, new_camera_matrix, prevCamPoints4d, good_prevCam_points, good_indexes);

                if(good_prevCam_points.rows < MIN_NUM_3DPOINTS)
                {
                    ROS_YELLOW_STREAM("NOT ENOUGH TRIANGULATED POINTS - ASSUMING CONSTANT MOTION");
                    successful_estimate.data  = 0;
                    SF                        = SF_vector[SF_vector.size()-1];

                    // Constant Motion Model
                    t_currCam_prevCam = previous_t_currCam_prevCam * deltaT.toSec() / previous_deltaT.toSec();
                    angular_vector    = from_vector_to_mat_type(estimated_angular_velocity_body.vector);
                    R_currCam_prevCam = (deltaT.toSec() * from_vector_to_skew_matrix(from_mat_to_vector_type(R_body_cam.t() * angular_vector)) + Mat::eye(3, 3, CV_64F)) * previous_R_currCam_prevCam;
                    check_rotation_matrix(R_currCam_prevCam);
                }
                else
                {
                    good_currCam_points = convert_3Dpoints_camera(good_prevCam_points, R_currCam_prevCam, t_currCam_prevCam);
                    if(!good_currCam_points.empty())
                    {
                        SF = compute_scale_factor(distance, good_currCam_points);
                        SF_vector.push_back(SF);
                    }
                    else
                    {
                        ROS_YELLOW_STREAM("NOT ENOUGH TRIANGULATED POINTS - ASSUMING CONSTANT MOTION");
                        successful_estimate.data    = 0;
                        SF                          = SF_vector[SF_vector.size()-1];

                        // Constant Motion Model
                        t_currCam_prevCam = previous_t_currCam_prevCam * deltaT.toSec() / previous_deltaT.toSec();
                        angular_vector    = from_vector_to_mat_type(estimated_angular_velocity_body.vector);
                        R_currCam_prevCam = (deltaT.toSec() * from_vector_to_skew_matrix(from_mat_to_vector_type(R_body_cam.t() * angular_vector)) + Mat::eye(3, 3, CV_64F)) * previous_R_currCam_prevCam;
                        check_rotation_matrix(R_currCam_prevCam);
                    }   
                }
            }


            // UPDATE ABSOLUTE POSE
            vector<Mat> absPose = compute_absolute_pose(R_world_currCam, world_position, R_currCam_prevCam, t_currCam_prevCam, SF, good_currCam_points);
            world_position      = absPose[0];
            R_world_currCam     = absPose[1];
            if(good_currCam_points.cols >= MIN_NUM_3DPOINTS){world_points = absPose[2];}

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////// CREATING AND PUBLISHING OUTPUT ////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            R_world_body = R_world_body * R_body_cam * R_currCam_prevCam.t() * R_body_cam.t();

            mono_output_computation(estimated_rpy, estimated_position_NED, estimated_linear_velocity_body, estimated_angular_velocity_body,
                         R_world_body, R_body_cam, deltaT, R_currCam_prevCam, t_currCam_prevCam, SF, world_points, world_position, successful_estimate);

            ROS_GREEN_STREAM(" ######################################################################### ");
            ROS_GREEN_STREAM(" ########################### ITERATION ENDED ############################# ");
            ROS_GREEN_STREAM(" #########################################################################\n ");

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////// UPDATING FOR THE NEXT ITERATION ////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            if(FEATURES_TRACKING)
            {
                int n_kP = curr_keypoints_conv.size();
                vector<double> x_positions(n_kP);
                vector<double> y_positions(n_kP);

                for(int i = 0; i < n_kP; i++)
                {
                    x_positions[i] = curr_keypoints_conv[i].x;
                    y_positions[i] = curr_keypoints_conv[i].y;
                }

                double median_position_x = compute_median(x_positions);
                double median_position_y = compute_median(y_positions);          

                if(n_kP < MIN_NUM_TRACKED_FEATURES || median_position_x < min_w || median_position_x > max_w || median_position_y < min_h || median_position_y > max_h)
                {
                    ROS_WARN("NEW FEATURE DETECTION");
                    curr_keypoints.clear();
                    curr_descriptors.release();
                    detect_features(curr_img, curr_keypoints, curr_descriptors);
                }
            }


            previous_R_currCam_prevCam  = R_currCam_prevCam.clone();
            previous_t_currCam_prevCam  = t_currCam_prevCam.clone();
            previous_deltaT             = deltaT;
            prev_img                    = curr_img.clone();
            prev_keypoints              = curr_keypoints;
            prev_descriptors            = curr_descriptors.clone();
            prev_time                   = curr_time;
            
        }
    }
    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void mono_output_computation(geometry_msgs::Vector3Stamped estimated_rpy, geometry_msgs::Vector3Stamped estimated_position_NED, 
                      geometry_msgs::Vector3Stamped estimated_linear_velocity_body, geometry_msgs::Vector3Stamped estimated_angular_velocity_body,
                      Mat R_world_body, Mat R_body_cam, ros::Duration deltaT, Mat R_currCam_prevCam, Mat t_currCam_prevCam,
                      double SF, Mat world_points, Mat world_position, std_msgs::Bool successful_estimate)
{
    estimated_rpy.header.stamp.sec                      = ros::Time::now().toSec();
    estimated_position_NED.header.stamp.sec             = ros::Time::now().toSec();
    estimated_linear_velocity_body.header.stamp.sec     = ros::Time::now().toSec();
    estimated_angular_velocity_body.header.stamp.sec    = ros::Time::now().toSec();

    estimated_rpy.vector                                = from_rotation_matrix_to_euler_angles(R_world_body); 
    estimated_position_NED.vector                       = from_mat_to_vector_type(world_position);

    geometry_msgs::Twist estimated_twist                = estimate_twist(deltaT, R_currCam_prevCam, t_currCam_prevCam, SF, estimated_rpy.vector, R_body_cam);
    estimated_linear_velocity_body.vector               = estimated_twist.linear;
    estimated_angular_velocity_body.vector              = estimated_twist.angular;

    
    pcl::PointCloud<pcl::PointXYZ> cloud;           // CLOUD OBJECT
    sensor_msgs::PointCloud2 wp_cloud;              // PC2 OBJECT
    if(!world_points.empty())
    {
        cloud.points.resize(world_points.cols);
        for(int i = 0; i < world_points.cols; i++)
        {
            cloud.points[i].x = world_points.at<double>(0, i);
            cloud.points[i].y = world_points.at<double>(1, i);
            cloud.points[i].z = world_points.at<double>(2, i);
        }
    }
    pcl::toROSMsg(cloud, wp_cloud);
    wp_cloud.header.frame_id = "world_points";


    // PUBLISHING
    pub_estimated_rpy.publish(estimated_rpy);
    pub_estimated_pos.publish(estimated_position_NED);
    pub_estimated_linear_vel.publish(estimated_linear_velocity_body);
    pub_estimated_angular_vel.publish(estimated_angular_velocity_body);
    pub_validity.publish(successful_estimate);
    pub_pointcloud.publish(wp_cloud);
}






