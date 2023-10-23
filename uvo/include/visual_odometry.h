// ######################################################################################################################################################## //
// ######################################################################################################################################################## //
//                                                                                                                                                          //
//                           							    This is the VISUAL_ODOMETRY class    									                        //                                                                 
//                                                                                                                                                          //
// -------------------------------------------------------------------------------------------------------------------------------------------------------- //
//                                                                                                                                                          //
//                            Author:  Simone Tani            Email: simone.tani@phd.unipi.it          Date:  25/09/2023                                    //
//                            Author:  Francesco Ruscio       Email: francesco.ruscio@phd.unipi.it     Date:  25/09/2023                                    //
//                                                                                                                                                          //
// ######################################################################################################################################################## //
// ######################################################################################################################################################## //


#ifndef VISUAL_ODOMETRY
#define VISUAL_ODOMETRY


// ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| //
// ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| //
// ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| //


#include <uvo_libraries/VO_utility.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


// ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| //
// ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| //
// ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| //


class visual_odometry_node
{
	// #################################################################################################################### //
	// #################################################################################################################### //
	// #################################################################################################################### //

	private:

		ros::NodeHandle node_obj;

		std_msgs::Header img_header;

		bool first_img 				= false;
		bool new_img_available 	    = false;
		bool vo_initialized         = false;

		string CAMERA_NAME;
		geometry_msgs::Vector3Stamped estimated_linear_velocity_camera;
		std_msgs::Bool successful_estimate;
		double range = 1.0;
		Mat camera_img, camera_left, camera_right;
		ros::Publisher pub_estimated_linear_vel, pub_validity;

        ///////////////////////////////////////////////////////////////////
        //////////////////////////    MONO VO    //////////////////////////
        ///////////////////////////////////////////////////////////////////

        void mono_output_computation(ros::Duration deltaT, const Mat& R_currCam_prevCam, const Mat& t_currCam_prevCam,
                      double SF, const std_msgs::Bool& successful_estimate);

        void mono_VO(ros::Rate loop_rate);

        void mono_imgs_callback(const sensor_msgs::CompressedImage::ConstPtr& msg)
        {
            camera_img          = from_ros_to_cv_image(msg);
            img_header          = msg->header;
            first_img           = true;
            new_img_available   = true; 
        }

        void range_callback(const sensor_msgs::Range::ConstPtr& msg)
        {
            range               = msg->range;
        }

        ///////////////////////////////////////////////////////////////////
        /////////////////////////    STEREO VO    /////////////////////////
        ///////////////////////////////////////////////////////////////////

		void stereo_output_computation(ros::Duration deltaT, const Mat& t_prevCam_currCam, const std_msgs::Bool& successful_estimate);

		void stereo_VO(ros::Rate loop_rate);

		void stereo_imgs_callback(const sensor_msgs::CompressedImage::ConstPtr& left_image, const sensor_msgs::CompressedImage::ConstPtr& right_image)
		{
		    camera_left           = from_ros_to_cv_image(left_image);
		    camera_right          = from_ros_to_cv_image(right_image);
		    img_header            = left_image->header;
		    first_img             = true;
		    new_img_available     = true;
		}

	// #################################################################################################################### //
	// #################################################################################################################### //
	// #################################################################################################################### //

	public:

		// THE CLASS CONSTRUCTOR
        visual_odometry_node()
	    {
	    	ROS_WARN(" ###################################################################################### ");
	    	ROS_WARN(" ################## BUILDING THE OBJECT FOR THE VISUAL ODOMETRY TASK ################## ");
	    	ROS_WARN(" ###################################################################################### \n");
	    }

		// THE MAIN FUNCTION
	    void visual_odometry_workflow(string VO_NODE);
};


// ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| //
// ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| //
// ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| //


// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //


void visual_odometry_node::mono_output_computation(ros::Duration deltaT, const Mat& R_currCam_prevCam, const Mat& t_currCam_prevCam,
                      double SF, const std_msgs::Bool& successful_estimate)
{
    estimated_linear_velocity_camera.header.stamp.sec = ros::Time::now().toSec();

    Mat linear_velocity_camera = -SF * R_currCam_prevCam.t() * t_currCam_prevCam / deltaT.toSec();
    const double* lv_data = linear_velocity_camera.ptr<double>();

    estimated_linear_velocity_camera.vector.x = lv_data[0];
    estimated_linear_velocity_camera.vector.y = lv_data[1];
    estimated_linear_velocity_camera.vector.z = lv_data[2];

    pub_estimated_linear_vel.publish(estimated_linear_velocity_camera);
    pub_validity.publish(successful_estimate);
}


// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //


void visual_odometry_node::stereo_output_computation(ros::Duration deltaT, const Mat& t_prevCam_currCam, const std_msgs::Bool& successful_estimate)
{
    estimated_linear_velocity_camera.header.stamp.sec           = ros::Time::now().toSec();

    Mat linear_velocity_camera                                  = (Mat1d(3, 1) << t_prevCam_currCam.at<double>(0,0) / deltaT.toSec(), t_prevCam_currCam.at<double>(1,0) / deltaT.toSec(), t_prevCam_currCam.at<double>(2,0) / deltaT.toSec());
    estimated_linear_velocity_camera.vector.x                   = linear_velocity_camera.at<double>(0,0);
    estimated_linear_velocity_camera.vector.y                   = linear_velocity_camera.at<double>(0,1);
    estimated_linear_velocity_camera.vector.z                   = linear_velocity_camera.at<double>(0,2);
    
    pub_estimated_linear_vel.publish(estimated_linear_velocity_camera);
    pub_validity.publish(successful_estimate);    
}


// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //


void visual_odometry_node::mono_VO(ros::Rate loop_rate)
{
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////// WAITING FOR THE FIRST IMAGE  /////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    while(!first_img && ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_WARN(" ################################################################################################################################### ");
    ROS_WARN(" ############################################# STARTING MONOCULAR VISUAL ODOMETRY NODE ############################################# ");
    ROS_WARN(" ###################################################################################################################################\n ");

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////// LOADING CAMERA PARAMETERS FROM YAML FILE ///////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    Mat distortion_coefficients = (Mat1d(1, 4) << k1, k2, p1, p2);
    Mat camera_matrix = (Mat1d(3, 3) << fx, 0, ccx, 0, fy, ccy, 0, 0, 1);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////// INITIALIZATION OF VARIABLES FOR THE MAIN LOOP ///////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // INITIALIZATION OF RELATIVE ORIENTATION AND RELATIVE TRANSLATION
    Mat R_currCam_prevCam = Mat::eye(3, 3, CV_64F); 
    Mat t_currCam_prevCam = Mat::zeros(3, 1, CV_64F);

    // PREVIOUS PROJECTION MATRIX
    Mat R_prev = Mat::eye(3, 3, CV_64F);
    Mat t_prev = Mat::zeros(3, 1, CV_64F);

    // SUCCESS VARIABLES
    bool success; 

    // SCALE FACTOR VARIABLE
    double SF = 1.0;                     
    
    // IMGS & PCL VARIABLES
    Mat prev_img, curr_img;
    vector<KeyPoint> prev_keypoints;
    Mat prev_descriptors;

    // TIME VARIABLES
    ros::Time prev_time, curr_time;
    ros::Duration deltaT;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////// READING THE FIRST IMAGE /////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    Mat new_camera_matrix;
    resize_camera_matrix(camera_img, camera_matrix, distortion_coefficients, new_camera_matrix);

    Mat prev_projection_matrix  = compute_projection_matrix(R_prev, t_prev, new_camera_matrix);
    Mat curr_projection_matrix  = prev_projection_matrix.clone();

    while(!vo_initialized && ros::ok())
    {
        ros::spinOnce();    
        loop_rate.sleep();

        if(new_img_available)
        {
            prev_time                   = img_header.stamp;
            prev_img                    = get_image(camera_img, camera_matrix, distortion_coefficients, new_camera_matrix);
            new_img_available           = false;

            detect_features(prev_img, prev_keypoints, prev_descriptors);

            if(prev_keypoints.size() >= MIN_NUM_FEATURES)
            {
                vo_initialized = true;
            }
        }
    }
    
    while(vo_initialized && ros::ok())
    {
        ros::spinOnce();    
        loop_rate.sleep();

        if(new_img_available)
        {
        
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////// PROCESSING THE CURRENT IMAGE //////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            curr_time           = img_header.stamp;
            curr_img            = get_image(camera_img, camera_matrix, distortion_coefficients, new_camera_matrix);

            new_img_available   = false;
            deltaT              = curr_time - prev_time;

            vector<KeyPoint> curr_keypoints;
            vector<Point2f> prev_keypoints_conv, curr_keypoints_conv, prev_inliers, curr_inliers;
            vector<DMatch> matches, inlier_matches;
            Mat curr_descriptors;        

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////// FEATURE CORRESPONDENCES /////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
            detect_features(curr_img, curr_keypoints, curr_descriptors);

            if(curr_keypoints.size() < MIN_NUM_FEATURES)
            {
                ROS_ERROR("NUMBER OF DETECTED FEATURES IS TOO LOW. SKIP IMAGE!");
                prev_img                    = curr_img.clone();
                prev_keypoints              = curr_keypoints;
                prev_descriptors            = curr_descriptors.clone();
                prev_time                   = curr_time;
                continue;
            }
            else{
                
                match_features(prev_keypoints, curr_keypoints, prev_descriptors, curr_descriptors, matches, prev_keypoints_conv, curr_keypoints_conv, prev_img, curr_img);
            }


            if(SHOW_MATCHES)
            {
                Mat img_matches = show_matches(prev_keypoints, curr_keypoints, matches, prev_img, curr_img);
                imshow("MATCHES", img_matches);
                waitKey(FPS);
            }

            // CHECK THE NUMBER OF FEATURE CORRESPONDECES
            if(matches.size() < MIN_NUM_FEATURES)
            {
                ROS_ERROR("NUMBER OF FEATURES IS TOO LOW. SKIP IMAGE!");
                prev_img                    = curr_img.clone();
                prev_keypoints              = curr_keypoints;
                prev_descriptors            = curr_descriptors.clone();
                prev_time                   = curr_time;
                continue;
            }     

            // CHECK CONSECUTIVE IMAGES BASELINE
            if(!select_estimation_method(prev_keypoints_conv, curr_keypoints_conv))
            {
                use_essential = false;
            }
            else
            {
                use_essential = true;
            }

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////// EXTRACTING RELATIVE POSE //////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            estimate_relative_pose(prev_keypoints_conv, curr_keypoints_conv, new_camera_matrix, R_currCam_prevCam, t_currCam_prevCam, prev_inliers, curr_inliers, inlier_matches, success);

            if(SHOW_MATCHES)
            {
                vector<KeyPoint> prev_kpoint_inliers, curr_kpoint_inliers;
                KeyPoint::convert(prev_inliers, prev_kpoint_inliers);
                KeyPoint::convert(curr_inliers, curr_kpoint_inliers);
                Mat img_matches_inliers = show_matches(prev_kpoint_inliers, curr_kpoint_inliers, inlier_matches, prev_img, curr_img);
                imshow("INLIERS", img_matches_inliers);
                waitKey(FPS);
            }

            if(success)
            {
                successful_estimate.data = 1;              
            }

            else
            {
                ROS_ERROR("FAIL DURING RECOVER POSE - ASSUMING CONSTANT MOTION");
                successful_estimate.data = 0;              
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
                    ROS_ERROR("NOT ENOUGH TRIANGULATED POINTS - ASSUMING CONSTANT MOTION");
                    successful_estimate.data = 0;
                }
                else
                {
                    good_currCam_points = convert_3Dpoints_camera(good_prevCam_points, R_currCam_prevCam, t_currCam_prevCam);
                    if(!good_currCam_points.empty())
                    {
                        SF = compute_scale_factor(range, good_currCam_points);
                    }
                    else
                    {
                        ROS_ERROR("NOT ENOUGH TRIANGULATED POINTS - ASSUMING CONSTANT MOTION");
                        successful_estimate.data = 0;
                    }   
                }
            }

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////// CREATING AND PUBLISHING OUTPUT ////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            mono_output_computation(deltaT, R_currCam_prevCam, t_currCam_prevCam, SF, successful_estimate);

            ROS_WARN(" ######################################################################### ");
            ROS_WARN(" ########################### ITERATION ENDED ############################# ");
            ROS_WARN(" #########################################################################\n ");

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////// UPDATING FOR THE NEXT ITERATION ////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            prev_img            = curr_img.clone();
            prev_keypoints      = curr_keypoints;
            prev_descriptors    = curr_descriptors.clone();
            prev_time           = curr_time;
        }
    }
}


// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //


void visual_odometry_node::stereo_VO(ros::Rate loop_rate)
{
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////// WAITING FOR THE FIRST SENSORS MSGS ///////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    while(!first_img && ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_WARN(" #################################################################################################################################### ");
    ROS_WARN(" ############################################### STARTING STEREO VISUAL ODOMETRY NODE ############################################### ");
    ROS_WARN(" ####################################################################################################################################\n ");

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////// LOADING CAMERA PARAMETERS FROM YAML FILE ///////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    Mat camera_matrix_left          = (Mat1d(3, 3) << fx_left, 0, ccx_left, 0, fy_left, ccy_left, 0, 0, 1);     
    Mat camera_matrix_right         = (Mat1d(3, 3) << fx_right, 0, ccx_right, 0, fy_right, ccy_right, 0, 0, 1);
    Mat distortion_left             = (Mat1d(1, 4) << k1_left, k2_left, p1_left, p2_left);                             
    Mat distortion_right            = (Mat1d(1, 4) << k1_right, k2_right, p1_right, p2_right);

    //  INITIALIZATION OF RELATIVE ORIENTATION AND RELATIVE TRANSLATION
    Mat R_currCam_prevCam_Vec       = Mat::zeros(3, 1, CV_64FC1); 
    Mat R_currCam_prevCam           = Mat::eye(3, 3, CV_64FC1);           
    Mat R_prevCam_currCam           = Mat::eye(3, 3, CV_64FC1);           
    Mat t_currCam_prevCam           = Mat::zeros(3, 1, CV_64FC1);            
    Mat t_prevCam_currCam           = Mat::zeros(3, 1, CV_64FC1);
    Mat distCoeffs                  = Mat::zeros(4, 1, CV_64FC1);  // PnP distortion coefficients                  

    // TIME VARIABLES
    ros::Time prev_time, curr_time;
    ros::Duration deltaT;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////// READING THE FIRST TWO IMAGES /////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    Mat new_camera_matrix_left, new_camera_matrix_right;
    resize_camera_matrix(camera_left, camera_matrix_left, distortion_left, new_camera_matrix_left);
    resize_camera_matrix(camera_right, camera_matrix_right, distortion_right, new_camera_matrix_right);

    Mat R_eye, t_zeros, R_using_left_as_world, t_using_left_as_world, R_using_right_as_world, t_using_right_as_world;
    R_eye                           = Mat::eye(3, 3, CV_64FC1); 
    t_zeros                         = Mat::zeros(3, 1, CV_64FC1);
    R_using_left_as_world           = R_right;
    t_using_left_as_world           = t_right; 
    R_using_right_as_world          = R_right.t();
    t_using_right_as_world          = - R_right.t() * t_right;

    Mat P_eye_using_left_as_world, P_eye_using_right_as_world, P_using_left_as_world, P_using_right_as_world;
    P_eye_using_left_as_world       = compute_projection_matrix(R_eye, t_zeros, new_camera_matrix_left);
    P_eye_using_right_as_world      = compute_projection_matrix(R_eye, t_zeros, new_camera_matrix_right);
    P_using_left_as_world           = compute_projection_matrix(R_right, t_right, new_camera_matrix_right);
    P_using_right_as_world          = compute_projection_matrix(R_right.t(), -R_right.t()*t_right, new_camera_matrix_left);

    Mat prev_left_img, prev_right_img;
    vector<KeyPoint> prev_left_keypoints, prev_right_keypoints;
    Mat prev_left_descr, prev_right_descr;
    vector<DMatch> results_match_prev;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////// PROCESSING THE FIRST COUPLE OF IMAGES /////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    while(!vo_initialized && ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        if (new_img_available)
        {
            prev_time			= img_header.stamp;
            prev_left_img		= get_image(camera_left, camera_matrix_left, distortion_left, new_camera_matrix_left);
            prev_right_img		= get_image(camera_right, camera_matrix_right, distortion_right, new_camera_matrix_right);
            new_img_available	= false;

            detect_features(prev_left_img, prev_left_keypoints, prev_left_descr);
            detect_features(prev_right_img, prev_right_keypoints, prev_right_descr);

            if(prev_left_keypoints.size() >= MIN_NUM_FEATURES and prev_right_keypoints.size() >= MIN_NUM_FEATURES)
            {
                match_features(prev_left_keypoints, prev_right_keypoints, prev_left_descr, prev_right_descr, results_match_prev);
                
                if(SHOW_MATCHES)
                {
                    Mat img_matches_stereo = show_matches(prev_left_keypoints, prev_right_keypoints, results_match_prev, prev_left_img, prev_right_img);
                    imshow("MATCHING STEREO PAIR", img_matches_stereo);
                    waitKey(FPS);
                }

                if(results_match_prev.size() > MIN_NUM_FEATURES)
                {
                    vo_initialized = true;
                }
            }
        }
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

    while(vo_initialized && ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        if (new_img_available)
        {
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////// PROCESSING THE CURRENT COUPLE OF IMAGES ////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            ROS_WARN(" < ------------------- PROCESSING STEREO PAIR ------------------- > ");

            curr_time 			= img_header.stamp;
            deltaT 				= curr_time - prev_time;

            Mat curr_left_img 	= get_image(camera_left, camera_matrix_left, distortion_left, new_camera_matrix_left);
            Mat curr_right_img 	= get_image(camera_right, camera_matrix_right, distortion_right, new_camera_matrix_right);
            new_img_available 	= false;

            vector<KeyPoint> curr_left_keypoints, curr_right_keypoints, curr_img_to_use_keypoints;
            Mat curr_left_descr, curr_right_descr;
            detect_features(curr_left_img, curr_left_keypoints, curr_left_descr);
            detect_features(curr_right_img, curr_right_keypoints, curr_right_descr);

            vector<DMatch> results_match_curr;
            Mat curr_left_descr_after_stereo_match, curr_right_descr_after_stereo_match;
            vector<KeyPoint> curr_left_keypoints_after_stereo_match, curr_right_keypoints_after_stereo_match;
            Mat good_currCam_points, world_points;

            if(curr_left_keypoints.size() >= MIN_NUM_FEATURES and curr_right_keypoints.size() >= MIN_NUM_FEATURES)
            {
                match_features(curr_left_keypoints, curr_right_keypoints, curr_left_descr, curr_right_descr, results_match_curr);
            
                if(SHOW_MATCHES)
                {
                    Mat img_matches_stereo = show_matches(curr_left_keypoints, curr_right_keypoints, results_match_curr, curr_left_img, curr_right_img);
                    imshow("MATCHING STEREO PAIR", img_matches_stereo);
                    waitKey(FPS);
                }

                if(results_match_curr.size() > MIN_NUM_FEATURES)
                {
                    Mat curr_stereo_match_indexPairs_left, curr_stereo_match_indexPairs_right;
                    for (size_t i = 0; i < results_match_curr.size(); i++)
                    {
                        curr_stereo_match_indexPairs_left.push_back(results_match_curr[i].queryIdx);
                        curr_stereo_match_indexPairs_right.push_back(results_match_curr[i].trainIdx);
                    }

                    select_desired_descriptors(curr_left_descr, curr_left_descr_after_stereo_match, curr_stereo_match_indexPairs_left);
                    select_desired_descriptors(curr_right_descr, curr_right_descr_after_stereo_match, curr_stereo_match_indexPairs_right);
                    select_desired_keypoints(curr_left_keypoints, curr_left_keypoints_after_stereo_match, curr_stereo_match_indexPairs_left);
                    select_desired_keypoints(curr_right_keypoints, curr_right_keypoints_after_stereo_match, curr_stereo_match_indexPairs_right);

                    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    ////////////////////////////// PROCESSING PREVIOUS IMAGES AND ONE OF THE CURRENT FRAMES ///////////////////////////////
                    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                    ROS_WARN(" < ------------------- TRIANGULAR MATCHING ------------------- > ");

                    vector< DMatch > results_match_prev_curr;
                    Mat camera_matrix_to_use_for_pnp;

                    curr_img_to_use_keypoints = curr_left_keypoints;
                    camera_matrix_to_use_for_pnp = new_camera_matrix_left.clone();
                    match_features(prev_left_keypoints_after_stereo_match, curr_img_to_use_keypoints, prev_left_descr_after_stereo_match, curr_left_descr, results_match_prev_curr);
               
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

                    vector<KeyPoint> prev_left_keypoints_after_triangular_match, prev_right_keypoints_after_triangular_match, curr_img_to_use_keypoints_after_triangular_match;
                    //
                    select_desired_keypoints(prev_left_keypoints_after_stereo_match, prev_left_keypoints_after_triangular_match, prev_curr_match_indexPairs_left);
                    select_desired_keypoints(prev_right_keypoints_after_stereo_match, prev_right_keypoints_after_triangular_match, prev_curr_match_indexPairs_left);
                    //
                    select_desired_keypoints(curr_img_to_use_keypoints, curr_img_to_use_keypoints_after_triangular_match, prev_curr_match_indexPairs_right);

                    vector<Point2f> prev_left_keypoints_after_triangular_match_converted, prev_right_keypoints_after_triangular_match_converted;
                    KeyPoint::convert(prev_left_keypoints_after_triangular_match, prev_left_keypoints_after_triangular_match_converted, vector<int>());
                    KeyPoint::convert(prev_right_keypoints_after_triangular_match, prev_right_keypoints_after_triangular_match_converted, vector<int>());

                    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    ////////////////////////////// TRIANGULATION OF 2D POINTS OF THE PREVIOUS IMAGES //////////////////////////////////////
                    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    
                    Mat good_prevCam_points, points4D, good_indexes;
                    vector<double> reprojection_errors_vector;

                    if(results_match_prev_curr.size() > MIN_NUM_FEATURES)
                    {

                        // TRIANGULATION IS DONE BETWEEN PREVIOUS LEFT CAMERA AND PREVIOUS RIGHT CAMERA, THUS:
                        // AT THIS LEVEL, 3D POINTS ARE EXPRESSED IN THE FRAME ASSOCIATED TO THE PREVIOUS LEFT CAMERA
                        triangulatePoints(P_eye_using_left_as_world, P_using_left_as_world, prev_left_keypoints_after_triangular_match_converted, prev_right_keypoints_after_triangular_match_converted, points4D);
                        extract_3Dpoints(prev_left_keypoints_after_triangular_match_converted, prev_right_keypoints_after_triangular_match_converted, R_eye, t_zeros, R_using_left_as_world, t_using_left_as_world, new_camera_matrix_left, new_camera_matrix_right, points4D, good_prevCam_points, good_indexes);

                        if(good_prevCam_points.rows > MIN_NUM_3DPOINTS)
                        {

                            vector<KeyPoint> good_curr_img_keypoints_after_triangular_match;
                            select_desired_keypoints(curr_img_to_use_keypoints_after_triangular_match, good_curr_img_keypoints_after_triangular_match, good_indexes);
                            vector<Point2f> curr_img_keypoints_after_triangular_match_converted;
                            KeyPoint::convert(good_curr_img_keypoints_after_triangular_match, curr_img_keypoints_after_triangular_match_converted);

                            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                            ////////////////////////////// EXTRACTION OF ROTATION AND TRANSLATION - PNP METHOD ////////////////////////////////////
                            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                       
                            Mat inliers_idx;
                            solvePnPRansac(good_prevCam_points, curr_img_keypoints_after_triangular_match_converted, camera_matrix_to_use_for_pnp, distCoeffs, R_currCam_prevCam_Vec, t_currCam_prevCam,
                                            USE_EXTRINSIC_GUESS, ITERATIONS_COUNT, REPROJECTION_ERROR_THRESHOLD, CONFIDENCE, inliers_idx, PNP_METHOD_FLAG);         

                            vector<DMatch> inlier_matches   = results_match_prev_curr;
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
                                ROS_ERROR("NUMBER OF INLIERS AFTER PNP IS TOO LOW - ASSUMING CONSTANT MOTION");
                                successful_estimate.data    = 0;         
                            }
                            else
                            {
                                ROS_WARN("PNP SOLVED - COMPUTING RESULTS");
                                Rodrigues(R_currCam_prevCam_Vec, R_currCam_prevCam);
                                R_prevCam_currCam           = R_currCam_prevCam.t();                                       
                                t_prevCam_currCam           = - R_currCam_prevCam.t() * t_currCam_prevCam;
                                successful_estimate.data    = 1;

                                Mat good_prevCam_points_filtered;
                                select_desired_descriptors(good_prevCam_points, good_prevCam_points_filtered, inliers_idx);
                                good_currCam_points         = convert_3Dpoints_camera(good_prevCam_points_filtered, R_currCam_prevCam, t_currCam_prevCam); 

                                if(!good_currCam_points.empty())
                                {
                                    vector<double> z_vector;
                                    good_currCam_points.row(2).copyTo(z_vector);
                                }
                            }
                        }
                        else
                        {
                            ROS_ERROR("NOT ENOUGH TRIANGULATED POINTS - ASSUMING CONSTANT MOTION");
                            successful_estimate.data = 0;
                        }
                    }
                    else
                    {
                        ROS_ERROR("NUMBER OF FEATURES AFTER TRIANGULAR MATCHING IS TOO LOW - ASSUMING CONSTANT MOTION");
                        successful_estimate.data = 0;
                    }
                }
                else
                {
                    ROS_ERROR("NUMBER OF FEATURES MATCHED BETWEEN CURRENT IMAGES IS TOO LOW - ASSUMING CONSTANT MOTION");
                    successful_estimate.data = 0;
                }
            }
            else
            {
                ROS_ERROR("NUMBER OF DETECTED FEATURES IS TOO LOW - ASSUMING CONSTANT MOTION!");
                successful_estimate.data = 0;
            }

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////// CREATING AND PUBLISHING OUTPUT ////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            stereo_output_computation(deltaT, t_prevCam_currCam, successful_estimate);

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ///////////////////////////////////// UPDATING VARIABLES FOR THE NEXT ITERATION ///////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            prev_left_img                           = curr_left_img.clone();
            prev_right_img                          = curr_right_img.clone();
            prev_time                               = curr_time;

            prev_left_keypoints_after_stereo_match.clear();
            prev_right_keypoints_after_stereo_match.clear();
            prev_left_keypoints_after_stereo_match  = curr_left_keypoints_after_stereo_match;
            prev_right_keypoints_after_stereo_match = curr_right_keypoints_after_stereo_match;

            prev_left_descr_after_stereo_match      = curr_left_descr_after_stereo_match.clone();
            prev_right_descr_after_stereo_match     = curr_right_descr_after_stereo_match.clone();

            ROS_WARN(" ###################################################################################################################### ");
            ROS_WARN(" #################################################### ITERATION ENDED ################################################# ");
            ROS_WARN(" ######################################################################################################################\n ");
        
        }
    }
}


// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //


void visual_odometry_node::visual_odometry_workflow(string VO_NODE)
{
	ROS_WARN(" ###################################################################################### ");
	ROS_WARN(" ####################### WITHIN THE MAIN FUNCTION OF THE CLASS  ####################### ");
	ROS_WARN(" ###################################################################################### \n");

    // GET PARAMETERS
    node_obj.getParam("/camera_name", CAMERA_NAME);
    get_VO_parameters(node_obj);

    ros::Rate loop_rate(NODE_FREQ);

    if(VO_NODE == "stereo")
    {
        pub_estimated_linear_vel        = node_obj.advertise<geometry_msgs::Vector3Stamped>("/estimated_linear_vel_stereo_UVO", 10);
        pub_validity                    = node_obj.advertise<std_msgs::Bool>("/validity_stereo_UVO", 10);

        message_filters::Subscriber<sensor_msgs::CompressedImage> sub_cameraSX(node_obj, "/image_left/compressed", 1);
        message_filters::Subscriber<sensor_msgs::CompressedImage> sub_cameraDX(node_obj, "/image_right/compressed", 1);

        // APPROXIMATE TIME SYNCRHRONIZER
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_;
        sync_.reset(new Sync(MySyncPolicy(10), sub_cameraSX, sub_cameraDX));
        sync_->registerCallback(boost::bind(&visual_odometry_node::stereo_imgs_callback, this, _1, _2));

        get_stereo_camera_parameters(node_obj, CAMERA_NAME);
        stereo_VO(loop_rate);
    }
    else if(VO_NODE == "mono")
    {
        pub_estimated_linear_vel        = node_obj.advertise<geometry_msgs::Vector3Stamped>("/estimated_linear_vel_mono_UVO", 10);
        pub_validity                    = node_obj.advertise<std_msgs::Bool>("/validity_mono_UVO", 10);
        
        ros::Subscriber sub_camera_imgs = node_obj.subscribe("/image/compressed", 1, &visual_odometry_node::mono_imgs_callback, this);
        ros::Subscriber sub_range       = node_obj.subscribe("/range", 1, &visual_odometry_node::range_callback, this);

        get_mono_camera_parameters(node_obj, CAMERA_NAME);
        mono_VO(loop_rate);
    }
    else{ROS_ERROR(" ################ WRONG SELECTION OF VISUAL ODOMETRY NODE - CHOOSE BETWEEN mono AND stereo ################");}  
}


// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //


#endif /* VISUAL_ODOMETRY */