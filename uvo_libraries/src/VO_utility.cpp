#include "uvo_libraries/VO_utility.h"


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////// FUNCTIONS //////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


Mat compute_projection_matrix(const Mat& R, const Mat& t, const Mat& cameraIntrinsic)
{
    Mat R_t;
    hconcat(R, t, R_t);

    return cameraIntrinsic * R_t; 
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


double compute_scale_factor(float distance, const Mat& world_points)
{
    if (world_points.empty() || world_points.rows < 3)
    {
        ROS_ERROR("Invalid world_points matrix");
        return 0.0; 
    }

    vector<double> z_vector;
    world_points.row(2).copyTo(z_vector);

    double Zmedian;
    Zmedian = compute_median(z_vector);

    return distance / Zmedian;  
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


Mat convert_3Dpoints_camera(const Mat& points_to_convert, const Mat& R_to_from, const Mat& t_to_from)
{
    Mat converted_points;
    
    for(int i = 0; i < points_to_convert.rows; i++)
    {
        Mat points_to_convert_row = points_to_convert.row(i);
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


Mat convert_from_homogeneous_coords(const Mat& points4d)
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


void detect_features(Mat img, vector<KeyPoint> &keypoints, Mat &descriptors)
{
    if(FEATURE_DETECTOR == "AKAZE")
    {
        ROS_INFO("*** USING AKAZE DETECTOR ***");
        Ptr<AKAZE> detector = AKAZE::create();
        detector->detectAndCompute(img, noArray(), keypoints, descriptors);
    }

    else if(FEATURE_DETECTOR == "ORB")
    {
        ROS_INFO("*** USING ORB DETECTOR ***");
        Ptr<ORB> detector = ORB::create(10000, 1.2, 8, 31, 0, 2, ORB::HARRIS_SCORE, 31, 10);
        detector->detectAndCompute(img, noArray(), keypoints, descriptors);
    }

    else if(FEATURE_DETECTOR == "SIFT")
    {
        ROS_INFO("*** USING SIFT DETECTOR ***");
        Ptr<SIFT> detector = SIFT::create(10000, 3, 0.03, 10, 1.6);
        detector->detectAndCompute(img, noArray(), keypoints, descriptors);
    }

    else if(FEATURE_DETECTOR == "SURF")
    {
        ROS_INFO("*** USING SURF DETECTOR ***");
        Ptr<SURF> detector = SURF::create(SURF_MIN_HESSIAN, SURF_OCTAVES_NUMBER, SURF_OCTAVES_LAYERS, SURF_EXTENDED, SURF_UPRIGHT);
        detector->detectAndCompute(img, noArray(), keypoints, descriptors);
    }
    else
    {
        ROS_ERROR("WRONG SELECTION OF DETECTOR");
    }

    ROS_INFO("FEATURES EXTRACTED - CURR. IMAGE: %lu", keypoints.size());
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


void estimate_relative_pose(vector<Point2f> keypoints1_conv, vector<Point2f> keypoints2_conv, Mat cameraMatrix, Mat &R_currCam_prevCam, Mat &t_currCam_prevCam, vector<Point2f> &inliers1, vector<Point2f> &inliers2, vector<DMatch> &inlier_matches, bool &success)
{
    bool estimate_completed = false;
    bool switch_method      = false;
    success                 = false;

    while(!estimate_completed)
    {
        Mat mask, essential_matrix, homography_matrix;
        int valid_inliers = 0;

        if(use_essential)
        {
            essential_matrix = findEssentialMat(keypoints1_conv, keypoints2_conv, cameraMatrix, ESSENTIAL_OUTLIER_METHOD, ESSENTIAL_CONFIDENCE, ESSENTIAL_THRESHOLD, ESSENTIAL_MAX_ITERS, mask);
            extract_inliers(keypoints1_conv, keypoints2_conv, mask, inliers1, inliers2, inlier_matches);
            recoverPose(essential_matrix, keypoints1_conv, keypoints2_conv, cameraMatrix, R_currCam_prevCam, t_currCam_prevCam, mask);
        }
        else{
            homography_matrix = findHomography(keypoints1_conv, keypoints2_conv, HOMOGRAPHY_OUTLIER_METHOD, HOMOGRAPHY_THRESHOLD, mask, HOMOGRAPHY_MAX_ITERS, HOMOGRAPHY_CONFIDENCE);
            extract_inliers(keypoints1_conv, keypoints2_conv, mask, inliers1, inliers2, inlier_matches);
            recover_pose_homography(homography_matrix, keypoints1_conv, keypoints2_conv, cameraMatrix, R_currCam_prevCam, t_currCam_prevCam);
        }

        valid_inliers = countNonZero(mask);
        double valid_point_fraction = (double)valid_inliers / mask.rows;

        if (valid_point_fraction >= VPF_THRESHOLD && valid_inliers >= MIN_NUM_INLIERS)
        {
            success = true;
            estimate_completed = true;
        }
        else
        {
            if (switch_method)
            {
                ROS_WARN("###### BOTH METHODS FAILED ######");
                break;
            }
            else
            {
                switch_method = true;
                use_essential = !use_essential;
                ROS_WARN("###### SWITCHING METHOD ######");
            }
        }
    }
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


void extract_3Dpoints_and_reprojection(vector<Point2f> keypoints1_conv, vector<Point2f> keypoints2_conv, Mat R1, Mat t1, Mat R2, Mat t2, Mat cameraMatrix1, Mat cameraMatrix2, Mat points4D, Mat& very_good_cam1_points, Mat& very_good_indexes, vector<double>& reprojection_errors_vector)
{   
    Mat points3D, cam1_points, good_cam1_points, good_indexes;
    vector<double> reproject_mean_vector;

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
        
        for(int i = 0; i < cam1_points.rows; i++)
        {
            Mat cam1_points_row = cam1_points.row(i);

            double reproject_mean_ith = (reproject_cam1[i] + reproject_cam2[i]) / 2.0;
            if((reproject_mean_ith < REPROJECTION_TOLERANCE) && (cam1_points_row.at<double>(2) > 0))
            {
                good_indexes.push_back(i);
                good_cam1_points.push_back(cam1_points_row);
                reproject_mean_vector.push_back(reproject_mean_ith);
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
                reprojection_errors_vector.push_back(reproject_mean_vector[i]);
            }
        }
    }    
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


void extract_inliers(const vector<Point2f>& keypoints1_conv, const vector<Point2f>& keypoints2_conv, const Mat& mask, vector<Point2f>& inliers1, vector<Point2f>& inliers2, vector<DMatch>& inlier_matches)
{
    int numInliers = countNonZero(mask);

    inliers1.clear();
    inliers2.clear();
    inlier_matches.clear();

    inliers1.reserve(numInliers);
    inliers2.reserve(numInliers);
    inlier_matches.reserve(numInliers);

    for (int i = 0; i < mask.rows; i++) {
        if (mask.at<uchar>(i, 0) != 0) {
            inliers1.push_back(keypoints1_conv[i]);
            inliers2.push_back(keypoints2_conv[i]);

            DMatch match;
            match.queryIdx = static_cast<int>(inliers1.size()) - 1;
            match.trainIdx = static_cast<int>(inliers2.size()) - 1;
            inlier_matches.push_back(match);
        }
    }
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


Mat get_image(const Mat& current_img, const Mat& cameraMatrix, const Mat& distortionCoeff, const Mat& newCamMatrix)
{
    int original_width  = current_img.cols;
    int original_height = current_img.rows;
    double ratio        = (double)original_width / (double)DESIRED_WIDTH;
    int desired_height  = (int)(original_height / ratio);

    if (original_width == DESIRED_WIDTH && original_height == desired_height)
    {
        Mat gray_img;
        cvtColor(current_img, gray_img, COLOR_RGB2GRAY);

        Mat undistorted_image;
        undistort(gray_img, undistorted_image, cameraMatrix, distortionCoeff, newCamMatrix);

        if (CLAHE_CORRECTION)
        {
            Ptr<CLAHE> clahe = createCLAHE();
            clahe->setClipLimit(CLIP_LIMIT);
            clahe->apply(undistorted_image, undistorted_image);
        }

        return undistorted_image;
    }

    Mat resized_img;
    resize(current_img, resized_img, Size(DESIRED_WIDTH, desired_height), 0, 0, INTER_AREA);

    Mat gray_img;
    cvtColor(resized_img, gray_img, COLOR_RGB2GRAY);

    Mat undistorted_image;
    undistort(gray_img, undistorted_image, cameraMatrix, distortionCoeff, newCamMatrix);

    if (CLAHE_CORRECTION)
    {
        Ptr<CLAHE> clahe = createCLAHE();
        clahe->setClipLimit(CLIP_LIMIT);
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
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


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

    // READING DISTORTION COEFFICIENTS
    node_obj.getParam("/" + CAMERA_NAME + "/distortion_coefficient_left/radial/k1", k1_left);
    node_obj.getParam("/" + CAMERA_NAME + "/distortion_coefficient_left/radial/k2", k2_left);
    node_obj.getParam("/" + CAMERA_NAME + "/distortion_coefficient_left/tangential/p1", p1_left);
    node_obj.getParam("/" + CAMERA_NAME + "/distortion_coefficient_left/tangential/p2", p2_left);
    node_obj.getParam("/" + CAMERA_NAME + "/distortion_coefficient_right/radial/k1", k1_right);
    node_obj.getParam("/" + CAMERA_NAME + "/distortion_coefficient_right/radial/k2", k2_right);
    node_obj.getParam("/" + CAMERA_NAME + "/distortion_coefficient_right/tangential/p1", p1_right);
    node_obj.getParam("/" + CAMERA_NAME + "/distortion_coefficient_right/tangential/p2", p2_right);

    // READING ROTATION MATRIX FOR LEFT CAMERA
    node_obj.getParam("/" + CAMERA_NAME + "/left_camera_rotation_matrix/data", data);
    node_obj.getParam("/" + CAMERA_NAME + "/left_camera_rotation_matrix/rows", rows);
    node_obj.getParam("/" + CAMERA_NAME + "/left_camera_rotation_matrix/cols", cols);
    cv::Mat(rows, cols, CV_64F, data.data()).copyTo(R_left);

    // READING TRANSLATION VECTOR FOR LEFT CAMERA
    node_obj.getParam("/" + CAMERA_NAME + "/left_camera_translation_vector/data", data);
    node_obj.getParam("/" + CAMERA_NAME + "/left_camera_translation_vector/rows", rows);
    node_obj.getParam("/" + CAMERA_NAME + "/left_camera_translation_vector/cols", cols);
    cv::Mat(rows, cols, CV_64F, data.data()).copyTo(t_left);

    // READING ROTATION MATRIX FOR RIGHT CAMERA
    node_obj.getParam("/" + CAMERA_NAME + "/right_camera_rotation_matrix/data", data);
    node_obj.getParam("/" + CAMERA_NAME + "/right_camera_rotation_matrix/rows", rows);
    node_obj.getParam("/" + CAMERA_NAME + "/right_camera_rotation_matrix/cols", cols);
    cv::Mat(rows, cols, CV_64F, data.data()).copyTo(R_right);

    // READING TRANSLATION VECTOR FOR RIGHT CAMERA
    node_obj.getParam("/" + CAMERA_NAME + "/right_camera_translation_vector/data", data);
    node_obj.getParam("/" + CAMERA_NAME + "/right_camera_translation_vector/rows", rows);
    node_obj.getParam("/" + CAMERA_NAME + "/right_camera_translation_vector/cols", cols);
    cv::Mat(rows, cols, CV_64F, data.data()).copyTo(t_right);
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


void get_VO_parameters(ros::NodeHandle node_obj)
{
    // NODE PARAMETERS
    node_obj.getParam("/node_freq", NODE_FREQ);

    // PREPROCESSING PARAMETERS
    node_obj.getParam("/preprocessing/desired_width", DESIRED_WIDTH);
    node_obj.getParam("/preprocessing/clahe", CLAHE_CORRECTION);
    node_obj.getParam("/preprocessing/clip_limit", CLIP_LIMIT);

    // VO PARAMETERS
    node_obj.getParam("/vo_params/distance", DISTANCE);
    node_obj.getParam("/vo_params/feature_detector", FEATURE_DETECTOR);
    node_obj.getParam("/vo_params/lowe_ratio_test", LOWE_RATIO_THRESHOLD);
    node_obj.getParam("/vo_params/essential_outlier_method", ESSENTIAL_OUTLIER_METHOD);
    node_obj.getParam("/vo_params/essential_max_iters", ESSENTIAL_MAX_ITERS);
    node_obj.getParam("/vo_params/essential_confidence", ESSENTIAL_CONFIDENCE);
    node_obj.getParam("/vo_params/essential_threshold", ESSENTIAL_THRESHOLD);
    node_obj.getParam("/vo_params/homography_outlier_method", HOMOGRAPHY_OUTLIER_METHOD);
    node_obj.getParam("/vo_params/homography_max_iters", HOMOGRAPHY_MAX_ITERS);
    node_obj.getParam("/vo_params/homography_confidence", HOMOGRAPHY_CONFIDENCE);
    node_obj.getParam("/vo_params/homography_threshold", HOMOGRAPHY_THRESHOLD);
    node_obj.getParam("/vo_params/homography_distance", HOMOGRAPHY_DISTANCE);
    node_obj.getParam("/vo_params/valid_point_fraction", VPF_THRESHOLD);
    node_obj.getParam("/vo_params/reprojection_threshold", REPROJECTION_TOLERANCE);
    node_obj.getParam("/vo_params/min_num_features", MIN_NUM_FEATURES);
    node_obj.getParam("/vo_params/min_num_3Dpoints", MIN_NUM_3DPOINTS);
    node_obj.getParam("/vo_params/min_num_inliers", MIN_NUM_INLIERS);

    // PnP PARAMETERS
    node_obj.getParam("/vo_params/iterations_count", ITERATIONS_COUNT);
    node_obj.getParam("/vo_params/reprojection_error", REPROJECTION_ERROR_THRESHOLD);
    node_obj.getParam("/vo_params/confidence", CONFIDENCE);
    node_obj.getParam("/vo_params/use_extrinsic_guess", USE_EXTRINSIC_GUESS);
    node_obj.getParam("/vo_params/pnp_method_flag", PNP_METHOD_FLAG);
    
    // VISUALIZATION PARAMETERS
    node_obj.getParam("/visualization/fps", FPS);
    node_obj.getParam("/visualization/show_match", SHOW_MATCHES);

    // SURF PARAMETERS
    node_obj.getParam("/surf_params/min_hessian", SURF_MIN_HESSIAN);
    node_obj.getParam("/surf_params/n_octaves", SURF_OCTAVES_NUMBER);
    node_obj.getParam("/surf_params/n_octave_layers", SURF_OCTAVES_LAYERS);
    node_obj.getParam("/surf_params/extended", SURF_EXTENDED);
    node_obj.getParam("/surf_params/upright", SURF_UPRIGHT);
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


void match_features(vector<KeyPoint> keypoints1, vector<KeyPoint> keypoints2, Mat descriptors1, Mat descriptors2, vector<DMatch> &matches)
{
    vector< vector< DMatch > > knn_matches;
    float ratio_thresh = LOWE_RATIO_THRESHOLD;

    if(FEATURE_DETECTOR == "AKAZE" || FEATURE_DETECTOR == "ORB")
    {
        BFMatcher matcher(NORM_HAMMING, false);
        matcher.knnMatch(descriptors1, descriptors2, knn_matches, 2);
    }
    else if(FEATURE_DETECTOR == "SURF" || FEATURE_DETECTOR == "SIFT")
    {
        BFMatcher matcher(NORM_L2, false);
        matcher.knnMatch(descriptors1, descriptors2, knn_matches, 2);
    }

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


// ############################################################ //
// ############################################################ //
// ############################################################ //


void match_features(vector<KeyPoint> keypoints1, vector<KeyPoint> keypoints2, Mat descriptors1, Mat descriptors2, vector<DMatch> &matches, vector<Point2f> &keypoints1_conv, vector<Point2f> &keypoints2_conv)
{
    vector< vector< DMatch > > knn_matches;
    float ratio_thresh = LOWE_RATIO_THRESHOLD;

    BFMatcher matcher(NORM_L2, false);
    matcher.knnMatch(descriptors1, descriptors2, knn_matches, 2);

    ROS_INFO("MATCHES BEFORE LOWE'S RATIO: %lu", knn_matches.size());

    size_t i = 0;
    for (i; i < knn_matches.size(); i++) 
    {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {
            matches.push_back(knn_matches[i][0]);
            keypoints1_conv.push_back(keypoints1[knn_matches[i][0].queryIdx].pt); // QUERY IS FOR KEYPOINTS1
            keypoints2_conv.push_back(keypoints2[knn_matches[i][0].trainIdx].pt); // TRAIN IS FOR KEYPOINTS2
        }
    }

    ROS_INFO("MATCHES AFTER LOWE'S RATIO: %lu", matches.size());
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


int recover_pose_homography(Mat H, vector<Point2f> inliers1, vector<Point2f> inliers2, Mat cameraMatrix, Mat& R, Mat& t)
{
    // DECOMPOSE HOMOGRAPHY MATRIX
    vector<Mat> R_candidates, t_candidates;
    int solutions = decomposeHomographyMat(H, cameraMatrix, R_candidates, t_candidates, noArray());

    Mat proj_std = compute_projection_matrix(Mat::eye(3, 3, CV_64F), Mat::zeros(3, 1, CV_64F), cameraMatrix);

    int best_solution_idx = -1;
    int max_good_points = 0;
    
    for(int i = 0; i < solutions; i++)
    {
        Mat triangulated_points;
        triangulatePoints(proj_std, compute_projection_matrix(R_candidates[i], t_candidates[i], cameraMatrix), inliers1, inliers2, triangulated_points);
        triangulated_points = convert_from_homogeneous_coords(triangulated_points);

        int good_points = 0;
        for(int j = 0; j < triangulated_points.cols; j++)
        {
            Mat point = triangulated_points.col(j);
            if(point.at<double>(2) > 0 && point.at<double>(2) < HOMOGRAPHY_DISTANCE)
            {
                good_points++;
            }
        }

        if (good_points > max_good_points)
        {
            best_solution_idx = i;
            max_good_points = good_points;
        }
    }

    if (best_solution_idx != -1)
    {
        // Normalize the translation vector
        double translation_norm = norm(t_candidates[best_solution_idx]);
        R = R_candidates[best_solution_idx];
        t = t_candidates[best_solution_idx] / translation_norm;
    }

    return max_good_points;
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


vector<double> reproject_errors(const Mat& world_points, const Mat& R, const Mat& t, const Mat& cameraMatrix, const vector<Point2f>& img_points)
{
    // REPROJECT 3D POINTS IN THE IMAGE PLANE
    Mat reprojected_points;
    projectPoints(world_points, R, t, cameraMatrix, Mat(), reprojected_points); 

    // COMPUTE REPROJECTION ERROR
    vector<double> reproject_err(img_points.size());
    
    const double* reprojected_data = reprojected_points.ptr<double>();
    
    for (size_t i = 0; i < img_points.size(); i++)
    {
        double dx = img_points[i].x - reprojected_data[i * 2];
        double dy = img_points[i].y - reprojected_data[i * 2 + 1];
        reproject_err[i] = sqrt(dx * dx + dy * dy);
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

    // cout << "original_width" << original_width << endl << endl;
    // cout << "original_height" << original_height << endl << endl;

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


void select_desired_descriptors(const Mat& descriptors, Mat& descriptors_desired, const Mat& indexes)
{
    descriptors_desired.create(indexes.rows, descriptors.cols, descriptors.type());
    
    for (int i = 0; i < indexes.rows; i++)
    {
        int idx = indexes.at<int>(i);
        if (idx >= 0 && idx < descriptors.rows)
        {
            Mat descriptor_row = descriptors.row(idx);
            descriptor_row.copyTo(descriptors_desired.row(i));
        }
    }
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


void select_desired_keypoints(const vector<KeyPoint>& keypoints, vector<KeyPoint>& keypoints_desired, const Mat& indexes)
{
    keypoints_desired.reserve(indexes.rows);  
    const int* index_data = indexes.ptr<int>();  
    
    for (int i = 0; i < indexes.rows; i++)
    {
        int idx = index_data[i];  
        if (idx >= 0 && idx < keypoints.size())  
        {
            keypoints_desired.push_back(keypoints[idx]);
        }
    }
}


// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //


bool select_estimation_method(const vector<Point2f>& keypoints1_conv, const vector<Point2f>& keypoints2_conv)
{
    int n_kP = keypoints1_conv.size();
    vector<double> pixelTrasl(n_kP);

    for (int i = 0; i < n_kP; i++)
    {
        double dx = keypoints1_conv[i].x - keypoints2_conv[i].x;
        double dy = keypoints1_conv[i].y - keypoints2_conv[i].y;
        pixelTrasl[i] = sqrt(dx * dx + dy * dy);
    }

    double median_pixel_trasl = compute_median(pixelTrasl);

    if (median_pixel_trasl < DISTANCE)
    {
        ROS_INFO("BASELINE IS TOO LOW. USING HOMOGRAPHY!");
        return false;
    }
    else
    {
        return true;
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
