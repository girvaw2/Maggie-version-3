#include "arm_test_gui/trackball.h"

TrackBall::TrackBall(int hueLower, int hueUpper, int saturationLower, int saturationUpper, int valueLower, int valueUpper) :
    hueLower_(hueLower),
    hueUpper_(hueUpper),
    saturationLower_(saturationLower),
    saturationUpper_(saturationUpper),
    valueLower_(valueLower),
    valueUpper_(valueUpper)
{
    nodeHandle = (ros::NodeHandle *)0;
}

void TrackBall::setHueLowerValue(int value) { hueLower_ = value; }
void TrackBall::setHueUpperValue(int value) { hueUpper_ = value; }
void TrackBall::setSaturationLowerValue(int value) { saturationLower_ = value; }
void TrackBall::setSaturationUpperValue(int value) { saturationUpper_ = value; }
void TrackBall::setValueLowerValue(int value) { valueLower_ = value; }
void TrackBall::setValueUpperValue(int value) { valueUpper_ = value; }

void
TrackBall::image_cb (const sensor_msgs::Image& rgbImage)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(rgbImage, sensor_msgs::image_encodings::BGR8);

        //detectSurfFeatures(*cv_ptr);

        //drawCircles(*cv_ptr);

//        std::cout << "image width = " << cv_ptr->image.size().width << "image height = " << cv_ptr->image.size().height << std::endl;

        sobelImage(*cv_ptr);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void
TrackBall::depth_cb (const sensor_msgs::Image& depthImage)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(depthImage, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    float z = cv_ptr->image.at<float>(ball_centre_);
    if (std::isnan<float>(z) == false)
    {
           std::cout << boost::format("depth_cb %1%") %  z << std::endl;

           float x = z * FOV_WIDTH * (ball_centre_.x - 320) / 640;
           float y = (z * FOV_HEIGHT * (ball_centre_.y - 240) / 480);

           geometry_msgs::PointStamped point_out;
           point_out.header.frame_id = "head_link";
           point_out.point.x = z;
           point_out.point.y = -x;
           point_out.point.z = -y;
           ball_centre_pub_.publish(point_out);
    }
}

void
TrackBall::sobelImage(cv_bridge::CvImage &cv_ptr)
{
    Mat frame_bgr, frame_gray;
    Mat grad;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;

    GaussianBlur( cv_ptr.image, frame_bgr, Size(3,3), 0, 0, BORDER_DEFAULT );

    /// Convert it to gray
    cvtColor( frame_bgr, frame_gray, CV_BGR2GRAY );


    /// Generate grad_x and grad_y
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;

    /// Gradient X
    //Scharr( frame_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
    Sobel( frame_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_x, abs_grad_x );

    /// Gradient Y
    //Scharr( frame_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
    Sobel( frame_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_y, abs_grad_y );

    /// Total Gradient (approximate)
    addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

    cv_bridge::CvImage cv_ptr2;
    cv_ptr2.header = cv_ptr.header;
    cv_ptr2.encoding = sensor_msgs::image_encodings::MONO8;
    cv_ptr2.image = grad;
    image_pub_.publish(cv_ptr2.toImageMsg());

    drawCircles(cv_ptr, grad);
}

void TrackBall::detectSurfFeatures(cv_bridge::CvImage &cv_ptr)
{
    Mat frame_gray;
    cvtColor( cv_ptr.image, frame_gray, CV_BGR2GRAY );

    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 400;

    SurfFeatureDetector detector( minHessian );

    std::vector<KeyPoint> keypoints_object, keypoints_scene;

    detector.detect( ballImage_, keypoints_object );
    detector.detect( frame_gray, keypoints_scene );

    //-- Step 2: Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;

    Mat descriptors_object, descriptors_scene;

    extractor.compute( ballImage_, keypoints_object, descriptors_object );
    extractor.compute( frame_gray, keypoints_scene, descriptors_scene );

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    matcher.match( descriptors_object, descriptors_scene, matches );

    double max_dist = 0; double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_object.rows; i++ )
    { double dist = matches[i].distance;
      if( dist < min_dist ) min_dist = dist;
      if( dist > max_dist ) max_dist = dist;
    }

    printf("-- Max dist : %f \n", max_dist );
    printf("-- Min dist : %f \n", min_dist );

    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    std::vector< DMatch > good_matches;

    for( int i = 0; i < descriptors_object.rows; i++ )
    { if( matches[i].distance < 3*min_dist )
       { good_matches.push_back( matches[i]); }
    }

    Mat img_matches;
    drawMatches( ballImage_, keypoints_object, frame_gray, keypoints_scene,
                 good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    //-- Localize the object
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;

    for( int i = 0; i < good_matches.size(); i++ )
    {
      //-- Get the keypoints from the good matches
      obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
      scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }

    Mat H = findHomography( obj, scene, CV_RANSAC );

    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( ballImage_.cols, 0 );
    obj_corners[2] = cvPoint( ballImage_.cols, ballImage_.rows ); obj_corners[3] = cvPoint( 0, ballImage_.rows );
    std::vector<Point2f> scene_corners(4);

    perspectiveTransform( obj_corners, scene_corners, H);

    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    line( img_matches, scene_corners[0] + Point2f( ballImage_.cols, 0), scene_corners[1] + Point2f( ballImage_.cols, 0), Scalar(0, 255, 0), 4 );
    line( img_matches, scene_corners[1] + Point2f( ballImage_.cols, 0), scene_corners[2] + Point2f( ballImage_.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[2] + Point2f( ballImage_.cols, 0), scene_corners[3] + Point2f( ballImage_.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[3] + Point2f( ballImage_.cols, 0), scene_corners[0] + Point2f( ballImage_.cols, 0), Scalar( 0, 255, 0), 4 );

            cv_bridge::CvImage cv_ptr2;
            cv_ptr2.header = cv_ptr.header;
            cv_ptr2.encoding = sensor_msgs::image_encodings::BGR8;
            cv_ptr2.image = img_matches;
            image_pub_.publish(cv_ptr2.toImageMsg());


}

void TrackBall::drawCirclesHSV(cv_bridge::CvImage &cv_ptr)
{
    Mat frame_hsv;
    Mat frame_hsv_range;
    cvtColor( cv_ptr.image, frame_hsv, CV_BGR2HSV );

    inRange(frame_hsv, cv::Scalar(hueLower_,saturationLower_,valueLower_), cv::Scalar(hueUpper_,saturationUpper_, valueUpper_), frame_hsv_range);

    // remember, frame_hsv_range is already in grayscale!
    drawCircles(cv_ptr, frame_hsv_range);

    cv_bridge::CvImage cv_ptr2;

    cv_ptr2.header = cv_ptr.header;
    cv_ptr2.encoding = sensor_msgs::image_encodings::MONO8;
    cv_ptr2.image = frame_hsv_range;
    image_hsv_range_pub_.publish(cv_ptr2.toImageMsg());
}


void TrackBall::drawCircles(cv_bridge::CvImage &cv_ptr, Mat frame_gray)
{
    vector<Vec3f> circles;
    getHoughCircle(frame_gray, circles);

    /// Draw the circles detected
    for( size_t i = 0; i < circles.size(); i++ )
    {
        ball_centre_.x = cvRound(circles[i][0]);
        ball_centre_.y = cvRound(circles[i][1]);
        //ball_centre_(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        circle( cv_ptr.image, ball_centre_, 3, Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        circle( cv_ptr.image, ball_centre_, radius, Scalar(0,0,255), 3, 8, 0 );
    }

    if (circles.size())
    {
        hough_circle_pub_.publish(cv_ptr);
    }

}

void TrackBall::getHoughCircle(Mat &frame_gray, vector<Vec3f> &circles)
{
    GaussianBlur( frame_gray, frame_gray, Size(9, 9), 2, 2 );

    /// Apply the Hough Transform to find the circles
    HoughCircles( frame_gray, circles, CV_HOUGH_GRADIENT, 1, 1000, 70, 30);
}

void TrackBall::trackObject(cv_bridge::CvImage &cv_ptr)
{

}


void TrackBall::loop()
{
    service.run();

    ballImage_ = cv::imread("../res/ball.jpg", CV_LOAD_IMAGE_GRAYSCALE);

    std::string image_topic = "/camera/rgb/image_color";
    image_sub_ = getNodeHandle()->subscribe (image_topic, 1, &TrackBall::image_cb, this);

    std::string depth_topic = "/camera/depth_registered/image";
    depth_sub_ = getNodeHandle()->subscribe (depth_topic, 1, &TrackBall::depth_cb, this);

    image_pub_ = getNodeHandle()->advertise<sensor_msgs::Image> ("ball_image", 30);
    image_hsv_range_pub_ = getNodeHandle()->advertise<sensor_msgs::Image> ("hsv_range_image", 30);
    hough_circle_pub_ = getNodeHandle()->advertise<sensor_msgs::Image> ("hough_circle_image", 30);

    ball_centre_pub_ = getNodeHandle()->advertise<geometry_msgs::PointStamped> ("ball_centre", 30);

    std::string r_it = getNodeHandle()->resolveName (image_topic);
    ROS_INFO_STREAM("Listening for incoming data on topic " << r_it );

    r_it = getNodeHandle()->resolveName (depth_topic);
    ROS_INFO_STREAM("Listening for incoming data on topic " << r_it );

    ros::Rate r(10);
    while(true)
    {
        ros::spinOnce();
        r.sleep();
    }

    return;
}

ros::NodeHandle *TrackBall::getNodeHandle()
{
    if (nodeHandle == (ros::NodeHandle *)0)
        nodeHandle = new ros::NodeHandle();

    return nodeHandle;
}
