#include "arm_test_gui/trackball.h"

TrackBall::TrackBall(int hueLower, int hueUpper, int saturationLower, int saturationUpper, int valueLower, int valueUpper) :
    hueLower(hueLower),
    hueUpper(hueUpper),
    saturationLower(saturationLower),
    saturationUpper(saturationUpper),
    valueLower(valueLower),
    valueUpper(valueUpper)
{
    nodeHandle = (ros::NodeHandle *)0;
}

void TrackBall::setHueLowerValue(int value) { hueLower = value; }
void TrackBall::setHueUpperValue(int value) { hueUpper = value; }
void TrackBall::setSaturationLowerValue(int value) { saturationLower = value; }
void TrackBall::setSaturationUpperValue(int value) { saturationUpper = value; }
void TrackBall::setValueLowerValue(int value) { valueLower = value; }
void TrackBall::setValueUpperValue(int value) { valueUpper = value; }

void
TrackBall::image_cb (const sensor_msgs::Image& rgbImage)
{
    std::cout << "image_cb " << hueUpper << " " << saturationUpper << " " << valueUpper << std::endl;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(rgbImage, sensor_msgs::image_encodings::BGR8);

        applyHSVRange(*cv_ptr);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void TrackBall::applyHSVRange(cv_bridge::CvImage &cv_ptr)
{
    Mat frame_hsv;
    Mat frame_hsv_range;
    cvtColor( cv_ptr.image, frame_hsv, CV_BGR2HSV );

    inRange(frame_hsv, cv::Scalar(hueLower,saturationLower,valueLower), cv::Scalar(hueUpper,saturationUpper, valueUpper), frame_hsv_range);

    cv_bridge::CvImage cv_ptr2;

    cv_ptr2.header = cv_ptr.header;
    cv_ptr2.encoding = sensor_msgs::image_encodings::MONO8;
    cv_ptr2.image = frame_hsv_range;
    image_hsv_range_pub_.publish(cv_ptr2.toImageMsg());

    vector<Vec3f> circles;
    houghCircle(frame_hsv_range, circles); // remember, frame_hsv_range is already in grayscale!

    /// Draw the circles detected
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        circle( cv_ptr.image, center, 3, Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        circle( cv_ptr.image, center, radius, Scalar(0,0,255), 3, 8, 0 );
    }
    if (circles.size())
        image_pub_.publish(cv_ptr.toImageMsg());
}

void TrackBall::houghCircle(Mat &frame_gray, vector<Vec3f> &circles)
{
    GaussianBlur( frame_gray, frame_gray, Size(9, 9), 2, 2 );

    /// Apply the Hough Transform to find the circles
    HoughCircles( frame_gray, circles, CV_HOUGH_GRADIENT, 1, 1000, 70, 30);
}

void TrackBall::trackObject(cv_bridge::CvImage &cv_ptr)
{
    //cv::Moments moments()

    Mat canny_output;
    int thresh = 100;
    int max_thresh = 255;
    cv::Canny(cv_ptr.image, canny_output, thresh, thresh*2, 3 );

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    /// Get the moments
    vector<Moments> mu(contours.size() );
    for( int i = 0; i < contours.size(); i++ )
    {
        mu[i] = moments( contours[i], false );
    }

    ///  Get the mass centers:
    vector<Point2f> mc( contours.size() );
    for( int i = 0; i < contours.size(); i++ )
    {
        mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
    }



    RNG rng(12345);
    /// Draw contours
    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
        circle( drawing, mc[i], 4, color, -1, 8, 0 );
    }

    for( int i = 0; i< contours.size(); i++ )
       {
         printf(" * Contour[%d] - Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n", i, mu[i].m00, contourArea(contours[i]), arcLength( contours[i], true ) );
         Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
         drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
         circle( drawing, mc[i], 4, color, -1, 8, 0 );
       }

    cv_ptr.image = drawing;
    cv_ptr.encoding = sensor_msgs::image_encodings::BGR8;
    image_pub_.publish(cv_ptr.toImageMsg());
}


void TrackBall::loop()
{
    service.run();

    std::string image_topic = "/camera/rgb/image_color";
    image_sub_ = getNodeHandle()->subscribe (image_topic, 1, &TrackBall::image_cb, this);
    image_pub_ = getNodeHandle()->advertise<sensor_msgs::Image> ("ball_image", 30);
    image_hsv_range_pub_ = getNodeHandle()->advertise<sensor_msgs::Image> ("hsv_range_image", 30);

    std::string r_it = getNodeHandle()->resolveName (image_topic);
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
