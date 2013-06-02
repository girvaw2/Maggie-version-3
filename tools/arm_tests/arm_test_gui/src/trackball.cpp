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

        drawCircles(*cv_ptr);
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

    float z = cv_ptr->image.at<float>(center_);
    if (std::isnan<float>(z) == false)
    {
           std::cout << boost::format("depth_cb %1%") %  z << std::endl;
    }
}

void TrackBall::drawCircles(cv_bridge::CvImage &cv_ptr)
{
    Mat frame_hsv;
    Mat frame_hsv_range;
    cvtColor( cv_ptr.image, frame_hsv, CV_BGR2HSV );

    inRange(frame_hsv, cv::Scalar(hueLower_,saturationLower_,valueLower_), cv::Scalar(hueUpper_,saturationUpper_, valueUpper_), frame_hsv_range);

    cv_bridge::CvImage cv_ptr2;

    cv_ptr2.header = cv_ptr.header;
    cv_ptr2.encoding = sensor_msgs::image_encodings::MONO8;
    cv_ptr2.image = frame_hsv_range;
    image_hsv_range_pub_.publish(cv_ptr2.toImageMsg());

    vector<Vec3f> circles;
    getHoughCircle(frame_hsv_range, circles); // remember, frame_hsv_range is already in grayscale!

    /// Draw the circles detected
    for( size_t i = 0; i < circles.size(); i++ )
    {
        center_.x = cvRound(circles[i][0]);
        center_.y = cvRound(circles[i][1]);
        //center_(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        circle( cv_ptr.image, center_, 3, Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        circle( cv_ptr.image, center_, radius, Scalar(0,0,255), 3, 8, 0 );
    }

    if (circles.size())
        image_pub_.publish(cv_ptr.toImageMsg());
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

    std::string image_topic = "/camera/rgb/image_color";
    image_sub_ = getNodeHandle()->subscribe (image_topic, 1, &TrackBall::image_cb, this);

    std::string depth_topic = "/camera/depth_registered/image";
    depth_sub_ = getNodeHandle()->subscribe (depth_topic, 1, &TrackBall::depth_cb, this);

    image_pub_ = getNodeHandle()->advertise<sensor_msgs::Image> ("ball_image", 30);
    image_hsv_range_pub_ = getNodeHandle()->advertise<sensor_msgs::Image> ("hsv_range_image", 30);

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
