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
    std::cout << "image_cb" << std::endl;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(rgbImage, sensor_msgs::image_encodings::BGR8);

        Mat frame = cv_ptr->image;
        Mat frame_hsv;

        GaussianBlur(frame, frame, Size(3, 3), 0, 0);

        cvtColor( frame, frame_hsv, CV_BGR2HSV);

        vector<cv::Mat> v;
        split(frame_hsv, v);

        Mat frame_h = v.at(0);
        Mat frame_thresh;

        inRange(frame_h, cv::Scalar(hueLower,160,160), cv::Scalar(hueUpper,256,256), frame_thresh);

        GaussianBlur(frame_thresh, frame_thresh, Size(3, 3), 0, 0);

        cv_bridge::CvImage cv_ptr2;

        cv_ptr2.header = cv_ptr->header;
        cv_ptr2.encoding = sensor_msgs::image_encodings::MONO8;
        cv_ptr2.image = frame_thresh;

        image_pub_.publish(cv_ptr2.toImageMsg());
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void TrackBall::loop()
{
    service.run();

    std::string image_topic = "/camera/rgb/image_color";
    image_sub_ = getNodeHandle()->subscribe (image_topic, 1, &TrackBall::image_cb, this);
    image_pub_ = getNodeHandle()->advertise<sensor_msgs::Image> ("ball_image", 30);

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
