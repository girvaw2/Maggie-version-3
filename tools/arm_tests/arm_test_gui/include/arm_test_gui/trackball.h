#ifndef TRACKBALL_H
#define TRACKBALL_H

#include "ros/ros.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/asio/io_service.hpp>
#include <boost/format.hpp>

using namespace cv;

class TrackBall
{
public:
    TrackBall(int hueLower, int hueUpper, int saturationLower, int saturationUpper, int valueLower, int valueUpper);
    void doSomethingOp() const ;

    void setHueLowerValue(int value);
    void setHueUpperValue(int value);
    void setSaturationLowerValue(int value);
    void setSaturationUpperValue(int value);
    void setValueLowerValue(int value);
    void setValueUpperValue(int value);
    void loop();

private:
    void image_cb (const sensor_msgs::Image& rgbImage);
    void depth_cb (const sensor_msgs::Image& depthImage);
    void drawCircles(cv_bridge::CvImage &cv_ptr);
    void getHoughCircle(Mat &frame_gray, vector<Vec3f> &circles);
    void trackObject(cv_bridge::CvImage &cv_ptr);
    ros::NodeHandle *getNodeHandle();

private:
    ros::NodeHandle *nodeHandle;
    int hueLower_;
    int hueUpper_;
    int saturationLower_;
    int saturationUpper_;
    int valueLower_;
    int valueUpper_;
    Point center_;

    ros::Subscriber image_sub_;
    ros::Subscriber depth_sub_;
    ros::Publisher image_pub_;
    ros::Publisher image_hsv_range_pub_;

    boost::asio::io_service service;
};

#endif // TRACKBALL_H
