#ifndef TRACKBALL_H
#define TRACKBALL_H

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include <opencv/cv.h>
#include <opencv2/nonfree/features2d.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/asio/io_service.hpp>
#include <boost/format.hpp>
#include "dynamixel_controllers/SetSpeed.h"

using namespace cv;

#define FOV_WIDTH 	1.094
#define FOV_HEIGHT 	1.094

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
    void headBallTrack(bool track);
    void loop();

private:
    void image_cb (const sensor_msgs::Image& rgbImage);
    void depth_cb (const sensor_msgs::Image& depthImage);
    void sobelImage(cv_bridge::CvImage &cv_ptr);
    void detectSurfFeatures(cv_bridge::CvImage &cv_ptr);
    void drawCirclesHSV(cv_bridge::CvImage &cv_ptr);
    void drawCircles(cv_bridge::CvImage &cv_ptr, Mat frame_gray);
    void getHoughCircle(Mat &frame_gray, vector<Vec3f> &circles);
    void trackObject(cv_bridge::CvImage &cv_ptr);

    void adjustSpeedForDisplacement(float x, float y);
    void SetPanSpeed (float speed);
    void SetTiltSpeed (float speed);

    ros::NodeHandle *getNodeHandle();

private:
    ros::NodeHandle *nodeHandle;
    int hueLower_;
    int hueUpper_;
    int saturationLower_;
    int saturationUpper_;
    int valueLower_;
    int valueUpper_;
    bool headBallTrack_;
    Point ball_centre_;
    Mat ballImage_;

    ros::Subscriber image_sub_;
    ros::Subscriber depth_sub_;
    ros::Publisher image_pub_;
    ros::Publisher image_hsv_range_pub_;
    ros::Publisher hough_circle_pub_;
    ros::Publisher ball_centre_pub_;
    ros::Publisher head_target_pub_;
    ros::ServiceClient pan_speed_client_;
    ros::ServiceClient tilt_speed_client_;
    dynamixel_controllers::SetSpeed speed_srv_;

    boost::asio::io_service service;
};

#endif // TRACKBALL_H
