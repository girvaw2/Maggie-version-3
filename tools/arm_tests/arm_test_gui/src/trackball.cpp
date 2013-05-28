#include "arm_test_gui/trackball.h"

TrackBall::TrackBall()
{
    nodeHandle = (ros::NodeHandle *)0;
    init();
}

void
TrackBall::image_cb (const sensor_msgs::Image& rgbImage)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(rgbImage, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // set the global image pointer
  //cv_ptr_ = cv_ptr;
}

void TrackBall::init()
{
//    /* Start the CV system and get the first v4l camera */
//    cv::cvInitSystem(0, (char **)0);
//    CvCapture *cam = cvCreateCameraCapture(0);

//    cv::VideoCapture cap(0);

//    /* Create a window to use for displaying the images */
//    cvNamedWindow("img", 0);
//    cvMoveWindow("img", 200, 200);

//    /* Display images until the user presses q */
//    while (1)
//    {
//        cvGrabFrame(cam);
//        IplImage *img = cvRetrieveFrame(cam);
//        cvShowImage("img", img);
//        if (cvWaitKey(10) == XK_q)
//            return;
//        cvReleaseImage(&img);
//    }
    //

    std::string image_topic = "/camera/rgb/image_color";
    image_sub_ = getNodeHandle()->subscribe (image_topic, 1, &TrackBall::image_cb, this);
    image_pub_ = getNodeHandle()->advertise<sensor_msgs::Image> ("ball_image", 30);

    std::string r_it = getNodeHandle()->resolveName (image_topic);
    ROS_INFO_STREAM("Listening for incoming data on topic " << r_it );

//    cv::VideoCapture cap(0); // open the default camera
//    if(!cap.isOpened())  // check if we succeeded
//        return; // -1;

//    cv::Mat edges;
//    cv::namedWindow("edges",1);
//    for(;;)
//    {
//        cv::Mat frame;
//        cap >> frame; // get a new frame from camera
//        cv::cvtColor(frame, edges, CV_BGR2GRAY);
//        cv::GaussianBlur(edges, edges, cv::Size(7,7), 1.5, 1.5);
//        cv::Canny(edges, edges, 0, 30, 3);
//        cv::imshow("edges", edges);
//        if(cv::waitKey(30) >= 0)
//            break;
//    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return; // 0;

}

ros::NodeHandle *TrackBall::getNodeHandle()
{
    if (nodeHandle == (ros::NodeHandle *)0)
        nodeHandle = new ros::NodeHandle();

    return nodeHandle;
}
