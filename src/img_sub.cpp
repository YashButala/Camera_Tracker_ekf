#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
using namespace cv;
using namespace std;


Mat res;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_bridge::toCvShare(msg, "bgr8")->image.copyTo(res);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "img_sub");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::namedWindow("view_threshold");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  ros::Publisher pub = nh.advertise<geometry_msgs::PointStamped>("/buoy/image_cordinate",1000);
  image_transport::Subscriber sub = it.subscribe("/KrakenSimulator/front_camera/image_raw", 1, imageCallback);
  ros::Rate rate(10);
  while(ros::ok()){
    Point2f center;
    float radius=0;
    center.x=0;
    center.y=0;
    radius=0;
    if(!res.empty())
    {
    Mat blur;
    GaussianBlur(res, blur,Size(5,5),3,3);
    Mat frmHsv;
    cvtColor(blur, frmHsv, CV_BGR2HSV);
    Mat red(res.rows,res.cols,CV_8UC1,Scalar(0));
    Mat rangeRes = cv::Mat::zeros(res.size(), CV_8UC1);
    Mat lower_red_hue_range;
    Mat upper_red_hue_range;
    int x=100;
    int y=100;
    int lhue=10;
    int hhue=160;
    inRange(frmHsv, cv::Scalar(0, x, y), cv::Scalar(lhue, 255, 255), lower_red_hue_range);
    inRange(frmHsv, cv::Scalar(hhue, x,y), cv::Scalar(179, 255, 255), upper_red_hue_range);
    //Mat red_hue_image;
    addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, rangeRes);
      cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
      cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
      vector<vector<Point> > contours;
      vector<Vec4i> hierarchy;

      findContours(rangeRes, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
      int largest_contour_index=-1;
      double a=0;
      float largest_area=-1;
      if(contours.size()!=0)
        {
          //vector<vector<Point> > contours_poly( contours.size() );
          //cout<<contours.size()<<endl;
          int i;
          for(int i = 0; i< contours.size(); i++ ) // iterate through each contour.
            {
              a=contourArea( contours[i],false);  //  Find the area of contour
              if(a>largest_area)
              {
                largest_area=a;
                largest_contour_index=i;                //Store the index of largest contour
              }
            }
            if(largest_area<1) largest_contour_index=-1;
        }
          if (largest_contour_index!=-1)
          {
            minEnclosingCircle( (Mat)contours[largest_contour_index], center, radius);
          }
          //cout<<"HERE"<<endl;
	  if (radius!=0)
	  {
            circle( res, center, radius, Scalar(255,0,0), 10, 8, 0 );
	  }
    geometry_msgs::PointStamped msg;
    msg.point.x = center.x;     msg.point.y = center.y; msg.point.z=radius;
    msg.header.stamp = ros::Time::now();
    //cout<<"Sensor center Value:"<<center;
    //cout<<" Sensor radius value:"<<radius<<endl;
    pub.publish(msg);
    cv::imshow("view", res);
    cv::imshow("view_threshold",rangeRes);
    waitKey(1);
    }
    
    rate.sleep();
    ros::spinOnce();
    
  }
  cv::destroyWindow("view");
}
