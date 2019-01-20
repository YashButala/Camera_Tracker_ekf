#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include "sensor_msgs/Imu.h"
using namespace cv;
using namespace std;

Point2f center;
float radius=0;
float velx=0,vely=0,velz=0,ax=0,ay=0,az=0;
float cx=320.5,cy=240.5,fx=0.381362,fy=0.381362;
geometry_msgs::PointStamped threeD;

void Callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  center.x=msg->point.x;
  center.y=msg->point.y;
  radius=msg->point.z;
}
void Callback2(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg)
{
  velx=-(msg->twist.twist.linear.y);
  vely=-(msg->twist.twist.linear.z);
  velz=msg->twist.twist.linear.x;
  //REVIEW THE CONVENTION
}
void Callback3(const sensor_msgs::Imu::ConstPtr& msg)
{
  ax=-(msg->linear_acceleration.y);
  ay=-(msg->linear_acceleration.z);
  az=msg->linear_acceleration.x;
  //REVIEW THE CONVENTION
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ekf");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<geometry_msgs::PointStamped>("/buoy/3d_cordinate",1000);
	//ros::Publisher pub = nh.advertise<geometry_msgs::Point>("kposition",1000);
	ros::Subscriber sub1 = nh.subscribe("/buoy/image_cordinate", 1000, Callback);
  	ros::Subscriber sub2 = nh.subscribe("/kraken/sensors/DVL", 1000, Callback2);
	ros::Subscriber sub3 = nh.subscribe("/kraken/sensors/IMU", 1000, Callback3);
        unsigned int type = CV_32F;
    	KalmanFilter KF(9, 8, 0);//x,y,z,vx,vy,vz,ax,ay,az
	Mat state(9 ,1 , type);
    	Mat meas(8, 1, type);
	meas.setTo(Scalar(0));
	
  

    //  KF.transitionMatrix = *(Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
    // Transition State Matrix A

   	cv::setIdentity(KF.transitionMatrix);
    

    // Measure Matrix H
    // [ 1 0 0 0 0 ]
    // [ 0 1 0 0 0 ]
    // [ 0 0 0 0 1 ]
	KF.measurementMatrix = cv::Mat::zeros(8, 9, type);
	KF.measurementMatrix.at<float>(21) = 1;
	KF.measurementMatrix.at<float>(31) = 1;
	KF.measurementMatrix.at<float>(41) = 1;
	KF.measurementMatrix.at<float>(51) = 1;
	KF.measurementMatrix.at<float>(61) = 1;
	KF.measurementMatrix.at<float>(71) = 1;    

    // Process Noise Covariance Matrix Q
    // [ Ex   0   0     0     0    ]
    // [ 0    Ey  0     0     0    ]
    // [ 0    0   Evx   0     0    ]
    // [ 0    0   0     Evy   0    ]
    // [ 0    0   0     0     Er   ]
	KF.processNoiseCov = (Mat_<float>(9,9) << 0.1,0,0,0,0,0,0,0,0,0,0.1,0,0,0,0,0,0,0,0,0,0.1,0,0,0,0,0,0,0,0,0,0.1,0,0,0,0,0,0,0,0,0,0.1,0,0,0,0,0,0,0,0,0,0.1,0,0,0,0,0,0,0,0,0,0.1,0,0,0,0,0,0,0,0,0,0.1,0,0,0,0,0,0,0,0,0,0.1);  

       // Measures Noise Covariance Matrix R
       cv::setIdentity(KF.measurementNoiseCov, cv::Scalar(1));  
	setIdentity(KF.errorCovPost, Scalar::all(100));

	double ticks =(double) cv::getTickCount();
	int notFoundCount=0;
	bool found=false;
	ros::Rate rate(1);
	while(ros::ok())
	{
		cout<<"Received center "<<center;
		cout<<" Recieved radius: "<<radius<<endl;
		double precTick = ticks;
                ticks = (double) cv::getTickCount();

                double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

                KF.transitionMatrix.at<float>(3) = -dT;
                KF.transitionMatrix.at<float>(6) = -0.5*dT*dT;
		KF.transitionMatrix.at<float>(13) = -dT;
		KF.transitionMatrix.at<float>(16) = -0.5*dT*dT;
		KF.transitionMatrix.at<float>(23) = -dT;
		KF.transitionMatrix.at<float>(26) = -0.5*dT*dT;	
		KF.transitionMatrix.at<float>(33) = -dT;
		KF.transitionMatrix.at<float>(43) = -dT;
		KF.transitionMatrix.at<float>(53) = -dT;	
		state = KF.predict();
		
		if (center.x==0 || center.y==0 || radius==0  )
          	{
            		notFoundCount++;
            		cout << "notFoundCount:" << notFoundCount << endl;
            		found = false;
            		
           	}
	        else
          	{
            		notFoundCount = 0;
			//Measurement matrix
			KF.measurementMatrix.at<float>(0) =(float)1/state.at<float>(2);
			KF.measurementMatrix.at<float>(2) = -state.at<float>(0)/(state.at<float>(2)*state.at<float>(2));
			KF.measurementMatrix.at<float>(10) =(float)1/state.at<float>(2);
			KF.measurementMatrix.at<float>(11) = -state.at<float>(1)/(state.at<float>(2)*state.at<float>(2));
            		meas.at<float>(0) = (center.x-cx)/fx;
            		meas.at<float>(1) = (center.y-cy)/fy;
            		meas.at<float>(2) = velx;
			meas.at<float>(3) = vely;
			meas.at<float>(4) = velz;
			meas.at<float>(3) = ax;
			meas.at<float>(3) = ay;
			meas.at<float>(3) = az;
		
			if (!found) // First detection!
            		{
				KF.statePost.at<float>(0) = 1;    
				KF.statePost.at<float>(1) = 1;    
				KF.statePost.at<float>(2) = 1;
				KF.statePost.at<float>(3) = velx;
				KF.statePost.at<float>(4) = vely;
				KF.statePost.at<float>(5) = velz;
				KF.statePost.at<float>(6) = ax;
				KF.statePost.at<float>(7) = ay;
				KF.statePost.at<float>(8) = az; 
				//Initialization things
                		found = true;
            		}
            		else 
			{
				KF.correct(meas); // Kalman Correction
				KF.temp5.at<float>(0) = meas.at<float>(0) - (state.at<float>(0)/state.at<float>(2));
				KF.temp5.at<float>(1) = meas.at<float>(1) - (state.at<float>(1)/state.at<float>(2));
				KF.temp5.at<float>(2) = meas.at<float>(2) - (state.at<float>(3));
				KF.temp5.at<float>(3) = meas.at<float>(3) - (state.at<float>(4));
				KF.temp5.at<float>(4) = meas.at<float>(4) - (state.at<float>(5));
				KF.temp5.at<float>(5) = meas.at<float>(5) - (state.at<float>(6));
				KF.temp5.at<float>(6) = meas.at<float>(6) - (state.at<float>(7));
				KF.temp5.at<float>(7) = meas.at<float>(7) - (state.at<float>(8));
				KF.statePost = KF.statePre + KF.gain * KF.temp5;
				geometry_msgs::PointStamped msg;
   				msg.point.x = KF.statePost.at<float>(0);     
				msg.point.y = KF.statePost.at<float>(1); 
				msg.point.z = KF.statePost.at<float>(2);
				msg.header.stamp = ros::Time::now() ;
				pub.publish(msg);
			}
		}
		
		
		ros::spinOnce();
		rate.sleep();
	}
       	return 0;
}
