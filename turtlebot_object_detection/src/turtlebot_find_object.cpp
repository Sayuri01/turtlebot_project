#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "find_object_2d/ObjectsStamped.h"
//#include <QTransform>

using namespace std;
using namespace cv;

class RobotVision {
    public:
    RobotVision();
    
    void spin();
    std::vector<float> data;
    string frame_id;
    ros::Time stamp;
    
    private:
       
        //image_transport::ImageTransport it_;

        void vision_cb(const sensor_msgs::ImageConstPtr& imageMsg);
        void object_cb(const std_msgs::Float32MultiArrayConstPtr& objectsMsg);
        ros::NodeHandle nh;
        ros::Subscriber sub_image;
        ros::Subscriber sub_objects;
        //ros::Publisher pub_obj;
        ros::Publisher pub_obj_stamped;
        //sensor_msgs::LaserScan LaserScan;
};


RobotVision::RobotVision() {
    
    sub_image = nh.subscribe("/camera/rgb/image_raw", 1, &RobotVision::vision_cb, this);
    sub_objects = nh.subscribe("/objects", 1, &RobotVision::object_cb, this);
    //pub_obj = nh.advertise<std_msgs::Float32MultiArray>("/obj_array", 1);
    pub_obj_stamped = nh.advertise<find_object_2d::ObjectsStamped>("/obj_array_stamped", 1);
}   


void RobotVision::spin() {
    
     ros::Rate loop(10);
    
     while(ros::ok()) {
         
        // call all waiting callbacks
        ros::spinOnce();
        // enforce a max publish rate
        loop.sleep();
        //ros::spin();
      }
}

void RobotVision::object_cb(const std_msgs::Float32MultiArrayConstPtr& objectsMsg)
{
    this->data = objectsMsg->data; 
    this->frame_id = "camera_depth_optical_frame";
    this->stamp = ros::Time(0);
}


void RobotVision::vision_cb(const sensor_msgs::ImageConstPtr& imageMsg)
{
        //const std::vector<float>& data = objectsMsg->objects.data;
        int dead = 0;
        float object_id = -1;
        //std_msgs::Float32MultiArray obj;
        find_object_2d::ObjectsStamped objStamped;
        
        objStamped.objects.data.resize(12);
        if(data.size())
        {

                cv_bridge::CvImageConstPtr cvPtr;
                    try
                    {
                        cvPtr = cv_bridge::toCvShare(imageMsg, sensor_msgs::image_encodings::BGR8);
                    }
                    
                catch(const cv_bridge::Exception& e)
                {
                    ROS_ERROR_STREAM("Failed to get the image from the message: " << e.what());
                    return;
                }
                imshow("true_image", cvPtr->image);
                
            
                // get data
               //int id = (int)data[i];
                float objectWidth = data[1];
                float objectHeight = data[2];
                cv::Point2i size(objectWidth, objectHeight);
                // Find corners OpenCV
                cv::Mat cvHomography(3, 3, CV_32F);
                cvHomography.at<float>(0,0) = data[3];
                cvHomography.at<float>(1,0) = data[4];
                cvHomography.at<float>(2,0) = data[5];
                cvHomography.at<float>(0,1) = data[6];
                cvHomography.at<float>(1,1) = data[7];
                cvHomography.at<float>(2,1) = data[8];
                cvHomography.at<float>(0,2) = data[9];
                cvHomography.at<float>(1,2) = data[10];
                cvHomography.at<float>(2,2) = data[11];
                std::vector<cv::Point2f> inPts, outPts;
                inPts.push_back(cv::Point2f(0,0));
                inPts.push_back(cv::Point2f(objectWidth,0));
                inPts.push_back(cv::Point2f(objectWidth,objectHeight));
                inPts.push_back(cv::Point2f(0,objectHeight));
                inPts.push_back(cv::Point2f(objectWidth/2,objectHeight/2));
                cv::perspectiveTransform(inPts, outPts, cvHomography);

               // cv_bridge::CvImageConstPtr imageDepthPtr = cv_bridge::toCvShare(imageMsg);

                
                
                std::vector<cv::Point2i> outPtsInt;
                cv::Point2i InitRect = outPts[0];
                outPtsInt.push_back(outPts[0]);
                outPtsInt.push_back(outPts[1]);
                outPtsInt.push_back(outPts[2]);
                outPtsInt.push_back(outPts[3]);
              //  QColor color(QColor((Qt::GlobalColor)((id % 10 + 7)==Qt::yellow?Qt::darkYellow:(id % 10 + 7))));
                if(InitRect.x<0){InitRect.x = 0;}
                if(InitRect.y<0){InitRect.y = 0;}
                if(InitRect.x + size.x > 640){size.x = 640 - (InitRect.x + 5) ;
                  printf("Cropped immage too much on the left\n");}
                if(InitRect.y + size.y > 480){size.y= 480 - (InitRect.y + 5);
                  printf("Cropped immage too much down\n");}
                cv::Rect myROI(InitRect.x, InitRect.y, size.x, size.y);
                cv::Mat croppedImage = cvPtr->image(myROI);
               // imagePub.publish(img.toImageMsg());
                
                imshow("cropped", croppedImage);
                waitKey(1);

                object_id = data[0];
                object_id = std::fmod(object_id,7);
                cout << "Object id : "<<object_id<<endl;
                

                if(object_id == 3)
                {
                    dead = 2;
                    Mat hsv_image;
                    cvtColor(croppedImage, hsv_image, CV_BGR2HSV);
                    Mat green_image;
                   
                    inRange(hsv_image, Scalar(45, 10, 10), Scalar(95, 255, 255), green_image);

                    erode(green_image, green_image, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) );
                    
                    dilate(green_image, green_image, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) );

                    imshow("green_image", green_image);
                    waitKey(1);
                         // Find contours
                    Mat canny_output;
                    vector<vector<Point> > contours;
                    vector<Vec4i> hierarchy;
                    Mat gray = green_image;
                    
                      /// Detect edges using canny
                    Canny( gray, gray, 100, 200, 3 );
                      /// Find contours
                    findContours( gray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

                    if(contours.size() > 0){ 
                        dead = 1;
                        object_id = 7;
                        printf("This dude is ALIVE\n"); }
                } 
        objStamped.objects.data[0] = object_id;
        objStamped.objects.data[1] = data[1];
        objStamped.objects.data[2] = data[2];
        objStamped.objects.data[3] = data[3];
        objStamped.objects.data[4] = data[4];
        objStamped.objects.data[5] = data[5];
        objStamped.objects.data[6] = data[6];
        objStamped.objects.data[7] = data[7];
        objStamped.objects.data[8] = data[8];
        objStamped.objects.data[9] = data[9];
        objStamped.objects.data[10] = data[10];
        objStamped.objects.data[11] = data[11];
        //objStamped.header.stamp = imageMsg->header.stamp;
        objStamped.header.stamp = stamp;
        objStamped.header.frame_id = frame_id;
        

		//  pub_obj.publish(obj);
        
        }
        pub_obj_stamped.publish(objStamped);
        
}
    
 


int main(int argc, char** argv) {
    
   ros::init(argc, argv, "vision");
  
   RobotVision robot_vision;
   
   robot_vision.spin();

  return 0;
}
