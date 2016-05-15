// This file is generated using ros node template, feel free to edit
// Please finish the TODO part
// maskinsyn_aruco.cpp
/*
  --------------------------------
  (C) 2014, Biorobotics Lab, Department of Electrical Engineering, University of Washington
  This file is part of RNA - The Ros Node Automator.

  RNA - The Ros Node Automator is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  RNA - The Ros Node Automator is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with RNA - The Ros Node Automator.  If not, see <http://www.gnu.org/licenses/>.
  -------------------------------- */
#include <ros/ros.h>

#include "opencv2/core/version.hpp"
#include <string.h>


// if messages or services used
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/UInt16.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

const std::string camera_win = "camera image";
/////////////////// Message Callbacks
void image_raw_cb(const sensor_msgs::ImageConstPtr& msg) {
    ROS_INFO("maskinsyn_aruco: I got message on topic '/usb_cam/image_raw'");
/*
    sensor_msgs::Image current_msg;
    current_msg = *msg;

    cv::Mat image(640,480,CV_8UC1);
    std::string str(current_msg.data.begin(),current_msg.data.end());
    image.data = (uchar*)strdup(str.c_str());

    IplImage* ettEllerAnnetPiss = sensor_msgs::*/


    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        ROS_INFO("maskinsyn_aruco: making pointer");
        cv_ptr = cv_bridge::toCvCopy(msg,"");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
        }

    ROS_INFO("maskinsyn_aruco: Trying to show image\n");
    cv::imshow(camera_win, cv_ptr->image);
    cv::waitKey(5);
}



/////////////////// Service Callbacks


int main(int argc, char** argv) {
ROS_INFO("Hello world!");
ROS_INFO("Started template C++ node: maskinsyn_aruco.");
// init and setup the node
ros::init(argc, argv, "maskinsyn_aruco");
ros::NodeHandle nh;

//////////////  Message Subscribers & Publishers
ros::Subscriber Image_sub1 = nh.subscribe("/usb_cam/image_raw", 1000, image_raw_cb);

ros::Publisher UInt16_pub1 = nh.advertise<std_msgs::UInt16>("motor", 1000);


//////////////  Service Advertisers & Initializers



////////////// Message Objects
std_msgs::UInt16 UInt16_obj1;

UInt16_obj1.data = 0;

////////////// Service objects




while(true) {

    cv::namedWindow("camera_win");

// message publish calls
UInt16_pub1.publish(UInt16_obj1);

// service calls


ROS_INFO("maskinsyn_aruco: main loop");
ros::spinOnce();   // yield to ROS
ros::Duration(2.0).sleep();

    int key = cv::waitKey(30);
    if (key == 'q') break;
}

return 0;
}
