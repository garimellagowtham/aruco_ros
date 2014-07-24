/*****************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <aruco_ros/aruco_ros_utils.h>

/* Headers from Aruco example */
#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
/************************************
 *
 *
 *
 *
 ************************************/


/* Nodelet begins: */
namespace aruco_ros{
	using namespace std;
	using namespace cv;
	using namespace aruco; 

	class Aruco_MarkerDetector : public nodelet::Nodelet
	{
		// ROS communication
		public: 
			//Internal Variables
			bool cam_info_received;
			bool useRectifiedParams;
			string camera_frame;
			//Image Transportation
			boost::shared_ptr<image_transport::ImageTransport> it_;
			image_transport::CameraSubscriber sub_camera_;
			image_transport::Publisher image_pub;
			//Subscribers
			//[NONE]
			//Publishers
			ros::Publisher pose_pub;
			//Aruco variables:
			MarkerDetector MDetector;
			vector<Marker> TheMarkers;
			CameraParameters TheCameraParameters;
			Mat outImage;
			sensor_msgs::ImagePtr outimg_msg;
			double marker_size;
			//double ThreshParam1,ThreshParam2;
			//Member Functions:
			virtual void onInit();
			void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
					const sensor_msgs::CameraInfoConstPtr& info_msg);
	};

	/* --- Definitions: --- */

	void Aruco_MarkerDetector::onInit()
	{
		ros::NodeHandle &nh         = getNodeHandle();
		ros::NodeHandle &private_nh = getPrivateNodeHandle();


		/*---Initialize the variables---*/

		//Image Transportation
		it_.reset(new image_transport::ImageTransport(nh));
		image_transport::TransportHints hints("raw", ros::TransportHints(), private_nh);
		sub_camera_ = it_->subscribeCamera("image", 1, &Aruco_MarkerDetector::imageCb, this, hints);//Subscribe to raw data
		// Declare Publishers
		image_pub  = it_->advertise("result",  1);//Contour image
		pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);
		// Set the Parameters needed
		cam_info_received = false;
		private_nh.param<double>("marker_size", marker_size, -1);//If no markersize then has to be -1
		//private_nh.param<std::string>("reference_frame", reference_frame, "");
		private_nh.param<std::string>("camera_frame", camera_frame, "camera");
		private_nh.param<bool>("param_rectified", useRectifiedParams, false);
		ROS_INFO("Using Camera_frame[%s]\t Rectification: [%d]\t marker_size[%f]\t ",camera_frame.c_str(), useRectifiedParams, marker_size);
		/* Assertions about parameters */
		//ROS_ASSERT(camera_frame != "");
		//TODO Add reconfigure server for this node if needed
		// ThePyrDownLevel
		// Threshold
	}

	//Image Callback whenever image comes:
	void Aruco_MarkerDetector::imageCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
	{
		static tf::TransformBroadcaster br;
		if(!cam_info_received)
		{
			// Set the aruco variables:
			try
			{
				TheCameraParameters = aruco_ros::rosCameraInfo2ArucoCamParams(info_msg, useRectifiedParams);
        MDetector.setCornerRefinementMethod(MarkerDetector::SUBPIX);
				//MDetector.getThresholdParams(ThreshParam1,ThreshParam2);
        //MDetector.pyrDown(ThePyrDownLevel);
			} 
			catch (std::exception &ex)
			{
				cerr<<"Exception :"<<ex.what()<<endl;
				return;
			}
			cam_info_received = true;
		}
		else// If camera info msg received then:
		{
			// Create cv::Mat views onto both buffers
			const cv::Mat inImage = cv_bridge::toCvShare(image_msg)->image;
			MDetector.detect(inImage,TheMarkers,TheCameraParameters,marker_size);
			//show input with augmented information
			if(image_pub.getNumSubscribers() > 0)
			{
				//inImage.copyTo(outImage);
				cv::cvtColor(inImage,outImage,CV_GRAY2BGR);
			}
			for (unsigned int i=0;i<TheMarkers.size();i++) {
				//cout<<endl<<TheMarkers[i];//Debug Statement
				tf::Transform transform = aruco_ros::arucoMarker2Tf(TheMarkers[i]);
				string marker_frame = "m";
				marker_frame += boost::to_string(TheMarkers[i].id);
				tf::StampedTransform stampedTransform(transform, ros::Time::now(),//have to put the camera stamp here
						camera_frame, marker_frame);
				br.sendTransform(stampedTransform);
				geometry_msgs::PoseStamped poseMsg;
				//Can decide later not publish this
				tf::poseTFToMsg(transform, poseMsg.pose);
				poseMsg.header.frame_id = camera_frame;
				poseMsg.header.stamp = ros::Time::now();
				pose_pub.publish(poseMsg);


				if(image_pub.getNumSubscribers() > 0)
				{
					//print marker borders
					TheMarkers[i].draw(outImage,Scalar(0,0,255),1);
					//draw 3d axis
					CvDrawingUtils::draw3dAxis( outImage,TheMarkers[i],TheCameraParameters);
				}
			}
			if(image_pub.getNumSubscribers() > 0)
			{
				outimg_msg = cv_bridge::CvImage(image_msg->header, "bgr8", outImage).toImageMsg();
				image_pub.publish(outimg_msg);
			}
		}
	}
}//namespace aruco_ros
	// Register nodelet
#include <pluginlib/class_list_macros.h>
	PLUGINLIB_DECLARE_CLASS(aruco_ros, marker_detector, aruco_ros::Aruco_MarkerDetector, nodelet::Nodelet)
