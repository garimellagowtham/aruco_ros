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
#include <aruco/boarddetector.h>
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

	class Aruco_BoardDetector : public nodelet::Nodelet
	{
		// ROS communication
		public: 
			//Internal Variables
			bool cam_info_received;
			bool useRectifiedImages;
			string marker_frame;
			string camera_frame;
			string reference_frame;
			string boardconfigfile;
			tf::Transform cameraToReference;
			//Image Transportation
			boost::shared_ptr<image_transport::ImageTransport> it_;
			image_transport::CameraSubscriber sub_camera_;
			image_transport::Publisher image_pub;
			//Subscribers
			//[NONE]
			//Publishers
			ros::Publisher pose_pub;
			//Aruco variables:
			BoardConfiguration TheBoardConfig;
			BoardDetector TheBoardDetector;
			Board TheBoardDetected;
			CameraParameters TheCameraParameters;
			double marker_size;
			//double ThreshParam1,ThreshParam2;
			//Member Functions:
			virtual void onInit();
			void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
					const sensor_msgs::CameraInfoConstPtr& info_msg);
	};

	/* --- Definitions: --- */

	void Aruco_BoardDetector::onInit()
	{
		ros::NodeHandle &nh         = getNodeHandle();
		ros::NodeHandle &private_nh = getPrivateNodeHandle();


		/*---Initialize the variables---*/

		//Image Transportation
		it_.reset(new image_transport::ImageTransport(nh));
		image_transport::TransportHints hints("raw", ros::TransportHints(), private_nh);
		sub_camera_ = it_->subscribeCamera("image", 1, &Aruco_BoardDetector::imageCb, this, hints);//Subscribe to raw data
		// Declare Publishers
		image_pub  = it_->advertise("result",  1);//Contour image
		pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);
		// Set the Parameters needed
		cam_info_received = false;
		private_nh.param<double>("marker_size", marker_size, -1);//If no markersize then has to be -1
		private_nh.param<std::string>("reference_frame", reference_frame, "");
		private_nh.param<std::string>("camera_frame", camera_frame, "");
		private_nh.param<std::string>("marker_frame", marker_frame, "");
		private_nh.param<std::string>("board_config", boardconfigfile, "");
		private_nh.param<bool>("image_is_rectified", useRectifiedImages, true);

		/* Assertions about parameters */
		ROS_ASSERT(camera_frame != "" && marker_frame != "");
		ROS_ASSERT(boardconfigfile != "");
		if ( reference_frame.empty() )
			reference_frame = camera_frame;
		ROS_INFO("Aruco node will publish pose to TF with %s as parent and %s as child.",
				reference_frame.c_str(), marker_frame.c_str());
		cameraToReference.setIdentity();
		//TODO Add reconfigure server for this node if needed
	}

	//Non member function copied from simple_single old aruco
	static bool getTransform(const std::string& refFrame,
			const std::string& childFrame,
			tf::Transform& transform)
	{
    static tf::TransformListener _tfListener;
		static bool foundTransform = false;
		if(!foundTransform)
		{
			std::string errMsg;
			tf::StampedTransform temptransform;

			if ( !_tfListener.waitForTransform(refFrame,
						childFrame,
						ros::Time(0),
						ros::Duration(0.5),
						ros::Duration(0.01),
						&errMsg)
				 )
			{
				ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
				return false;
			}
			else
			{
				try
				{
					_tfListener.lookupTransform( refFrame, childFrame,
							ros::Time(0),                  //get latest available
							temptransform);
				}
				catch ( const tf::TransformException& e)
				{
					ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame);
					return false;
				}
			}
			transform = static_cast<tf::Transform>(temptransform);
			return true;
		}
	}
	//Image Callback whenever image comes:
	void Aruco_BoardDetector::imageCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
	{
		static tf::TransformBroadcaster br;
		if(!cam_info_received)
		{
			// Set the aruco variables:
			try
			{
				TheCameraParameters = aruco_ros::rosCameraInfo2ArucoCamParams(info_msg, useRectifiedImages);
				TheBoardConfig.readFromFile(boardconfigfile);
				TheBoardDetector.setParams(TheBoardConfig,TheCameraParameters,marker_size);
				//TheBoardDetector.getMarkerDetector().getThresholdParams(ThreshParam1,ThreshParam2);
				TheBoardDetector.getMarkerDetector().setCornerRefinementMethod(MarkerDetector::HARRIS);
				TheBoardDetector.set_repj_err_thres(1.5);//Copying defaults from utility
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
			float probDetect=TheBoardDetector.detect(inImage);
			//Use Probability to find covariances etc for the pose estimate later on

			tf::Transform transform = aruco_ros::arucoBoard2Tf(TheBoardDetector.getDetectedBoard());

			if ( reference_frame != camera_frame )
			{
				getTransform(reference_frame,//Use static transform for greater efficiency
						camera_frame,
						cameraToReference);
			}

			transform = cameraToReference * transform;

			tf::StampedTransform stampedTransform(transform, ros::Time::now(),//have to put the camera stamp
					reference_frame, marker_frame);
			br.sendTransform(stampedTransform);
			geometry_msgs::PoseStamped poseMsg;
			tf::poseTFToMsg(transform, poseMsg.pose);
			poseMsg.header.frame_id = reference_frame;
			poseMsg.header.stamp = ros::Time::now();
			pose_pub.publish(poseMsg);


			if(image_pub.getNumSubscribers() > 0)
			{
				//show input with augmented information
				Mat outImage;
				inImage.copyTo(outImage);
				//print marker borders
				for (unsigned int i=0;i<TheBoardDetector.getDetectedMarkers().size();i++)
					TheBoardDetector.getDetectedMarkers()[i].draw(outImage,Scalar(0,0,255),1);
				//draw 3d axis
				CvDrawingUtils::draw3dAxis( outImage,TheBoardDetector.getDetectedBoard(),TheCameraParameters);
				sensor_msgs::ImagePtr outimg_msg = cv_bridge::CvImage(image_msg->header, "rgb8", outImage).toImageMsg();
				image_pub.publish(outimg_msg);
			}
		}
	}
}//namespace aruco_ros
	// Register nodelet
#include <pluginlib/class_list_macros.h>
	PLUGINLIB_DECLARE_CLASS(aruco_ros, board_detector, aruco_ros::Aruco_BoardDetector, nodelet::Nodelet)
