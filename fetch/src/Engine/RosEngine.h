// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ImageSourceEngine.h"

#include <ros/ros.h>
//#include<ros_infiniTAM/req_im>
#include <cv_bridge/cv_bridge.h>	// OpenCV
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>

#define APPROXIMATE

#ifdef EXACT
#include <message_filters/sync_policies/exact_time.h>
#endif
#ifdef APPROXIMATE
#include <message_filters/sync_policies/approximate_time.h>
#endif

#ifdef EXACT
	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
#endif
#ifdef APPROXIMATE
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
#endif
typedef message_filters::Subscriber<sensor_msgs::Image> Image_sub_type;

namespace InfiniTAM
{
	namespace Engine
	{
		class RosEngine : public ImageSourceEngine
		{
		private:
			ros::NodeHandle nh;
			class PrivateData;
			PrivateData *data;
			Vector2i imageSize_rgb, imageSize_d;
			Image_sub_type *subscriber_depth;
			Image_sub_type *subscriber_rgb;
			message_filters::Synchronizer<MySyncPolicy> *sync;
			bool dataAvailable;
            ros::ServiceServer service;
		public:
            RosEngine(const char *calibFilename);
			~RosEngine();
			void callback( const sensor_msgs::ImageConstPtr& msg_rgb , const sensor_msgs::ImageConstPtr& msg_depth);
			bool hasMoreImages(void);
			void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);
            bool sendImages();
			Vector2i getDepthImageSize(void);
			Vector2i getRGBImageSize(void);
		};
	}
}

