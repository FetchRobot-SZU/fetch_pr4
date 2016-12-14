// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "RosEngine.h"

#include "../Utils/FileUtils.h"

#include <cstdio>
#include <stdexcept>


using namespace InfiniTAM::Engine;

class RosEngine::PrivateData {
	public:
	PrivateData(void){}
	cv::Mat depthFrame;
	cv::Mat colorFrame;
};

RosEngine::RosEngine(const char *calibFilename)
	: ImageSourceEngine(calibFilename)
{	
	this->calib.disparityCalib.type = ITMDisparityCalib::TRAFO_AFFINE;
    this->calib.disparityCalib.params = Vector2f(1.0f/1000.0f, 0.0f);
	subscriber_depth=new Image_sub_type( nh , "/head_camera/depth_registered/image_raw", 1);
    subscriber_rgb=new Image_sub_type( nh , "/head_camera/rgb/image_raw" , 1);
    sync=new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10),*subscriber_rgb, *subscriber_depth);
  	sync->registerCallback(boost::bind(&RosEngine::callback,this,_1, _2));
    imageSize_rgb = Vector2i(640, 480);
    imageSize_d = Vector2i(640, 480);
	data=new PrivateData();
	this->dataAvailable=false;
    //service = nh.advertiseService("send_image_server", &RosEngine::sendImage);
}

RosEngine::~RosEngine()
{
	if (data != NULL)
	   delete data;
}

void RosEngine::callback( const sensor_msgs::ImageConstPtr& msg_rgb , const sensor_msgs::ImageConstPtr& msg_depth )
{
    cv_bridge::CvImagePtr img_ptr_rgb;
    cv_bridge::CvImagePtr img_ptr_depth;
 //       ROS_INFO("orgin depth image_type is = %s", msg_depth->encoding.c_str() );
    try{
        img_ptr_depth = cv_bridge::toCvCopy(*msg_depth, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception:  %s", e.what());
        return;
    }
   //     ROS_INFO("new depth image_type is = %s", img_ptr_depth->encoding.c_str() );
    try{
        img_ptr_rgb = cv_bridge::toCvCopy(*msg_rgb, sensor_msgs::image_encodings::TYPE_8UC3);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception:  %s", e.what());
        return;
    }
    cv::Mat& mat_depth = img_ptr_depth->image;
    data->colorFrame = img_ptr_rgb->image;
    mat_depth.convertTo(data->depthFrame,CV_16UC1,1000,0);
    this->dataAvailable=true;
}
//bool RosEngine:sendImages(ros_infiniTAM::req_im::Request  &req,ros_infiniTAM::req_im::Response &res){
//    if (req.isok==1)
//    {
//        if (this->dataAvailable)
//        {

//        }
//    }

//}
void RosEngine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{
	Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);
	short *depth = rawDepthImage->GetData(MEMORYDEVICE_CPU);
	if (this->dataAvailable)
	{
		cv::MatIterator_<cv::Vec3b> it=data->colorFrame.begin<cv::Vec3b>();
		cv::MatIterator_<short> d_it=data->depthFrame.begin<short>();
		for (int i = 0; i < rgbImage->noDims.x * rgbImage->noDims.y; i++)
		{
			Vector4u newPix;
			newPix.x =(*it)[2]; newPix.y = (*it)[1]; newPix.z = (*it)[0]; newPix.w = 255;
			rgb[i] = newPix;
			depth[i] = (*d_it);
			++it;
			++d_it;
		}
	}
	else 
	{
	     memset(rgb, 0, rgbImage->dataSize * sizeof(Vector4u));
	     memset(depth, 0, rawDepthImage->dataSize * sizeof(short));
	}

	return /*true*/;
}

bool RosEngine::hasMoreImages(void) { return (data!=NULL); }
Vector2i RosEngine::getDepthImageSize(void) { return (data!=NULL)?imageSize_d:Vector2i(0,0); }
Vector2i RosEngine::getRGBImageSize(void) { return (data!=NULL)?imageSize_rgb:Vector2i(0,0); }



