/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Kangneoung Lee.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   *The names of its  contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Kangneoung Lee
 *********************************************************************/

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <sensor_msgs/CameraInfo.h>
#include <boost/bind.hpp>
#include <std_msgs/Int8.h>

#include <sys/stat.h> /*directory check*/
//#include <windows.h>  /*directory check*/

/************Function architecture*******************/
std::mutex _lock;

class DynamicVoronoiRos{
	
	private:
		 ros::NodeHandle main_nh_;
         ros::NodeHandle param_nh_;
	     ros::Rate* loop_rate_;

		 image_transport::ImageTransport  it_;		 
		 image_transport::Subscriber rgb_sub_;
		 image_transport::Subscriber depth_sub_;
		 ros::Subscriber rgb_cam_info_sub_;
		 ros::Subscriber depth_cam_info_sub_;
		 
		 float dscale_;
		 float fx_, fy_, px_, py_;	 
		 int map_height_;
		 int map_width_;
		 
		  int update_rate_;
		 
		 cv::Mat rgb_image_;
		 cv::Mat depth_image_;
	
	public:
	
	    void run();
		
		void rgb_image_callback(const sensor_msgs::Image::ConstPtr& msg, int value);
		void depth_image_callback(const sensor_msgs::Image::ConstPtr& msg, int value);
		void rgb_cam_info_callback(const sensor_msgs::CameraInfo::ConstPtr& msg, int value);
		void depth_cam_info_callback(const sensor_msgs::CameraInfo::ConstPtr& msg, int value);
	 
		/*constructor and destructor*/
	    DynamicVoronoiRos(ros::NodeHandle m_nh, ros::NodeHandle p_nh);
	    ~DynamicVoronoiRos();
	

}; // class DynamicVoronoiRos end

DynamicVoronoiRos::DynamicVoronoiRos(ros::NodeHandle m_nh, ros::NodeHandle p_nh):main_nh_(m_nh),param_nh_(p_nh),it_(m_nh)
{
	
	 int map_height = 200;
	 int map_width = 200;
	 
	 int update_rate = 10;
	
	 std::string  rgb_image_topic = "front_cam/camera/color/image_raw";
	 std::string  depth_image_topic = "front_cam/camera/depth/image_rect_raw";
	 std::string camera_rgb_info_topic = "front_cam/camera/color/camera_info";
	 std::string camera_depth_info_topic = "front_cam/camera/depth/camera_info";

	
	 param_nh_.getParam("depth_image_topic",depth_image_topic);
	 param_nh_.getParam("rgb_image_topic",rgb_image_topic);
	 param_nh_.getParam("camera_depth_info_topic",camera_depth_info_topic);
	 param_nh_.getParam("camera_rgb_info_topic",camera_rgb_info_topic);
	 
   	 param_nh_.getParam("map_height",map_height);
	 param_nh_.getParam("map_width",map_width);
	 param_nh_.getParam("update_rate",update_rate);

     this->map_height_ = map_height;
	 this->map_width_ = map_width;
	 this->update_rate_ = update_rate;
	 
	 this->rgb_sub_ =  it_.subscribe(rgb_image_topic, 1,boost::bind(&DynamicVoronoiRos::rgb_image_callback,this,_1,1));
	 this->depth_sub_ =  it_.subscribe(depth_image_topic, 1, boost::bind(&DynamicVoronoiRos::depth_image_callback,this,_1,1));
	
	 this->rgb_cam_info_sub_ = main_nh_.subscribe<sensor_msgs::CameraInfo>(camera_rgb_info_topic, 1, boost::bind(&DynamicVoronoiRos::rgb_cam_info_callback, this,_1,0));
	 this->depth_cam_info_sub_ = main_nh_.subscribe<sensor_msgs::CameraInfo>(camera_depth_info_topic, 1, boost::bind(&DynamicVoronoiRos::depth_cam_info_callback, this,_1,0));

     this->loop_rate_ = new ros::Rate(this->update_rate_);	
      
}

DynamicVoronoiRos::~DynamicVoronoiRos()
{
	
	delete this->loop_rate_;
}


void DynamicVoronoiRos::rgb_image_callback(const sensor_msgs::Image::ConstPtr& msg , int value)
{
	std::lock_guard<std::mutex> lock(_lock);
	
	cv::Mat rgb_image;
	rgb_image = cv_bridge::toCvShare(msg, "bgr8")->image;
	this->rgb_image_ = rgb_image.clone();
	
	if(value > 0)
	{
		ROS_INFO("rgb_image_callback");
	}
	
}


void DynamicVoronoiRos::depth_image_callback(const sensor_msgs::Image::ConstPtr& msg,int value)
{
	std::lock_guard<std::mutex> lock(_lock);
	cv::Mat depth_image = cv_bridge::toCvCopy(msg)->image;
	this->depth_image_ = depth_image.clone();
	
	if(value > 0)
	{
		ROS_INFO("depth_image_callback");
	}
}

void DynamicVoronoiRos::rgb_cam_info_callback(const sensor_msgs::CameraInfo::ConstPtr& msg ,int value)
{
          /*empty*/
		  
    if(value > 0)
	{
		ROS_INFO("rgb_cam_info_callback");
	}
}

void DynamicVoronoiRos::depth_cam_info_callback(const sensor_msgs::CameraInfo::ConstPtr& msg ,int value)
{        
          this->fx_ = msg->K[0];
          this->fy_ = msg->K[4];
          this->px_ = msg->K[2];
          this->py_ = msg->K[5];
	      this->dscale_ = 0.001;//this->_dscale = msg->D[0];
		  
	if(value > 0)
	{
		ROS_INFO("depth_cam_info_callback");
	}
}


void DynamicVoronoiRos::run()
{  
    while(ros::ok())
	{
		
	   //ros::spin();
	   ros::spinOnce();
	   this->loop_rate_->sleep();
	}
	
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_voronoi_ros");
  ros::NodeHandle nh;
  ros::NodeHandle _nh("~");
  
  DynamicVoronoiRos  dynamicvoronoi_ros(nh,_nh);
  
  dynamicvoronoi_ros.run();
 
   return 0;
}