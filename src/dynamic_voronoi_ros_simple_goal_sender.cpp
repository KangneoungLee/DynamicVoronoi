/*********************************************************************
MIT License

Copyright (c) 2022 Kangneoung Lee

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
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
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include "dynamic_voronoi/dynamic_voronoi.h"

#include <sys/stat.h> /*directory check*/
//#include <windows.h>  /*directory check*/

#define NO_INFORMATION_CELL -1
#define FREE_CELL 0

/************Function architecture*******************/
std::mutex _lock;
int global_cnt = 0;

struct agent_pose
{
	float x_m;
	float y_m;
	int index;
	bool is_renew;
	bool is_real;   // 0 : virtual agent, 1: real agent
	int explore_enable_cnt;
};

struct agent_prev_goal
{
	float prev_x_m;
	float prev_y_m;
	int index;
};

class DynamicVoronoiRosSimpleGoal{
	
	private:
		 ros::NodeHandle main_nh_;
         ros::NodeHandle param_nh_;
	     ros::Rate* loop_rate_;

		 image_transport::ImageTransport  it_;		 
		 image_transport::Subscriber rgb_sub_;
		 //image_transport::Subscriber depth_sub_;
		 //ros::Subscriber rgb_cam_info_sub_;
		 //ros::Subscriber depth_cam_info_sub_;
		 ros::Subscriber map_resol_sub_;
		 
		 float dscale_;
		 float fx_, fy_, px_, py_;	 
		 int map_height_;
		 int map_width_;
		 
		 float travel_time_duration_;
		 float min_dist_goal_send_;
		 int explore_enable_cnt_th_;
		 
		 bool is_img_save_;  
		 bool IsOnlineDenMap_;
		 bool ReadyByOnlineDenMap_;
		 bool ReadyMapResol_;
		 bool ResetMapComp_;
		 bool IsOnlinePoseUp_;
		 float map_resolution_;
		 
		 bool y_axis_swap_;
		 float agent_num_;
		 int no_agent_internal_param_;
		 
		 unsigned char* densityPtr_ = NULL;
		 
		 bool RealTime_InhbtDropout_;
		 bool RealTime_DropoutAct_;

		 float RealTime_CcMetric_ = 100;
		 float RealTime_CcMetric_prev_= 100;
		
		 float RealTime_CcMetricDif_prev_ = 0;
		 float RealTime_CcMetricDif_final_ = 0;
		 float CcRate_ = 0.3;
		 
		 boost::recursive_mutex online_map_mutex_;
		 boost::condition_variable_any online_map_cond_;
		 
		 VoroCell** Cellmap_;
		 DynamicVoronoi* DynVoroHandle_;

	     std::vector<std::vector<float> > agent_attribute_vec_;
	     XmlRpc::XmlRpcValue agent_attribute_list_;
		 
		 float update_rate_;
		 
		 std::vector<signed char*> agent_dynamic_cov_maps_;
		 
		 std::vector<ros::Subscriber> pose_sub_vec_;
		 std::vector<ros::Publisher> goal_pub_vec_;
		 std::vector<ros::Publisher> explore_enable_pub_vec_;
		 std::vector<ros::Publisher> is_real_agent_pub_vec_;
		 std::vector<ros::Publisher> cov_map_pub_vec_;
		 
		 struct agent_pose* agent_pose_array_;
		 struct agent_prev_goal* agent_goal_checker_;
		 
		 cv::Mat rgb_image_;
		 cv::Mat depth_image_;
		 cv::Mat gray_image_;
		 cv::Mat density_image_;
	
	public:
	
	    void run();
		
		void read_density(unsigned char*  density_array, std::string img_dir, int height, int width);
		void IntializedAgents();
		void RealTimeVoroProcess(int cnt);
		void UpdateCovCostMap();
		void ResetMap();
		void pose_sub_callback(const nav_msgs::Odometry::ConstPtr& msg , int index);
		void map_resol_callback(const std_msgs::Float64::ConstPtr& msg , int index);
		void density_image_callback(const sensor_msgs::Image::ConstPtr& msg, int value);
		//void depth_image_callback(const sensor_msgs::Image::ConstPtr& msg, int value);
		//void rgb_cam_info_callback(const sensor_msgs::CameraInfo::ConstPtr& msg, int value);
		//void depth_cam_info_callback(const sensor_msgs::CameraInfo::ConstPtr& msg, int value);
	 
		/*constructor and destructor*/
	    DynamicVoronoiRosSimpleGoal(ros::NodeHandle m_nh, ros::NodeHandle p_nh);
	    ~DynamicVoronoiRosSimpleGoal();
	

}; // class DynamicVoronoiRosSimpleGoal end

DynamicVoronoiRosSimpleGoal::DynamicVoronoiRosSimpleGoal(ros::NodeHandle m_nh, ros::NodeHandle p_nh):main_nh_(m_nh),param_nh_(p_nh),it_(m_nh)
{

	 float update_rate = 10;  //hz
	
	 
	 //std::string  depth_image_topic = "front_cam/camera/depth/image_rect_raw";
	 //std::string  camera_rgb_info_topic = "front_cam/camera/color/camera_info";
	 //std::string  camera_depth_info_topic = "front_cam/camera/depth/camera_info";
	 

	
	 //param_nh_.getParam("depth_image_topic",depth_image_topic);
	// param_nh_.getParam("rgb_image_topic",rgb_image_topic);
	 //param_nh_.getParam("camera_depth_info_topic",camera_depth_info_topic);
	 //param_nh_.getParam("camera_rgb_info_topic",camera_rgb_info_topic);

	 param_nh_.getParam("online_update_rate",update_rate);

	 this->update_rate_ = update_rate;
	 

	 //this->depth_sub_ =  it_.subscribe(depth_image_topic, 1, boost::bind(&DynamicVoronoiRosSimpleGoal::depth_image_callback,this,_1,1));
	
	 //this->rgb_cam_info_sub_ = main_nh_.subscribe<sensor_msgs::CameraInfo>(camera_rgb_info_topic, 1, boost::bind(&DynamicVoronoiRosSimpleGoal::rgb_cam_info_callback, this,_1,0));
	 //this->depth_cam_info_sub_ = main_nh_.subscribe<sensor_msgs::CameraInfo>(camera_depth_info_topic, 1, boost::bind(&DynamicVoronoiRosSimpleGoal::depth_cam_info_callback, this,_1,0));

     this->loop_rate_ = new ros::Rate(this->update_rate_);	
	 
	 bool IsOnlineDenMap = false;
	 bool IsOnlinePoseUp = true;
	 float map_resolution = 0.1;
	 
	 param_nh_.getParam("is_online_density_map",IsOnlineDenMap);	 
 	 param_nh_.getParam("is_online_pose_update",IsOnlinePoseUp);	  
     param_nh_.getParam("map_resolution",map_resolution);	 
	 
	 this->IsOnlineDenMap_ = IsOnlineDenMap;
	 this->IsOnlinePoseUp_ = IsOnlinePoseUp;
	 this->map_resolution_ = map_resolution;
	 
	 std::string  rgb_image_topic = "coverage_control/color/density";
	 std::string  map_resol_topic =  "coverage_control/map_resolution";
	 if(this->IsOnlineDenMap_ == true)
	 {
		 this->rgb_sub_ =  it_.subscribe(rgb_image_topic, 1,boost::bind(&DynamicVoronoiRosSimpleGoal::density_image_callback,this,_1,1));
		 this->map_resol_sub_ = main_nh_.subscribe<std_msgs::Float64>(map_resol_topic, 1, boost::bind(&DynamicVoronoiRosSimpleGoal::map_resol_callback, this,_1,0));
	 }
	 
	 this->ReadyByOnlineDenMap_ = false;
	 this->ReadyMapResol_ = false;
	 this->ResetMapComp_ = false;
	
	 int map_height = 200;
	 int map_width = 200;	 
	 	 
   	 param_nh_.getParam("map_height",map_height);
	 param_nh_.getParam("map_width",map_width);
	 
     this->map_height_ = map_height;
	 this->map_width_ = map_width;
	 
	 float weight_w = 1;
	 float weight_h = 1;
	 float dataum_x = 1;
	 float dataum_y = 1;
	 float lamda = 0;	 
	 
	 float travel_time_duration = 20;
	 float min_dist_goal_send = 0.5; // 0.5m

	 
	 
	 param_nh_.getParam("travel_time_duration",travel_time_duration);
	 param_nh_.getParam("min_dist_goal_send",min_dist_goal_send);
	 
	 this->travel_time_duration_ = travel_time_duration;
     this->min_dist_goal_send_ = min_dist_goal_send;
	 this->explore_enable_cnt_th_ =  5;

	 bool is_hete_cov_radius = false;
	 bool is_dropout_use = true;
	 bool is_propa_connected_area = false;
	 bool is_img_save = false;	 
	 bool work_eff_flag = false;	
	 bool is_uniform_density = false;
	
	 float agent_num = 10; 
	 int no_agent_internal_param = 4;
	 bool y_axis_swap = true;
	
	 
	 param_nh_.getParam("weight_width",weight_w);
	 param_nh_.getParam("weight_height",weight_h);
	 param_nh_.getParam("dataum_x",dataum_x);
	 param_nh_.getParam("dataum_y",dataum_y);
	 param_nh_.getParam("lamda",lamda);
	 param_nh_.getParam("is_hete_cov_radius",is_hete_cov_radius);
	 param_nh_.getParam("is_dropout_use",is_dropout_use);
	 param_nh_.getParam("is_img_save",is_img_save);
	 param_nh_.getParam("is_propa_connected_area",is_propa_connected_area);

	 
	 param_nh_.getParam("agent_attribute_list",agent_attribute_list_);
	 param_nh_.getParam("agent_num",agent_num);
	 param_nh_.getParam("no_agent_internal_param",no_agent_internal_param);
	 param_nh_.getParam("y_axis_swap",y_axis_swap);
	 // if y_axis_swap is true, map coordinate in dynamic voronoi follows image coordinate (E : x, S :y) and real agent follows ENU coordinate (E : x, N : y)
	 // if y_axis_swap is false, map coordinate in dynamic voronoi and real agents follow image coordinate (E : x, S :y)
	 
	 this->is_img_save_ = is_img_save;
	 this->y_axis_swap_ = y_axis_swap;
	 this->agent_num_ = agent_num;
	 this->no_agent_internal_param_ = no_agent_internal_param;
	 
	 if(is_dropout_use == true)  RealTime_InhbtDropout_ = false; 
	 else RealTime_InhbtDropout_ = true;
	 
	 float DropOutWeight = (float)this->agent_num_/(float)(this->agent_num_-1);
	 
	 std::string density_img_dir = "/home/kangneoung/sw_repo/dynamic_voronoi/src/dynamic_voronoi/test/density/density_200_200_ver3.png"; 
     param_nh_.getParam("density_img_dir",density_img_dir);
	 
	 
	 densityPtr_ = new unsigned char[ this->map_height_* this->map_width_];
	 
	 if(IsOnlineDenMap_ == false)  this->read_density(densityPtr_, density_img_dir, this->map_height_ , this->map_width_); //read density map from image file
	 else 
	 {
		 memset(densityPtr_, 0, this->map_height_* this->map_width_* sizeof(unsigned char));
	 }

	 
	 DynVoroHandle_ = new DynamicVoronoi(this->map_height_, this->map_width_, DropOutWeight, weight_w, weight_h, lamda, is_uniform_density, is_dropout_use, work_eff_flag, 
	                                                                       is_hete_cov_radius, is_propa_connected_area, is_img_save, densityPtr_);
																		   
	 DynVoroHandle_->PushDatum(dataum_x, dataum_y);
	 
	 agent_pose_array_ = new struct agent_pose[static_cast<int>(this->agent_num_)];
	 agent_goal_checker_ = new struct agent_prev_goal[static_cast<int>(this->agent_num_)];
	 
	 
	 this->IntializedAgents();	
	 
	 /**generate subscriber for pose of robots, create publisher for goal, explore enable, costmap for exploration,  intiailize the cost map for exploration*/
	 for(int index=0; index< this->agent_num_; index++)
	 {
		std::string robot_str = "Robot"; 
		std::string sub_topic = robot_str+std::to_string(index)+"/odom";
	    ros::Subscriber pose_sub = main_nh_.subscribe<nav_msgs::Odometry>(sub_topic, 1, boost::bind(&DynamicVoronoiRosSimpleGoal::pose_sub_callback, this,_1,index));
	    pose_sub_vec_.push_back(pose_sub);		 
		
		std::string pub_topic = robot_str+std::to_string(index)+"/goal";
		ros::Publisher goal_pub = main_nh_.advertise<geometry_msgs::PoseStamped>(pub_topic, 1);
		goal_pub_vec_.push_back(goal_pub);
		
		std::string pub2_topic = robot_str+std::to_string(index)+"/explore_enable";
		ros::Publisher exp_enable_pub = main_nh_.advertise<std_msgs::Int8>(pub2_topic, 1);
		explore_enable_pub_vec_.push_back(exp_enable_pub);
		
		std::string pub3_topic = robot_str+std::to_string(index)+"/is_real_agent";
		ros::Publisher is_real_agent_pub = main_nh_.advertise<std_msgs::Int8>(pub3_topic, 1);		
		is_real_agent_pub_vec_.push_back(is_real_agent_pub);
		
		std::string pub4_topic = robot_str+std::to_string(index)+"/coverage_costmap";
		ros::Publisher cov_map_pub = main_nh_.advertise<nav_msgs::OccupancyGrid>(pub4_topic, 1);
		cov_map_pub_vec_.push_back(cov_map_pub);
		
		signed char* cov_map = new signed char[ this->map_height_* this->map_width_];
		memset(cov_map, NO_INFORMATION_CELL, this->map_height_*this->map_width_*sizeof (signed char));
		agent_dynamic_cov_maps_.push_back(cov_map);
		
	 }
}

DynamicVoronoiRosSimpleGoal::~DynamicVoronoiRosSimpleGoal()
{
	
	delete this->loop_rate_;
	delete this->DynVoroHandle_;
    delete[] this->densityPtr_;
	delete[] this->agent_pose_array_;
	//delete[] this->agent_dynamic_cov_maps_;
}

void DynamicVoronoiRosSimpleGoal::read_density(unsigned char*  density_array, std::string img_dir, int height, int width)
{
	cv::Mat density_img = imread(img_dir, cv::IMREAD_GRAYSCALE);
	
	cv::resize(density_img, density_img, cv::Size(width, height));
    cv::Mat  density_img_thresh;

    //cv::threshold(density_img, density_img_thresh, 254, 254, cv::THRESH_TRUNC);
    density_img_thresh = density_img.clone();
	
	cv::Mat mat_255(cv::Size(density_img_thresh.cols, density_img_thresh.rows), CV_8UC1, cv::Scalar(255));

	cv::Mat converted_density;
	cv::absdiff(mat_255, density_img_thresh, converted_density);

	cv::Mat density_resized;
	cv::resize(converted_density, density_resized, cv::Size(width, height));

	density_resized.convertTo(density_resized, CV_32FC1);

	for(int row=0; row<density_resized.rows; row++)  
	{
      for(int col=0; col<density_resized.cols; col++)
	  {
		   int index =col + row*width;
		   density_array[index] =  (unsigned char)density_resized.at<float>(row, col);
	  }
	}		
	std::cout<<" ************Read density information ***************" <<std::endl;
	
	return;
	
}



void DynamicVoronoiRosSimpleGoal::IntializedAgents()
{
	 ROS_ASSERT(agent_attribute_list_.getType() == XmlRpc::XmlRpcValue::TypeArray);
	 
	 for (int i = 0; i < this->agent_num_; i++)
	 {
		 agent_goal_checker_[i].prev_x_m = 0.0;
         agent_goal_checker_[i].prev_y_m = 0.0; 			
		 agent_goal_checker_[i].index = i;
 		
		 agent_pose_array_[i].index = i;
		 agent_pose_array_[i].is_renew = false;
		 agent_pose_array_[i].explore_enable_cnt = 0;
		 
		 std::vector<float> agent_attribute;
		 for (int j =0; j<  this->no_agent_internal_param_; j++)
		 {
			 // These matrices can cause problems if all the types
			 // aren't specified with decimal points. Handle that
		    std::ostringstream ostr;
            ostr << agent_attribute_list_[this->no_agent_internal_param_ * i + j];    
			std::istringstream istr(ostr.str()); // istringstream parses the data corresponding to the data type
			
			float value_temp;
			istr >> value_temp;
			
			agent_attribute.push_back(value_temp);
		 }
		 agent_attribute_vec_.push_back(agent_attribute);
	 }
	 
	 std::vector<std::vector<float>>::iterator iter;
	 
	 int index = 0;
	 for (iter = this->agent_attribute_vec_.begin(); iter != this->agent_attribute_vec_.end(); iter++) {
		
		std::vector<float> agent_attribute;
		agent_attribute = *iter;
		if(this->no_agent_internal_param_ == 4)
		{
			float init_map_pos_x = agent_attribute.at(0);
			float init_map_pos_y = agent_attribute.at(1);
			float dummy_var1 = 0;
			float dummy_var2 = 0;
			DynVoroHandle_->PushPoint(init_map_pos_x, init_map_pos_y, dummy_var1, dummy_var2);   
			// agent_attribute.at(0)  is x_pose, agent_attribute.at(1)  is y_pose, agent_attribute.at(2)  is v_travel (pixel/time), agent_attribute.at(3)  is v_work (pixel/time)
        }
		else if(this->no_agent_internal_param_ == 5) 
		{
			float init_map_pos_x = agent_attribute.at(0);
			float init_map_pos_y = agent_attribute.at(1);
			float dummy_var1 = 0;
			float dummy_var2 = 0;
			float max_speed = agent_attribute.at(4);
			
			DynVoroHandle_->PushPoint(init_map_pos_x, init_map_pos_y, dummy_var1, dummy_var2,  this->travel_time_duration_*max_speed);
		}			
		else if(this->no_agent_internal_param_ == 6) 
		{
			float init_map_pos_x = agent_attribute.at(0);
			float init_map_pos_y = agent_attribute.at(1);
			float dummy_var1 = 0;
			float dummy_var2 = 0;
			float max_speed = agent_attribute.at(4);
			
			DynVoroHandle_->PushPoint(init_map_pos_x, init_map_pos_y, dummy_var1, dummy_var2,  this->travel_time_duration_*max_speed);
			agent_pose_array_[index].is_real = agent_attribute.at(5); // mark if the agent is real agent or virtual agent
		}		
    	
        index++;		
	 }	
}


void DynamicVoronoiRosSimpleGoal::map_resol_callback(const std_msgs::Float64::ConstPtr& msg , int index)
{
	if(this->map_resolution_ == (float)msg->data) return;
	this->map_resolution_ = (float)msg->data;
	this->ReadyMapResol_ = true;
}


void DynamicVoronoiRosSimpleGoal::pose_sub_callback(const nav_msgs::Odometry::ConstPtr& msg , int index)
{
	if(agent_pose_array_[index].is_renew == false)
	{
	    agent_pose_array_[index].x_m = msg->pose.pose.position.x;
	    agent_pose_array_[index].y_m= msg->pose.pose.position.y;
	    agent_pose_array_[index].index =index;
	    agent_pose_array_[index].is_renew =true;
	}
}



void DynamicVoronoiRosSimpleGoal::density_image_callback(const sensor_msgs::Image::ConstPtr& msg , int value)
{

	int map_height_new = msg->height;
	int map_width_new = msg->width;
	cv::Mat rgb_image;
	rgb_image = cv_bridge::toCvShare(msg, "bgr8")->image;
	
	if((this->map_height_ == map_height_new)&&(this->map_width_  == map_width_new)&&(!this->gray_image_.empty()))
	{
		     int sum; 
	         // compare previous density map and current density map
			 cv::Mat density_temp(map_height_new ,map_width_new, CV_8UC1);
			 cv::Mat density_temp_32f, density_image_32f;
			 cv::Mat abs_sub_out_32f, abs_sub_out_32s;
			 
			 cvtColor(rgb_image, density_temp, cv::COLOR_RGB2GRAY);
			 density_temp.convertTo(density_temp_32f, CV_32FC1);
			 
			 this->gray_image_.convertTo(density_image_32f, CV_32FC1);
			 
			 cv::absdiff(density_image_32f, density_temp_32f, abs_sub_out_32f);
			 
			 //abs_sub_out_32f.convertTo(abs_sub_out_32s, CV_32SC1);
             sum = cv::countNonZero(abs_sub_out_32f);
			 
			 if(sum == 0) 
			 {
				 ROS_INFO("same density map received");
				 return;
			 }
	}
	
	this->ResetMapComp_ = false;

	this->map_height_ = map_height_new;
	this->map_width_ = map_width_new;
	
	cv::Mat mat_255(cv::Size(this->map_width_, this->map_height_), CV_8UC1, cv::Scalar(255));	
    
	//this->rgb_image_ = rgb_image.clone();
	cvtColor(rgb_image, this->gray_image_, cv::COLOR_RGB2GRAY);
	
	cv::absdiff(mat_255, this->gray_image_, this->density_image_);
	
	//cv::imshow("test",this->density_image_);
	//cv::waitKey(10000);
	
	this->ReadyByOnlineDenMap_ = true;
	
	if(value > 0)
	{
		ROS_INFO("density_image_callback");
	}
	
}



/*void DynamicVoronoiRosSimpleGoal::depth_image_callback(const sensor_msgs::Image::ConstPtr& msg,int value)
{
	std::lock_guard<std::mutex> lock(_lock);
	cv::Mat depth_image = cv_bridge::toCvCopy(msg)->image;
	this->depth_image_ = depth_image.clone();
	
	if(value > 0)
	{
		ROS_INFO("depth_image_callback");
	}
}*/



/*void DynamicVoronoiRosSimpleGoal::rgb_cam_info_callback(const sensor_msgs::CameraInfo::ConstPtr& msg ,int value)
{
         
		  
    if(value > 0)
	{
		ROS_INFO("rgb_cam_info_callback");
	}
}*/



/*void DynamicVoronoiRosSimpleGoal::depth_cam_info_callback(const sensor_msgs::CameraInfo::ConstPtr& msg ,int value)
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
} */



void DynamicVoronoiRosSimpleGoal::ResetMap()
{
		
	if(!(densityPtr_ == NULL)) delete[] this->densityPtr_;
	
	this->densityPtr_ = new unsigned char[ this->map_height_* this->map_width_];


	uchar* p;
    for (int i = 0; i < this->map_height_; ++i) {
        p = this->density_image_.ptr<uchar>(i);
        for (int j = 0; j < this->map_width_; ++j)  this->densityPtr_ [i * this->map_width_ + j] = p[j];
     }
	
	DynVoroHandle_->ResizeMap(this->map_height_, this->map_width_, this->densityPtr_);
	
	if(!agent_dynamic_cov_maps_.empty()) agent_dynamic_cov_maps_.clear();
	
	for(int index=0; index< this->agent_num_; index++){
		signed char* cov_map = new signed char[ this->map_height_* this->map_width_];
		memset(cov_map, NO_INFORMATION_CELL, this->map_height_*this->map_width_*sizeof (signed char));
		agent_dynamic_cov_maps_.push_back(cov_map);
	}
}



void DynamicVoronoiRosSimpleGoal::RealTimeVoroProcess(int cnt)
{
	std::string img_dir =  "/home/kangneoung/sw_repo/dynamic_voronoi/src/dynamic_voronoi/test/real_time_debug/";
	std::string label_dir = "/home/kangneoung/sw_repo/dynamic_voronoi/src/dynamic_voronoi/label/label.txt";

	 if(cnt == 0)
	 {
		 DynVoroHandle_->ExpandedVoronoi();
	     DynVoroHandle_->CentroidCal();
	     DynVoroHandle_->MoveAgents();
		 
		 if(this->is_img_save_ == true)
		 {
		    std::string img_save_dir = img_dir + "map_" + std::to_string(global_cnt)+"_0"+"_apivot" +".png";
		    DynVoroHandle_->Colorized(img_save_dir,label_dir);
		 }			
		 
	     DynVoroHandle_->InitializeCell(); 
		 
	     this->RealTime_CcMetric_prev_ = this->RealTime_CcMetric_;
	     this->RealTime_CcMetric_ = DynVoroHandle_->CoverageMetric();
	 }
	 
	 if(this->RealTime_InhbtDropout_ == false)  this->RealTime_DropoutAct_ = DynVoroHandle_->AgentDropOut();
	 
	 if (this->RealTime_DropoutAct_ == true)
	 {
		 DynVoroHandle_->ExpandedVoronoi();
	     DynVoroHandle_->CentroidCal();
		 DynVoroHandle_->MoveAgents();
		 DynVoroHandle_->InitializeCell();
		  
         if(this->is_img_save_ == true)
		 {		  
		     std::string img_save_dir = img_dir + "map_" + std::to_string(global_cnt)+"_"+ std::to_string(cnt) + "_dropout" +".png";
		     DynVoroHandle_->Colorized(img_save_dir,label_dir);
		 }
		   
         DynVoroHandle_->ResetDropout();
		 this->RealTime_DropoutAct_ = false;  

	 }

	 DynVoroHandle_->ExpandedVoronoi();
	 DynVoroHandle_->CentroidCal();
	 DynVoroHandle_->MoveAgents();
	 

     if(this->is_img_save_ == true)
	 {		  		   
	     std::string img_save_dir = img_dir + "map_" + std::to_string(global_cnt) +"_"+ std::to_string(cnt) + "_final" +".png";
	     DynVoroHandle_->Colorized(img_save_dir,label_dir);
	 }
	 
	 //DynVoroHandle_->InitializeCell(); change the locatlion of InitializeCell because of coverage cost map update

	 

	 this->RealTime_CcMetric_prev_ = this->RealTime_CcMetric_;
	 this->RealTime_CcMetric_ = DynVoroHandle_->CoverageMetric();
	   
	 float RealTime_CcMetricDif_ =  this->RealTime_CcMetric_prev_ - this->RealTime_CcMetric_;
	 this->RealTime_CcMetricDif_final_ = this->CcRate_*RealTime_CcMetricDif_ + (1- this->CcRate_)*this->RealTime_CcMetricDif_prev_;
	 this->RealTime_CcMetricDif_prev_ = this->RealTime_CcMetricDif_final_;
		   
	 if ((this->RealTime_InhbtDropout_ == false)&&(this->RealTime_CcMetricDif_final_ <= 0.002))    
	 { 
		 this->RealTime_InhbtDropout_ = true;
		 ROS_INFO("Dropout inhibited, Loop count : %d, Delta Metric final :  %f, Delta Metric prev :  %f,  Metric :  %f, Metric prev :  %f",cnt ,this->RealTime_CcMetricDif_final_,
		                       this->RealTime_CcMetricDif_prev_, this->RealTime_CcMetric_, this->RealTime_CcMetric_prev_);
	 }
}

void DynamicVoronoiRosSimpleGoal::UpdateCovCostMap()
{
	   for (int x = 0; x<this->map_width_;x++)
	   {
		   for (int y = 0; y<this->map_height_;y++)
		   {
			    VoroCell* vorocell_temp = DynVoroHandle_->GetSingleCellByIndex(x, y);
			    int cell_index = DynVoroHandle_->GetIndex(x, y);
			    int agent_index = vorocell_temp->agentclass_;
				float density = DynVoroHandle_->GetDensity(x,y);
			  
			    if((agent_index >= this->agent_num_)||(density <= 0.0)) continue;
			  
			    signed char* temp_map =  agent_dynamic_cov_maps_[agent_index];
				
				if(this->y_axis_swap_ == true)  cell_index = x + this->map_width_*(this->map_height_ - y - 1);
				
			    signed char debug_cell_info = temp_map[cell_index];
			    temp_map[cell_index] = FREE_CELL;
			    //std::cout << " x : " << x << " y : " << y << " cell index : " << cell_index << " agent_index : " <<  " debug_cell_info : " << debug_cell_info << " new cell info : " <<  temp_map[cell_index] << std::endl;
		   }
	    }
	
}


void DynamicVoronoiRosSimpleGoal::run()
{  
    while(ros::ok())
	{
		if((this->ReadyByOnlineDenMap_ == true)&&(this->ReadyMapResol_ == true))
		{
			this->ResetMap();
			this->ReadyByOnlineDenMap_ = false;
			this->ResetMapComp_ = true;
			
		}
		
		bool ready = true;
		for(int index=0; index < this->agent_num_; index++)
		{
			if(agent_pose_array_[index].is_real == true)
			{
			     std_msgs::Int8 msg;
				 msg.data = 1;
				 is_real_agent_pub_vec_[index].publish(msg);
			}
			else
			{
			     std_msgs::Int8 msg;
				 msg.data = 0;
				 is_real_agent_pub_vec_[index].publish(msg);				
			}
			
			if(agent_pose_array_[index].is_renew == false)
			{
				ready = false;
				ROS_INFO("pose of robot %d is not received",index);
			}
            else 
			{
				float x_cell, y_cell;
				
				x_cell	= agent_pose_array_[index].x_m/this->map_resolution_;
				
				if(this->y_axis_swap_ == true) y_cell = (float)this->map_height_ - agent_pose_array_[index].y_m/this->map_resolution_;
				else y_cell = agent_pose_array_[index].y_m/this->map_resolution_;
				
				if(x_cell >= (float)this->map_width_)  x_cell = (float)this->map_width_ - 1.0;
			    if(y_cell >= (float)this->map_height_) y_cell = (float)this->map_height_ - 1.0;
           		
				bool success = DynVoroHandle_->AgentPoseUpdate(x_cell,  y_cell, agent_pose_array_[index].index);
				if(success == false) ROS_INFO("pose of robot %d is not updated",index);
			}				
		}
		
		bool duration_on = true;
		ros::Time init_time = ros::Time::now();
		
		int cnt = 0;
		if(( this->IsOnlineDenMap_ == true)&&(this->ResetMapComp_ == false)) ready= false;
			
		while((ready == true)&&(duration_on == true))
		{
			
			this->RealTimeVoroProcess(cnt);
			cnt++;
			ros::Duration elapse_time = ros::Time::now() - init_time;
			if(elapse_time >ros::Duration(1/this->update_rate_)) 
			{
				duration_on = false;
				this->UpdateCovCostMap();
				DynVoroHandle_->InitializeCell();
			}
			else DynVoroHandle_->InitializeCell();
		}
		
		global_cnt++;
		
		//Cellmap_ =   DynVoroHandle_->GetVoroCellMap();
				
		for(int index=0; index < this->agent_num_; index++)
		{
			float goal_x, goal_y;
			bool success = DynVoroHandle_->AgentPoseGet(goal_x,  goal_y, agent_pose_array_[index].index);
			
			float goal_x_m, goal_y_m;
			
			goal_x_m = goal_x*this->map_resolution_;
		
			if(this->y_axis_swap_ == true) goal_y_m = ((float)this->map_height_ - goal_y - 1.0)*this->map_resolution_;
		    else goal_y_m = goal_y*this->map_resolution_;
			
			
			float prev_x_m = agent_goal_checker_[index].prev_x_m;
			float prev_y_m = agent_goal_checker_[index].prev_y_m;
			
			float sq_dist_goal_to_goal = (goal_x_m - prev_x_m)*(goal_x_m - prev_x_m) + (goal_y_m - prev_y_m)*(goal_y_m - prev_y_m); 
			
			float pose_x = agent_pose_array_[index].x_m;
			float pose_y = agent_pose_array_[index].y_m;
			
			float sq_dist_goal_to_robot =  (goal_x_m - pose_x)*(goal_x_m - pose_x) + (goal_y_m - pose_y)*(goal_y_m - pose_y);
			
			if(success == false)
			{
				ROS_INFO("goal of robot %d is not published, check the index",index);
				continue;
			}
			
			if(sq_dist_goal_to_goal > this->min_dist_goal_send_*this->min_dist_goal_send_) 
			{
				geometry_msgs::PoseStamped goal_msg;
				
				goal_msg.pose.position.x = goal_x_m;
				goal_msg.pose.position.y = goal_y_m;
				goal_pub_vec_[index].publish(goal_msg);
				
				agent_goal_checker_[index].prev_x_m = goal_x_m;
				agent_goal_checker_[index].prev_y_m = goal_y_m;
				
				agent_pose_array_[index].explore_enable_cnt = 0;
				
				signed char* temp_map =  agent_dynamic_cov_maps_[index];
				memset(temp_map, NO_INFORMATION_CELL, this->map_height_*this->map_width_*sizeof (signed char));
			}
			else if(sq_dist_goal_to_robot< this->min_dist_goal_send_*this->min_dist_goal_send_)
			{
				agent_pose_array_[index].explore_enable_cnt = agent_pose_array_[index].explore_enable_cnt + 1;
			}

			
			if(agent_pose_array_[index].explore_enable_cnt >= this->explore_enable_cnt_th_)
			{
				std_msgs::Int8 msg;
				msg.data = 1;
				explore_enable_pub_vec_[index].publish(msg);
				
				nav_msgs::OccupancyGrid cov_costmap_msg;
				cov_costmap_msg.header.frame_id = "map";
				cov_costmap_msg.header.stamp = ros::Time::now();
				cov_costmap_msg.info.resolution = this->map_resolution_;
				cov_costmap_msg.info.width = this->map_width_;
				cov_costmap_msg.info.height = this->map_height_;
				cov_costmap_msg.info.origin.position.x = 0.0;
				cov_costmap_msg.info.origin.position.y = 0.0;
				cov_costmap_msg.info.origin.position.z = 0.0;
				cov_costmap_msg.info.origin.orientation.w = 1.0;
				
				//cov_costmap_msg.data.resize(this->map_width_* this->map_height_);
				
				
				//element copy from array to vector//
				signed char* temp_map =  agent_dynamic_cov_maps_[index];
                cov_costmap_msg.data.insert(cov_costmap_msg.data.begin(), temp_map, temp_map + this->map_width_*this->map_height_);
				//cov_costmap_msg.data.assign( agent_dynamic_cov_maps_[index],  agent_dynamic_cov_maps_[index] + this->map_width_*this->map_height_);
				
				cov_map_pub_vec_[index].publish(cov_costmap_msg);

				ROS_INFO("exploration of robot %d is enabled",index);
			}
			else
			{
				signed char* temp_map =  agent_dynamic_cov_maps_[index];
				memset(temp_map, NO_INFORMATION_CELL, this->map_height_*this->map_width_*sizeof (signed char));
			}
			
			agent_pose_array_[index].is_renew = false;
		}
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
  
  DynamicVoronoiRosSimpleGoal  dynamicvoronoi_ros_simple_goal(nh,_nh);
  
  dynamicvoronoi_ros_simple_goal.run();
 
   return 0;
}
