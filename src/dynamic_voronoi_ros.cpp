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
#include <geometry_msgs/PoseStamped.h>

#include "dynamic_voronoi/dynamic_voronoi.h"

#include <sys/stat.h> /*directory check*/
//#include <windows.h>  /*directory check*/

/************Function architecture*******************/
std::mutex _lock;
int global_cnt = 0;

struct agent_pose
{
	float x;
	float y;
	int index;
	bool is_renew;
};

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
		 
		 bool is_img_save_;  
		 bool IsOnlineDenMap_;
		 bool IsOnlinePoseUp_;
		 float map_resolution_;
		 
		 float agent_num_;
		 int no_agent_internal_param_;
		 
		 unsigned char* densityPtr_;
		 
		 bool RealTime_InhbtDropout_;
		 bool RealTime_DropoutAct_;

		 float RealTime_CcMetric_ = 100;
		 float RealTime_CcMetric_prev_= 100;
		
		 float RealTime_CcMetricDif_prev_ = 0;
		 float RealTime_CcMetricDif_final_ = 0;
		 float CcRate_ = 0.3;
		 
		 DynamicVoronoi* DynVoroHandle_;

	     std::vector<std::vector<int> > agent_attribute_vec_;
	     XmlRpc::XmlRpcValue agent_attribute_list_;
		 
		 float update_rate_;
		 
		 std::vector<ros::Subscriber> pose_sub_vec_;
		 std::vector<ros::Publisher> goal_pub_vec_;
		 struct agent_pose* agent_pose_array_;
		 
		 cv::Mat rgb_image_;
		 cv::Mat depth_image_;
	
	public:
	
	    void run();
		
		void read_density(unsigned char*  density_array, std::string img_dir, int height, int width);
		void IntializedAgents();
		void RealTimeVoroProcess(int cnt);
		void pose_sub_callback(const geometry_msgs::PoseStamped::ConstPtr& msg , int index);
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

	 float update_rate = 10;  //hz
	
	 std::string  rgb_image_topic = "front_cam/camera/color/image_raw";
	 std::string  depth_image_topic = "front_cam/camera/depth/image_rect_raw";
	 std::string camera_rgb_info_topic = "front_cam/camera/color/camera_info";
	 std::string camera_depth_info_topic = "front_cam/camera/depth/camera_info";

	
	 param_nh_.getParam("depth_image_topic",depth_image_topic);
	 param_nh_.getParam("rgb_image_topic",rgb_image_topic);
	 param_nh_.getParam("camera_depth_info_topic",camera_depth_info_topic);
	 param_nh_.getParam("camera_rgb_info_topic",camera_rgb_info_topic);

	 param_nh_.getParam("online_update_rate",update_rate);

	 this->update_rate_ = update_rate;
	 
	 this->rgb_sub_ =  it_.subscribe(rgb_image_topic, 1,boost::bind(&DynamicVoronoiRos::rgb_image_callback,this,_1,1));
	 this->depth_sub_ =  it_.subscribe(depth_image_topic, 1, boost::bind(&DynamicVoronoiRos::depth_image_callback,this,_1,1));
	
	 this->rgb_cam_info_sub_ = main_nh_.subscribe<sensor_msgs::CameraInfo>(camera_rgb_info_topic, 1, boost::bind(&DynamicVoronoiRos::rgb_cam_info_callback, this,_1,0));
	 this->depth_cam_info_sub_ = main_nh_.subscribe<sensor_msgs::CameraInfo>(camera_depth_info_topic, 1, boost::bind(&DynamicVoronoiRos::depth_cam_info_callback, this,_1,0));

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

	 bool is_hete_cov_radius = false;
	 bool is_dropout_use = true;
	 bool is_propa_connected_area = false;
	 bool is_img_save = false;	 
	 bool work_eff_flag = false;	
	 bool is_uniform_density = false;
	
	 float agent_num = 10; 
	 int no_agent_internal_param = 4;
	 
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
	 
	 this->is_img_save_ = is_img_save;
	 this->agent_num_ = agent_num;
	 this->no_agent_internal_param_ = no_agent_internal_param;
	 
	 if(is_dropout_use == true)  RealTime_InhbtDropout_ = false; 
	 else RealTime_InhbtDropout_ = true;
	 
	 float DropOutWeight = (float)this->agent_num_/(float)(this->agent_num_-1);
	 
	 std::string density_img_dir = "/home/kangneoung/sw_repo/dynamic_voronoi/src/dynamic_voronoi/test/density/density_200_200_ver3.png"; 
     param_nh_.getParam("density_img_dir",density_img_dir);
	 
	 
	 densityPtr_ = new unsigned char[ this->map_height_* this->map_width_];
	 
	 if(IsOnlineDenMap_ == false)  this->read_density(densityPtr_, density_img_dir, this->map_height_ , this->map_width_);
	 else memset(densityPtr_, 0, this->map_height_* this->map_width_* sizeof(unsigned char));

	 
	 DynVoroHandle_ = new DynamicVoronoi(this->map_height_, this->map_width_, DropOutWeight, weight_w, weight_h, lamda, is_uniform_density, is_dropout_use, work_eff_flag, 
	                                                                       is_hete_cov_radius, is_propa_connected_area, is_img_save, densityPtr_);
																		   
	 DynVoroHandle_->PushDatum(dataum_x, dataum_y);
	 
	 agent_pose_array_ = new struct agent_pose[static_cast<int>(this->agent_num_)];
	 
	 
	 this->IntializedAgents();	
	 
	 for(int index=0; index< this->agent_num_; index++)
	 {
		std::string robot_str = "robot"; 
		std::string sub_topic = robot_str+std::to_string(index)+"/pose";
	    ros::Subscriber pose_sub = main_nh_.subscribe<geometry_msgs::PoseStamped>(sub_topic, 1, boost::bind(&DynamicVoronoiRos::pose_sub_callback, this,_1,index));
	    pose_sub_vec_.push_back(pose_sub);		 
		
		std::string pub_topic = robot_str+std::to_string(index)+"/goal";
		ros::Publisher goal_pub = main_nh_.advertise<geometry_msgs::PoseStamped>(pub_topic, 1);
		goal_pub_vec_.push_back(goal_pub);
		
		agent_pose_array_[index].index = index;
		agent_pose_array_[index].is_renew = false;
	 }
}

DynamicVoronoiRos::~DynamicVoronoiRos()
{
	
	delete this->loop_rate_;
	delete this->DynVoroHandle_;
    delete[] this->densityPtr_;
	delete[] this->agent_pose_array_;
}

void DynamicVoronoiRos::read_density(unsigned char*  density_array, std::string img_dir, int height, int width)
{
	cv::Mat density_img = imread(img_dir, cv::IMREAD_GRAYSCALE);
	
	cv::resize(density_img, density_img, cv::Size(height, width));
    cv::Mat  density_img_thresh;

    //cv::threshold(density_img, density_img_thresh, 254, 254, cv::THRESH_TRUNC);
    density_img_thresh = density_img.clone();
	
	cv::Mat mat_255(cv::Size(density_img_thresh.rows, density_img_thresh.cols), CV_8UC1, cv::Scalar(255));

	cv::Mat converted_density;
	cv::absdiff(mat_255, density_img_thresh, converted_density);

	cv::Mat density_resized;
	cv::resize(converted_density, density_resized, cv::Size(height, width));

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

void DynamicVoronoiRos::IntializedAgents()
{
	 ROS_ASSERT(agent_attribute_list_.getType() == XmlRpc::XmlRpcValue::TypeArray);
	 
	 for (int i = 0; i < this->agent_num_; i++)
	 {
		 std::vector<int> agent_attribute;
		 for (int j =0; j<  this->no_agent_internal_param_; j++)
		 {
			 // These matrices can cause problems if all the types
			 // aren't specified with decimal points. Handle that
		    std::ostringstream ostr;
            ostr << agent_attribute_list_[this->no_agent_internal_param_ * i + j];    
			std::istringstream istr(ostr.str()); // istringstream parses the data corresponding to the data type
			
			int value_temp;
			istr >> value_temp;
			agent_attribute.push_back(value_temp);
		 }
		 agent_attribute_vec_.push_back(agent_attribute);
	 }
	 
	 std::vector<std::vector<int>>::iterator iter;
	 
	 for (iter = this->agent_attribute_vec_.begin(); iter != this->agent_attribute_vec_.end(); iter++) {
		
		std::vector<int> agent_attribute;
		agent_attribute = *iter;
		if(this->no_agent_internal_param_ == 4)
		{
			DynVoroHandle_->PushPoint(agent_attribute.at(0), agent_attribute.at(1), agent_attribute.at(2), agent_attribute.at(3));   
			// agent_attribute.at(0)  is x_pose, agent_attribute.at(1)  is y_pose, agent_attribute.at(2)  is v_travel (pixel/time), agent_attribute.at(3)  is v_work (pixel/time)
        }
		else if(this->no_agent_internal_param_ == 5) DynVoroHandle_->PushPoint(agent_attribute.at(0), agent_attribute.at(1), agent_attribute.at(2), agent_attribute.at(3), agent_attribute.at(4)); 
	 }	
}

void DynamicVoronoiRos::pose_sub_callback(const geometry_msgs::PoseStamped::ConstPtr& msg , int index)
{
	if(agent_pose_array_[index].is_renew == false)
	{
	    agent_pose_array_[index].x = msg->pose.position.x/this->map_resolution_;
	    agent_pose_array_[index].y = msg->pose.position.y/this->map_resolution_;
	    agent_pose_array_[index].index =index;
	    agent_pose_array_[index].is_renew =true;
	}
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

void DynamicVoronoiRos::RealTimeVoroProcess(int cnt)
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
	 
	 DynVoroHandle_->InitializeCell();

	 

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

void DynamicVoronoiRos::run()
{  
    while(ros::ok())
	{
		bool ready = true;
		for(int index=0; index < this->agent_num_; index++)
		{
			if(agent_pose_array_[index].is_renew == false)
			{
				ready = false;
				ROS_INFO("pose of robot %d is not received",index);
			}
            else 
			{
				bool success = DynVoroHandle_->AgentPoseUpdate(agent_pose_array_[index].x,  agent_pose_array_[index].y, agent_pose_array_[index].index);
				if(success == false) ROS_INFO("pose of robot %d is not updated",index);
			}				
		}
		
		bool duration_on = true;
		ros::Time init_time = ros::Time::now();
		
		int cnt = 0;
		
		while((ready == true)&&(duration_on == true))
		{
			
			this->RealTimeVoroProcess(cnt);
			cnt++;
			ros::Duration elapse_time = ros::Time::now() - init_time;
			if(elapse_time >ros::Duration(1/this->update_rate_)) duration_on = false;
		}
		
		global_cnt++;
		
		for(int index=0; index < this->agent_num_; index++)
		{
			float goal_x, goal_y;
			bool success = DynVoroHandle_->AgentPoseGet(goal_x,  goal_y, agent_pose_array_[index].index);
			
			geometry_msgs::PoseStamped goal_msg;
			
			if(success == true) 
			{
				goal_msg.pose.position.x = goal_x*this->map_resolution_;
				goal_msg.pose.position.y = goal_y*this->map_resolution_;
				goal_pub_vec_[index].publish(goal_msg);
			}
			else ROS_INFO("goal of robot %d is not published, check the index",index);
			
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
  
  DynamicVoronoiRos  dynamicvoronoi_ros(nh,_nh);
  
  dynamicvoronoi_ros.run();
 
   return 0;
}
