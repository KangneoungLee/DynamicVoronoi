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
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <sys/stat.h> /*directory check*/
//#include <windows.h>  /*directory check*/

/************Function architecture*******************/
std::mutex _lock;
int global_cnt = 0;

struct agent_info
{
	float x;
	float y;
	float max_speed;
	int index;
	bool is_renew;
};

class DynVoroVirtualAgentRos{
	
	private:
		 image_transport::ImageTransport  it_;
		 ros::NodeHandle main_nh_;
         ros::NodeHandle param_nh_;
	     ros::Rate* loop_rate_;

		 float map_height_;
		 float map_width_;
		 float map_resolution_;

		 float update_rate_;
		 
		 int virtual_agent_start_index_;
		 
		 bool IsOnlineDenMap_;		 
		 bool ReadyByOnlineDenMap_;
		 bool ReadyMapResol_;
		 
		 bool y_axis_swap_;
		 int agent_num_;
		 
		 std::vector<ros::Subscriber> goal_sub_vec_;
		 std::vector<ros::Publisher> pose_pub_vec_;
		 
		 image_transport::Subscriber rgb_sub_;
		 ros::Subscriber map_resol_sub_;
		 
		 float move_rate_ = 0.1;
		 
		 float init_pivot_x_m_;
		 float init_pivot_y_m_;
		 float init_random_d_m_;
		 
		 struct agent_info* agent_pose_array_;
		 struct agent_info* agent_goal_array_;
		 
	public:
	
	    void run();
		void IntializedAgents();
		void map_resol_callback(const std_msgs::Float64::ConstPtr& msg , int index);		
		void goal_sub_callback(const geometry_msgs::PoseStamped::ConstPtr& msg , int index);
		void density_image_callback(const sensor_msgs::Image::ConstPtr& msg, int value);
	 
		/*constructor and destructor*/
	    DynVoroVirtualAgentRos(ros::NodeHandle m_nh, ros::NodeHandle p_nh);
	    ~DynVoroVirtualAgentRos();
	

}; // class DynamicVoronoiRos end

DynVoroVirtualAgentRos::DynVoroVirtualAgentRos(ros::NodeHandle m_nh, ros::NodeHandle p_nh):main_nh_(m_nh),param_nh_(p_nh),it_(m_nh)
{

	 float update_rate = 10;  //hz

	 param_nh_.getParam("online_update_rate",update_rate);

	 this->update_rate_ = update_rate;
	 
     this->loop_rate_ = new ros::Rate(this->update_rate_);	



	 bool IsOnlineDenMap = false;
	 
	 param_nh_.getParam("is_online_density_map",IsOnlineDenMap);	

	 this->IsOnlineDenMap_ = IsOnlineDenMap;
	 
	 bool y_axis_swap = true;
	 
	 param_nh_.getParam("y_axis_swap",y_axis_swap);	 
	 this->y_axis_swap_ = y_axis_swap;
	
	 float map_height = 20; //unit : meter
	 float map_width = 20; //unit : meter  
	 float map_resolution = 0.1;
	 	 
   	 param_nh_.getParam("map_height",map_height);
	 param_nh_.getParam("map_width",map_width);
	 param_nh_.getParam("map_resolution",map_resolution);
	 
     this->map_height_ = map_height;
	 this->map_width_ = map_width;
	 this->map_resolution_ = map_resolution;
	 
	 int virtual_agent_start_index = 0;
	 param_nh_.getParam("virtual_agent_start_index", virtual_agent_start_index);
	 this->virtual_agent_start_index_ = virtual_agent_start_index;
	
	 float agent_num = 10; 
	 param_nh_.getParam("agent_num",agent_num);
	 this->agent_num_ = agent_num;

     float init_pivot_x_m;
	 param_nh_.getParam("init_pivot_x_m",init_pivot_x_m);
	 this->init_pivot_x_m_ = init_pivot_x_m;

     float init_pivot_y_m;
	 param_nh_.getParam("init_pivot_y_m",init_pivot_y_m);
	 this->init_pivot_y_m_ = init_pivot_y_m;
	 
	 float init_random_d_m;
	 param_nh_.getParam("init_random_d_m",init_random_d_m);
	 this->init_random_d_m_ = init_random_d_m;
    
	if(this->IsOnlineDenMap_ == true)
	{
	 	 std::string  rgb_image_topic = "coverage_control/color/init";
 	     this->rgb_sub_ =  it_.subscribe(rgb_image_topic, 1,boost::bind(&DynVoroVirtualAgentRos::density_image_callback,this,_1,1)); 
	 
	     std::string  map_resol_topic =  "coverage_control/map_resolution";
	     this->map_resol_sub_ = main_nh_.subscribe<std_msgs::Float64>(map_resol_topic, 1, boost::bind(&DynVoroVirtualAgentRos::map_resol_callback, this,_1,0));
		 
		 std::string  cancel_mission_topic =  "coverage_control/cancel";
	}
	
	 this->ReadyByOnlineDenMap_ = false;
	 this->ReadyMapResol_ = false;
 
	 agent_pose_array_ = new struct agent_info[static_cast<int>(this->agent_num_)];
	 agent_goal_array_ = new struct agent_info[static_cast<int>(this->agent_num_)];
	 
	 this->IntializedAgents();	
	 
	 for(int index=this->virtual_agent_start_index_; index< this->agent_num_; index++)
	 {
		std::string robot_str = "Robot"; 
		std::string sub_topic = robot_str+std::to_string(index)+"/goal";
	    ros::Subscriber goal_sub = main_nh_.subscribe<geometry_msgs::PoseStamped>(sub_topic, 1, boost::bind(&DynVoroVirtualAgentRos::goal_sub_callback, this,_1,index));
	    goal_sub_vec_.push_back(goal_sub);		 
		
		std::string pub_topic = robot_str+std::to_string(index)+"/odom";
		ros::Publisher pose_pub = main_nh_.advertise<nav_msgs::Odometry>(pub_topic, 1);
		pose_pub_vec_.push_back(pose_pub);
		
		agent_pose_array_[index].index = index;
		agent_pose_array_[index].is_renew = false;
		
		agent_goal_array_[index].index = index;
		agent_goal_array_[index].is_renew = false;
	 }
	 
	 ROS_INFO("virtual agent initialize done");
}

DynVoroVirtualAgentRos::~DynVoroVirtualAgentRos()
{
	
	delete this->loop_rate_;
	delete[] this->agent_pose_array_;
    delete[] this->agent_goal_array_;
}


void DynVoroVirtualAgentRos::IntializedAgents()
{ 
	 for(int index=this->virtual_agent_start_index_; index< this->agent_num_; index++)
	 {
		 int inc1 = (index -  this->virtual_agent_start_index_)/(int)2 + index%2;
		 int inc2 = (index -  this->virtual_agent_start_index_)/(int)2 + (index+1)%2;
         agent_pose_array_[index].x = this->init_pivot_x_m_ + this->init_random_d_m_*(float)inc1;
		 
		 if( this->y_axis_swap_ == true) agent_pose_array_[index].y = (float)this->map_height_*this->map_resolution_ - (this->init_pivot_y_m_ + this->init_random_d_m_*(float)inc2);
		 else agent_pose_array_[index].y = this->init_pivot_y_m_ + this->init_random_d_m_*(float)inc2; 
	     
	     agent_pose_array_[index].max_speed = 0.4; // m/s
		 
		 //ROS_INFO("%d inc1  : %d inc 2 : %d, %d, %d ", index, inc1, inc2, index%2, (index+1)%2);
     }
}

void DynVoroVirtualAgentRos::map_resol_callback(const std_msgs::Float64::ConstPtr& msg , int index)
{
	if(this->map_resolution_ == (float)msg->data) return;
	this->map_resolution_ = (float)msg->data;
	this->ReadyMapResol_ = true;
}

void DynVoroVirtualAgentRos::goal_sub_callback(const geometry_msgs::PoseStamped::ConstPtr& msg , int index)
{
	 agent_goal_array_[index].x = msg->pose.position.x;
	 agent_goal_array_[index].y = msg->pose.position.y;
	 agent_goal_array_[index].index =index;
	 agent_goal_array_[index].is_renew =true;
}

void DynVoroVirtualAgentRos::density_image_callback(const sensor_msgs::Image::ConstPtr& msg , int value)
{
    
	if(this->ReadyMapResol_ == false) return;
	
	int map_height_new = msg->height;
	int map_width_new = msg->width;
	
	if((this->map_height_ == map_height_new)&&(this->map_width_  == map_width_new))  return;
   
    this->ReadyByOnlineDenMap_ = false;
	
	this->map_height_ = map_height_new;
	this->map_width_ = map_width_new;
    
	this->IntializedAgents();
	
	this->ReadyByOnlineDenMap_ = true;
		
	if(value > 0)
	{
		ROS_INFO("density_image_callback");
	}
	
}

void DynVoroVirtualAgentRos::run()
{  

	

	
    while(ros::ok())
	{
		bool ready = false;
		
        if ((this->IsOnlineDenMap_ == true)&&(this->ReadyByOnlineDenMap_ == true)&&(this->ReadyMapResol_ == true)) ready = true;
        else if (this->IsOnlineDenMap_ == false) ready = true;
		
		
		for(int index=this->virtual_agent_start_index_; index < this->agent_num_; index++)
		{
			if (ready == false) break;
			
			if(agent_goal_array_[index].is_renew == true)
			{
				float del_x =  (agent_goal_array_[index].x - agent_pose_array_[index].x);
				float del_y = (agent_goal_array_[index].y - agent_pose_array_[index].y);
				float dist = (float)sqrt((double)(del_x*del_x + del_y*del_y));
				float max_dist = agent_pose_array_[index].max_speed/this->update_rate_;
				
				float inc_x = 0;
				float inc_y = 0;
				
				if(dist > max_dist)
				{
					float cos =  del_x/dist;
					float sin =   del_y/dist;
					
					inc_x = cos*max_dist;
					inc_y = sin*max_dist;
				}
				else
				{
					inc_x = del_x;
					inc_y = del_y;
				}
				
			   agent_pose_array_[index].x =  agent_pose_array_[index].x +  inc_x;
			   agent_pose_array_[index].y =  agent_pose_array_[index].y +  inc_y;
			}
			
			nav_msgs::Odometry pose_msg;
			pose_msg.pose.pose.position.x = agent_pose_array_[index].x;
			pose_msg.pose.pose.position.y = agent_pose_array_[index].y;
			pose_pub_vec_[ index- this->virtual_agent_start_index_].publish(pose_msg);
			
			ROS_INFO("%d agent pose x : %f y : %f", index, agent_pose_array_[index].x, agent_pose_array_[index].y);
		}
		
	   //ros::spin();
	   ros::spinOnce();
	   this->loop_rate_->sleep();
	}
	
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_voronoi_virtual_agent_ros");
  ros::NodeHandle nh;
  ros::NodeHandle _nh("~");
  
  DynVoroVirtualAgentRos  dynvorovirtualagent_ros(nh,_nh);
  
  dynvorovirtualagent_ros.run();
 
   return 0;
}
