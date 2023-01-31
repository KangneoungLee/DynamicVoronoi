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

#include "dynamic_voronoi/dynamic_voronoi.h"
#include <chrono>

std::vector<std::string> split(std::string str, char Delimiter) {
    std::istringstream iss(str);             
    std::string buffer;                      
 
    std::vector<std::string> result;
 
 
    while (std::getline(iss, buffer, Delimiter)) {
        result.push_back(buffer);             
    }
 
    return result;
}

float v_travel_sum = 0;
float v_travel_avg = 0;
float v_work_sum = 0;
float v_work_avg = 0;
float density_avg = 1;

int propagation_frame_interval = 100;
std::ofstream img_list;
std::ofstream PartInfoTxt;
std::ofstream MetricTxt;

bool is_optimize_animation = false;

/*constructor and destructor*/
DynamicVoronoi::DynamicVoronoi(unsigned short map_height, unsigned short map_width, float DropOutWeight, float weight_w, float weight_h, float lamda,
                                                            bool is_uniform_density, bool is_dropout_use, bool is_workeff_constraint, bool is_hete_cov_radius, bool is_propa_connected_area, bool is_img_save, unsigned char* vorocellDenseMapExtPtr):
map_height_(map_height),map_width_(map_width), DropOutWeight_(DropOutWeight), weight_w_(weight_w), weight_h_(weight_h), lamda_(lamda),
is_uniform_density_(is_uniform_density), is_dropout_use_(is_dropout_use), is_workeff_constraint_(is_workeff_constraint), is_hete_cov_radius_(is_hete_cov_radius), 
is_propa_connected_area_(is_propa_connected_area), is_img_save_(is_img_save), vorocellDenseMapExtPtr_(vorocellDenseMapExtPtr)
{
	/* datum is for calculating the starting point of coverage*/
	this->x_datum_ = 0;
	this->y_datum_ = 0;
   /*c++ style */	
   this->vorocell_ = new VoroCell*[map_height*map_width];
   
   for (int i = 0; i < map_height*map_width; ++i) {
    this->vorocell_[i] = new VoroCell();
   }
   
   this->vorocellDenseMap_ = new unsigned char[map_height*map_width];
   
   this->vorocellObsMap_ = new unsigned char[map_height*map_width];
   /*c style */
   //this->vorocell_ = (VoroCell*)malloc(sizeof(VoroCell)*((size_t)map_height)*((size_t)map_width));
   //this->vorocellDenseMap_ = (unsigned char*)malloc(sizeof(unsigned char)*((size_t)map_height)*((size_t)map_width));
   
   this->is_propa_completed_ = false;
   this->VoroPartNum_ = 0;
   this->TotalMass_ = 0;
   this->CovrdMass_ = 0;
   this->TotalArea_ = 0;
   this->CovrdArea_ = 0;   
   
   this->InitializeCell();
   this->InitializeDensityMap();
}

DynamicVoronoi::~DynamicVoronoi()
{
	 for(int i =0; i <this->VoroPartNum_;i++)
	 {
		 PartitionInfo* partition_info_single = this->partition_info_[i];
		 PartInfoTxt << "agent_index : " << partition_info_single->part_agentclass_ << "\n";
		 PartInfoTxt << "partition agent pos : " << partition_info_single->part_agent_coor_x_<< "(x) " << partition_info_single->part_agent_coor_y_<< "(y)" << "\n";
		 PartInfoTxt << "partition centroid : " << partition_info_single->part_centroid_x_<< "(x) " << partition_info_single->part_centroid_y_<< "(y)" << "\n";
		 PartInfoTxt << "startng point : " << partition_info_single->StartingPt[0] << "(x) " << partition_info_single->StartingPt[1] << "(y)" << "\n";
		 PartInfoTxt << "mass : " << partition_info_single->part_mass_ << "\n";
		 PartInfoTxt << "v_travel : " << partition_info_single->part_v_travel_ << "\n";
		 PartInfoTxt << "v_work : " << partition_info_single->part_v_work_ << "\n";
	 }

     for (int i = 0; i < this->map_height_*this->map_width_; ++i) delete this->vorocell_[i];

     for (int i = 0; i < this->VoroPartNum_; ++i) delete this->partition_info_[i];

	 img_list.close();
	 PartInfoTxt.close();
	 MetricTxt.close();
	 
	 delete[] this->vorocell_;
	 delete[] this->vorocellDenseMap_;
	 delete[] this->partition_info_;
	 delete[] this->vorocellObsMap_;

}

int DynamicVoronoi::GetIndex(int x, int y)
{
	if((x>=this->map_width_)||(y>=this->map_height_))
	{
		std::cout<<"x or y index is out of bound in DynamicVoronoi :" << " x : " << x << " y : " << y <<std::endl;
		exit(0);
	}
	
	int index = x + y*this->map_width_;
	
	return index;
};

int DynamicVoronoi::MapPointToIndex(float x, float y)
{
	int x_rounded = (int)std::round(x);
	int y_rounded = (int)std::round(y);
	
	int index = this->GetIndex(x_rounded, y_rounded);
	
	return index;
};

VoroCell* DynamicVoronoi::GetSingleCellByIndex(int x, int y)
{
	int index = this->GetIndex(x, y);
	
	return   this->vorocell_[index];
};

VoroCell* DynamicVoronoi::GetSingleCellByPoint(float x, float y)
{
	int index = this->MapPointToIndex(x, y);
	
	return  this->vorocell_[index];
};

bool DynamicVoronoi::PushDatum(float x_datum, float y_datum)
{
	/* datum is for calculating the starting point of coverage*/
	x_datum_ = x_datum;
	y_datum_ = y_datum;
	
}

bool DynamicVoronoi::PushPoint(float x, float y, float v_travel, float v_work, float cov_radius)
{
	/* add agent's parameter set (for example, agent position, velocity.. etc*/
	
	if(x >= this->map_width_)
	{
		return false;
	}
	
	if(y >= this->map_height_)
	{
		return false;
	}
	
	if(v_travel < 0.01)
	{
		v_travel = 0.01;
	}
	
	if(v_work < 0.01)
	{
		v_work = 0.01;
	}

	if(cov_radius < 1)
	{
		cov_radius = 1; 
		std::cout<<" covarge radius should be greater than or equal to 1"<<std::endl;
	}	
	
	if(this->VoroPartNum_ == 0)
	{
		this->partition_info_ = new PartitionInfo*[this->VoroPartNum_+1];
		this->partition_info_[0] = new PartitionInfo();
		this->agentDropoutCheck_ = new bool[this->VoroPartNum_+1];
	}
	else
	{
		
		PartitionInfo** partition_info_Temp = new PartitionInfo*[this->VoroPartNum_+1];
		
		memcpy(partition_info_Temp,this->partition_info_,sizeof(PartitionInfo*)*this->VoroPartNum_);
		partition_info_Temp[this->VoroPartNum_] = new PartitionInfo();
		
		delete[] this->partition_info_;
		
		this->partition_info_ = new PartitionInfo*[this->VoroPartNum_+1];
		
		
		memcpy(this->partition_info_,partition_info_Temp,sizeof(PartitionInfo*)*(this->VoroPartNum_+1));
		
		delete[] partition_info_Temp;
		
		delete[] this->agentDropoutCheck_;
		this->agentDropoutCheck_ = new bool[this->VoroPartNum_+1];		
	}
	
	PartitionInfo* partition_info_single = this->partition_info_[this->VoroPartNum_];
    partition_info_single->part_agentclass_ = this->VoroPartNum_;
	partition_info_single->centroid_momentsum_x_ = 0;
	partition_info_single->centroid_momentsum_y_ = 0;
	partition_info_single->part_mass_ = 0;
	partition_info_single->part_area_ = 0;
	partition_info_single->part_radius_in_area_ = 0;
	partition_info_single->part_centroid_x_ = 0;
	partition_info_single->part_centroid_y_ = 0;
	partition_info_single->part_agent_init_x_ = x;
	partition_info_single->part_agent_init_y_ = y;
	partition_info_single->part_agent_coor_x_ = x;
	partition_info_single->part_agent_coor_y_ = y;
	partition_info_single->part_v_travel_ = v_travel;
	partition_info_single->part_v_work_ = v_work;
	partition_info_single->part_cov_radius_ = cov_radius;
	partition_info_single->agent_parllel_momentsum_x_ = 0;
	partition_info_single->agent_parllel_momentsum_y_ = 0;


	//std::vector<float>  xy_coor;
    
    //xy_coor.push_back(x);
    //xy_coor.push_back(y);	
	
	//this->VoroPartCenters_.insert(std::make_pair(this->VoroPartNum_, xy_coor));

	this->is_propa_completed_ = false;
	
	this->VoroPartNum_ = this->VoroPartNum_ + 1;

    for(int j = 0; j < this->VoroPartNum_; j++)
	{
	    this->agentDropoutCheck_[j] = false;
	}
	
	this->inhibit_dropout_ = false;
	this->inhibit_dropout_cnt = 0;
	
	
	v_travel_sum = v_travel + v_travel_sum;
	v_work_sum = v_work + v_work_sum;
	v_travel_avg = v_travel_sum/this->VoroPartNum_;
	v_work_avg = v_work_sum/this->VoroPartNum_;
	
	return true;
}

void DynamicVoronoi::InitializeCell()
{
	int x,y;
	
	VoroCell* single_cell;
	
	for(x= 0;x<this->map_width_;x++)
	{
		for(y=0;y<this->map_height_;y++)
		{
			
			single_cell = GetSingleCellByIndex(x,y);
			single_cell->is_edge_ = false; 
			single_cell->sq_dist_ = 0xFFFFFFFF;   /* Max value of usigned int value */
			single_cell->agentclass_ = 0xFFFF; /*Max value of unsigned short value */
			single_cell->col_index_ = x;
			single_cell->row_index_ = y; 
			single_cell->state_ = init;
			
		}	
	}
	
	std::cout<<" ************Cell info initialization ***************" <<std::endl;
}

bool DynamicVoronoi::saveDensityMap(std::string file_name)
{
  FILE *fp = fopen(file_name.c_str(), "w");

  if (!fp)
  {
    return false;
  }

  fprintf(fp, "P2\n%u\n%u\n%u\n", map_width_, map_height_, 0xff);
  for (unsigned int iy = 0; iy < map_height_; iy++) 
  {
  for (unsigned int ix = 0; ix < map_width_; ix++) 
    {
		int index = this->GetIndex(ix, iy);
        unsigned char density = this->vorocellDenseMap_[index];
        fprintf(fp, "%d ", density);
    }
    fprintf(fp, "\n");
  }
  fclose(fp);
  return true;
}

bool DynamicVoronoi::saveAgentMap(std::string file_name)
{
  FILE *fp = fopen(file_name.c_str(), "w");

  if (!fp)
  {
    return false;
  }

  fprintf(fp, "P2\n%u\n%u\n%u\n", map_width_, map_height_, 0xff);
  for (unsigned int iy = 0; iy < map_height_; iy++)
  {
    for (unsigned int ix = 0; ix < map_width_; ix++)
    {
		int index = this->GetIndex(ix, iy);
       VoroCell* single_vorocell = this->vorocell_[index];
      fprintf(fp, "%d ", single_vorocell->agentclass_);
    }
    fprintf(fp, "\n");
  }
  fclose(fp);
  return true;
}

bool DynamicVoronoi::Colorized(std::string img_save_dir, std::string label_txt_dir)
{
	 if(is_img_save_ == false) return false; 
     label_text_.open(label_txt_dir);

	 std::string label_text_s;
	 while(std::getline(label_text_,label_text_s))
	 {
	 
	    std::istringstream ss(label_text_s);
	 
	    std::string word;
	 
	    std::vector<int> color_temp;  /*red green blue*/
	 
	    while (ss >> word)
	    {
		    color_temp.push_back(std::stoi(word));
	    }
	    this->color_map_label_.push_back(color_temp);
	 }
	
	 cv::Mat labelImg = cv::Mat::zeros(this->map_width_, this->map_height_, CV_8UC3);
	 
	 for (int x = 0; x<this->map_width_;x++)
	 {
		 for (int y = 0; y<this->map_height_;y++)
		 {
			 VoroCell* vorocell_temp = GetSingleCellByIndex(x, y);
			 int agent_index = vorocell_temp->agentclass_;
			 int density = this->GetDensity(x,y);
			 
			 if(agent_index == 0xFFFF)
			 {
				 labelImg.at<cv::Vec3b>(y,x)[0] = 255;
				 labelImg.at<cv::Vec3b>(y,x)[1] = 255;
				 labelImg.at<cv::Vec3b>(y,x)[2] = 255;

			 }
			 else
			 {
				 std::vector<int> color_load = this->color_map_label_[agent_index]; /*red green blue*/
				 labelImg.at<cv::Vec3b>(y,x)[0] = (unsigned char)color_load.at(0);
				 labelImg.at<cv::Vec3b>(y,x)[1] = (unsigned char)color_load.at(1);
				 labelImg.at<cv::Vec3b>(y,x)[2] = (unsigned char)color_load.at(2);
			 }
			 if(density == 0)  // if the point is obstacle, set wight color
			 {
				 labelImg.at<cv::Vec3b>(y,x)[0] = 255;
				 labelImg.at<cv::Vec3b>(y,x)[1] = 255;
				 labelImg.at<cv::Vec3b>(y,x)[2] = 255;
				 
			 }
		 }
		 
	 }


	for(int i =0; i <this->VoroPartNum_;i++)
	{
		PartitionInfo* partition_info_single = this->partition_info_[i];
		
		unsigned short centroid_x = (unsigned short)std::round(partition_info_single->part_centroid_x_);
		unsigned short centroid_y = (unsigned short)std::round(partition_info_single->part_centroid_y_);
		
		//labelImg.at<cv::Vec3b>(centroid_y,centroid_x)[0] = 0;
		//labelImg.at<cv::Vec3b>(centroid_y,centroid_x)[1] = 0;
		//labelImg.at<cv::Vec3b>(centroid_y,centroid_x)[2] = 255;
		
		unsigned short agent_cen_x = (unsigned short)std::round(partition_info_single->part_agent_coor_x_);
		unsigned short agent_cen_y = (unsigned short)std::round(partition_info_single->part_agent_coor_y_);
		unsigned short radius = (unsigned short) partition_info_single->part_cov_radius_;
		
		labelImg.at<cv::Vec3b>(agent_cen_y,agent_cen_x)[0] = 0;
		labelImg.at<cv::Vec3b>(agent_cen_y,agent_cen_x)[1] = 0;
		labelImg.at<cv::Vec3b>(agent_cen_y,agent_cen_x)[2] = 0;
		
		if(this->is_hete_cov_radius_) cv::circle(labelImg, cv::Point(agent_cen_x, agent_cen_y), radius, cv::Scalar(0,0,0));
		
		unsigned short StartingPt_x = (unsigned short)std::round(partition_info_single->StartingPt[0]);
		unsigned short StartingPt_y = (unsigned short)std::round(partition_info_single->StartingPt[1]);
		
		/* Starting point visualization for agricutre field*/
		//cv::rectangle( labelImg, cv::Point( StartingPt_x - 1, StartingPt_y - 1), cv::Point( StartingPt_x + 1, StartingPt_y + 1), cv::Scalar( 255, 255, 255 ), cv::FILLED, cv::LINE_8);
		
		//labelImg.at<cv::Vec3b>(StartingPt_y,StartingPt_x)[0] = 0;
		//labelImg.at<cv::Vec3b>(StartingPt_y,StartingPt_x)[1] = 0;
		//labelImg.at<cv::Vec3b>(StartingPt_y,StartingPt_x)[2] = 0;

	}


	 cv::imwrite(img_save_dir, labelImg);
	 
	 
	 std::string gray = "_edge";
	 std::string extention = ".png";
	 std::vector<std::string> image_save_dir_split = split(img_save_dir,'.');
	 std::string img_save_edge_dir = image_save_dir_split[0] + gray + extention;
	 
	 std::cout << img_save_edge_dir << std::endl;
	 
	 cv::Mat labelImg_gray, labelImg_edge;
	 cv::cvtColor( labelImg, labelImg_gray, CV_RGB2GRAY );
	 cv::Canny(labelImg_gray, labelImg_edge, 1, 5, 3);
	 
	 uchar * pt = labelImg_edge.data;
	 for(int i = 0; i <labelImg_edge.rows; i++)
     {
        for(int j = 0; j < labelImg_edge.cols; j++)
        {
			unsigned char data = pt[labelImg_edge.cols * i + j];
			if(data == 255)
			{
				pt[labelImg_edge.cols * i + j] = 0;
			}
			else if(data == 0)
			{
				pt[labelImg_edge.cols * i + j] = 255;
			}
		}
     }		
	 
	 cv::imwrite(img_save_edge_dir, labelImg_edge);
	 
	 return true;
}


bool DynamicVoronoi::saveSingleVoro(std::string file_name, int x, int y)
{
  FILE *fp = fopen(file_name.c_str(), "w");
  
  if (!fp)
  {
    return false;
  }

  fprintf(fp, "P2\n%u\n%u\n%u\n", x, y, 0xff);
 
  int index = this->GetIndex(x, y);
      VoroCell* single_vorocell = this->vorocell_[index];
      fprintf(fp, "is_edge_ : %d \n", single_vorocell->is_edge_);
      fprintf(fp, "sq_dist_ : %u  \n", single_vorocell->sq_dist_);
	  fprintf(fp, "agentclass_ : %d  \n", single_vorocell->agentclass_);
	  fprintf(fp, "col_index_ : %d  \n", single_vorocell->col_index_);
	  fprintf(fp, "row_index_ : %d  \n", single_vorocell->row_index_);
	  fprintf(fp, "state_ : %d  \n", single_vorocell->state_);

   fclose(fp);
  return true;
}


void DynamicVoronoi::InitializeDensityMap()
{
	int x,y;
	int area_wo_density = 0;
	unsigned char density;
	
	if((this->is_uniform_density_ == true)||(this->vorocellDenseMapExtPtr_ == NULL))
	{
	   density = 1;
	   for(x= 0;x<this->map_width_;x++)
	   {
		   for(y=0;y<this->map_height_;y++)
		   {
			    int index = this->GetIndex(x, y);
			    this->vorocellDenseMap_[index] = density;
			
			    this->TotalMass_ = this->TotalMass_ + density;
				this->TotalArea_ = this->TotalArea_ + 1;
				
				if(density <= 0)   this->vorocellObsMap_[index] = 1;
				else   this->vorocellObsMap_[index] = 0;
		   }	
	   }
	   density_avg = 1;
	}
	else
	{
		memcpy(this->vorocellDenseMap_, this->vorocellDenseMapExtPtr_, sizeof(unsigned char)*(this->map_height_)*(this->map_width_));
	
		for(x= 0;x<this->map_width_;x++)
		{
			for(y=0;y<this->map_height_;y++)
			{
				density = this->GetDensity(x,y);
				int index = this->GetIndex(x, y);
				//std::cout<<" row: "<< y << " col: "<< x << " density: "<< (float)density <<std::endl;
				this->TotalMass_ = this->TotalMass_ + density;
				this->TotalArea_ = this->TotalArea_ + 1;
				
				area_wo_density = area_wo_density + 1;
				if(density <= 0)   this->vorocellObsMap_[index] = 1;
				else   this->vorocellObsMap_[index] = 0;
			}
		}
	    density_avg = 	this->TotalMass_/area_wo_density;
	}
	
	this->inhibit_dropout_ = false;
	this->inhibit_dropout_cnt = 0;
	std::cout<<" ************Initialize density map success ***************" <<std::endl;
}


void DynamicVoronoi::UpdateDensityMap(unsigned char* vorocellDenseMapExtPtr)
{
	memcpy(this->vorocellDenseMap_, vorocellDenseMapExtPtr, sizeof(unsigned char)*(this->map_height_)*(this->map_width_));
    //memcopy
	this->TotalMass_ = 0;
	
	for(int x= 0;x<this->map_width_;x++)
	{
		for(int y=0;y<this->map_height_;y++)
		{
			unsigned char  density = this->GetDensity(x,y);
			int index = this->GetIndex(x, y);
			this->TotalMass_ = this->TotalMass_ + density;
				
			if(density <= 0)   this->vorocellObsMap_[index] = 1;
			else  this->vorocellObsMap_[index] = 0;
		}
	}
}

unsigned char DynamicVoronoi::GetDensity(int x, int y)
{
	int index = this->GetIndex(x, y);
	return this->vorocellDenseMap_[index];
}

float DynamicVoronoi::CoverageMetric()
{
	if(this->is_hete_cov_radius_==true)
	{
		float var = 0;
		for(int i =0; i <this->VoroPartNum_;i++)
	   {
		   PartitionInfo* partition_info_single = this->partition_info_[i];
		   float in_area = partition_info_single->part_radius_in_area_;
		   float radius = partition_info_single->part_cov_radius_;
		   float max_area = PI*radius*radius;
		   float temp_var = in_area/max_area - 1.0;
		   var = std::abs(temp_var) + var;
	   }
	   var = var/this->VoroPartNum_;
	   return var;

	}
	else
	{
	   float ave_mass = this->CovrdMass_/this->VoroPartNum_;
	   float var = 0;
	   for(int i =0; i <this->VoroPartNum_;i++)
	   {
		   PartitionInfo* partition_info_single = this->partition_info_[i];
		   float mass = partition_info_single->part_mass_;
		   float temp_var = mass/ave_mass - 1.0;
		   var = std::abs(temp_var) + var;
	   }
	   float NotCovrdMass = this->TotalMass_ - this->CovrdMass_;
	   //float NotCovrdMass = 0;
	   
	   if( NotCovrdMass == 0)   var = var/this->VoroPartNum_;
	   else var = var/this->VoroPartNum_ + NotCovrdMass/(ave_mass*this->VoroPartNum_);
	
	   return var;		
		
	}
}

void DynamicVoronoi::MainOfflineProcess(bool is_optimize_animation, std::string img_dir, std::string label_dir, int max_step_size, float terminate_criteria)
{

    std::cout<<" DynamicVoronoi::MainOfflineProcess start " << std::endl;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	
	
	if(is_optimize_animation == true)
	{
	   std::string list_dir = img_dir + "img_file_list.txt";
	   img_list.open(list_dir);   
	}
	std::string  info_dir = img_dir + "partition_info.txt";
	PartInfoTxt.open(info_dir);
	
    std::string metric_dir =  "/home/kangneoung/sw_repo/dynamic_voronoi/src/dynamic_voronoi/test/opt_animation/metric.txt";
	MetricTxt.open(metric_dir);
	 MetricTxt <<  "cc_metric_" << "   cc_metric_diff   " <<"   cc_metric_diff_final   " << " rate for final : " << cc_metric_diff_rate_ << std::endl;   


     std::string img_save_dir = img_dir + "init_pose_map.png";
     this->Colorized(img_save_dir,label_dir);

	this->ExpandedVoronoi();
	this->CentroidCal();
    this->MoveAgents();	
	
	if(is_optimize_animation == true)
	{
		 std::string img_save_dir = img_dir + "map" +".png";
		 this->Colorized(img_save_dir,label_dir);
		 img_list<<"map.png"<<std::endl;
	 }
	   

	   
	this->InitializeCell(); 
	
	cc_metric_prev_ = cc_metric_;
	cc_metric_ = this->CoverageMetric();
	   
	float cc_metric_dif_init =  cc_metric_prev_ - cc_metric_;
	MetricTxt << "  "<<cc_metric_ <<"    "<< cc_metric_dif_init<<std::endl;
	 
	for(int i = 0; i < max_step_size; i++)
	{
       
	   if(this->is_dropout_use_ == false)   this->inhibit_dropout_ = true; //forced inhibition for dropout
    
	   if ((this->inhibit_dropout_ == false)&&(this->cc_metric_diff_final_ <= terminate_criteria*2)&&(i>0))     this->inhibit_dropout_ = true;
	   //if(this->inhibit_dropout_cnt >=  this->VoroPartNum_)  this->inhibit_dropout_ = true;
	   if(this->inhibit_dropout_ == false)  this->dropout_active_ = this->AgentDropOut(); /*AgentDropOut function determines if dropout_active_ is false or true */   

	   if (this->dropout_active_ == true)
	   {
		   this->inhibit_dropout_cnt ++;
		   this->ExpandedVoronoi();
	       this->CentroidCal();
		   this->MoveAgents();
		   
		   if(is_optimize_animation == true)
	       {
		   	    std::string img_save_dir = img_dir + "map" + std::to_string(i) + "dropout" +".png";
			    this->Colorized(img_save_dir,label_dir);
			    //img_list<<"map" + std::to_string(i) +".png"<<std::endl;
	       }
		   
		   this->InitializeCell();
		   
		   /* Initialize the dropout array*/
		   for(int k = 0; k < this->VoroPartNum_; k++)
	       {
			    this->agentDropoutCheck_[k] = false;
		   }
		   
		   this->ExpandedVoronoi();
	       this->CentroidCal();
		   this->MoveAgents();
		   
		   if(is_optimize_animation == true)
	       {
		   	    std::string img_save_dir = img_dir + "map" + std::to_string(i) + "after_dropout" +".png";
			    this->Colorized(img_save_dir,label_dir);
			    //img_list<<"map" + std::to_string(i) +".png"<<std::endl;
	       }
	   }
	   else
	   {
		   this->ExpandedVoronoi();
	       this->CentroidCal();
		   this->MoveAgents();   
	   }
	   
	   this->dropout_active_ = false;
	   
	   
	   if(is_optimize_animation == true)
	   {
		  std::string img_save_dir = img_dir + "map" + std::to_string(i) +".png";
		  this->Colorized(img_save_dir,label_dir);
		  img_list<<"map" + std::to_string(i) +".png"<<std::endl;
	   }
	   
	   this->InitializeCell();

	   cc_metric_prev_ = cc_metric_;
	   cc_metric_ = this->CoverageMetric();
	   
	   float cc_metric_diff =  cc_metric_prev_ - cc_metric_;
	   this->cc_metric_diff_final_ =cc_metric_diff_rate_*cc_metric_diff + (1-cc_metric_diff_rate_)*this->cc_metric_diff_prev_;
	   this->cc_metric_diff_prev_ = this->cc_metric_diff_final_;
	   
	   MetricTxt << "  "<<cc_metric_ <<"    "<< cc_metric_diff<<"   "<<cc_metric_diff_final_<<std::endl;
	   if ((this->cc_metric_diff_final_ <= terminate_criteria)&&(this->inhibit_dropout_ == true)) 
	   {
		     this->ExpandedVoronoi();

			 if(is_optimize_animation == true)
	         {
		   	      std::string img_save_dir = img_dir + "map" + std::to_string(i+1) +".png";
			      this->Colorized(img_save_dir,label_dir);
			      img_list<<"map" + std::to_string(i+1) +".png"<<std::endl;
	         }
			 
		     std::cout<<" MainOptProcess : terminate criteria is satisfied " << std::endl;
		     break;
	   }
	   
	   
	   
	}
	
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    std::cout<<" DynamicVoronoi::MainOptProcess end " << std::endl;

}


bool DynamicVoronoi::ExpandedVoronoi(bool is_propagation_animation, std::string img_dir, std::string label_dir)
{
	int agent_index;
	VoroCell* vorocell_temp;

	std::cout<<" ******************** DynamicVoronoi::ExpandedVoronoi start ******************* " << std::endl;
	
	for(int i =0; i <this->VoroPartNum_;i++)
	{
		
		float v_travel;
	    float v_work;
		int StartingPt_x;
		int StartingPt_y;
		
		PartitionInfo* partition_info_single = this->partition_info_[i];
		agent_index = partition_info_single->part_agentclass_;
		float radius = partition_info_single->part_cov_radius_;
		v_travel = partition_info_single->part_v_travel_;
		v_work = partition_info_single->part_v_work_;
		int mass = partition_info_single->part_mass_;
		float init_pos_x = partition_info_single->part_agent_init_x_;
		float init_pos_y = partition_info_single->part_agent_init_y_;
		StartingPt_x = partition_info_single->StartingPt[0];
		StartingPt_y = partition_info_single->StartingPt[1];
		
	    /* agentDropoutCheck == true means that the agent should be ignored when propagating */ 
		if(this->agentDropoutCheck_[agent_index] == true)
		{
			continue;
		}
		 
		unsigned short agent_cen_x = (unsigned short)std::round(partition_info_single->part_agent_coor_x_);
		unsigned short agent_cen_y = (unsigned short)std::round(partition_info_single->part_agent_coor_y_);
		
		vorocell_temp = GetSingleCellByIndex(agent_cen_x,agent_cen_y);
			
		vorocell_temp->sq_dist_ = 0;
		vorocell_temp->agentclass_ = agent_index;
		vorocell_temp->agent_cen_x_ = agent_cen_x;
		vorocell_temp->agent_cen_y_ = agent_cen_y;
		vorocell_temp->state_ = completed;
		
		//int sq_dist_prev = vorocell_temp->sq_dist_;
		
		int col = vorocell_temp->col_index_; // equavalent to x
		int row = vorocell_temp->row_index_; // equavalent to y
				
		// Propagate the cells from the initial cell for each agent
        this->Propagatation(agent_index, agent_cen_x, agent_cen_y, col, row, radius, StartingPt_x, StartingPt_y, init_pos_x, init_pos_y, v_travel, v_work, mass);		
	}


	
	int i = 0;
	int f = 0;
    
	if(is_propagation_animation == true)
	{		
       //std::string img_dir = "/home/kangneoung/sw_repo/dynamic_voronoi/src/dynamic_voronoi/test/animation/";
	   std::string list_dir = img_dir + "img_file_list.txt";
	   img_list.open(list_dir);   
	}
	
   // Propagate the cells from the queued cell 	
	std::vector<int>  xy_point_int;
 
	while (this->ProccessingQueue_.size()>0)
	{
		xy_point_int = this->ProccessingQueue_.front();
		this->ProccessingQueue_.pop();
		
	    //std::cout<<"xy_point_int "<<xy_point_int[0]<<"  "<<xy_point_int[1]<<" queue size : "<<this->ProccessingQueue_.size()<<" empty :"<<this->ProccessingQueue_.empty()<<std::endl;
		
		vorocell_temp = GetSingleCellByIndex(xy_point_int[0], xy_point_int[1]);
		vorocell_temp->state_ = completed;
		
		unsigned short agent_cen_x = vorocell_temp->agent_cen_x_;
		unsigned short agent_cen_y = vorocell_temp->agent_cen_y_;
	
		agent_index = vorocell_temp->agentclass_;
		
		PartitionInfo* partition_info_single = this->partition_info_[agent_index];
		float radius = partition_info_single->part_cov_radius_;
		float v_travel = partition_info_single->part_v_travel_;
	    float v_work = partition_info_single->part_v_work_;
		float init_pos_x = partition_info_single->part_agent_init_x_;
		float init_pos_y = partition_info_single->part_agent_init_y_;
		int StartingPt_x = partition_info_single->StartingPt[0];
		int StartingPt_y = partition_info_single->StartingPt[1];
		int mass = partition_info_single->part_mass_;
		
		/* agentDropoutCheck == true means that the agent should be ignored when propagating */ 
		if(this->agentDropoutCheck_[agent_index] == true)
		{
			continue;
		}
			
		int col = vorocell_temp->col_index_; // equavalent to x
		int row = vorocell_temp->row_index_; // equavalent to y
		
		// Propagate the cells from the initial cell for each agent
        this->Propagatation(agent_index, agent_cen_x, agent_cen_y, col, row, radius, StartingPt_x, StartingPt_y, init_pos_x, init_pos_y, v_travel, v_work, mass);
		
		if((is_propagation_animation==true)&&((i%propagation_frame_interval) == 0))
		{	
	
	        this->CentroidCal();
	
		    //std::string img_dir = "/home/kangneoung/sw_repo/dynamic_voronoi/src/dynamic_voronoi/test/animation/";
	        //std::string label_dir = "/home/kangneoung/sw_repo/dynamic_voronoi/src/dynamic_voronoi/label/label.txt";			
			std::string img_save_dir = img_dir + "map" + std::to_string(f) +".png";
			
			this->Colorized(img_save_dir,label_dir);
			
			img_list<<"map" + std::to_string(f) +".png"<<std::endl;
			
			f = f +1;			
		}
        i = i +1;
	}
	
	
	std::cout<<" ******************** DynamicVoronoi::ExpandedVoronoi end ******************* " << std::endl;

    this->is_propa_completed_ = true;

	return true;

}

void DynamicVoronoi::Propagatation(int agent_index, unsigned short agent_cen_x, unsigned short agent_cen_y, int col, int row, float radius, int StartingPt_x, int StartingPt_y, 
                                                                                   float init_pos_x, float init_pos_y, float v_travel, float v_work, int mass)
{
	VoroCell* vorocell_temp;
	std::vector<int> queue_xy;
     
	 float vel_cost;
	 float work_cost;
	 float travel_time;
	 float work_time;
	 
	 int col_move[4] = {1, 0, -1, 0};
	 int row_move[4] = {0, 1, 0, -1};
	 
	 if (this->is_workeff_constraint_ == true)
     {
		 
		 travel_time = (std::abs(StartingPt_x - init_pos_x) + std::abs(StartingPt_y - init_pos_y))/v_travel;
		 work_time = (mass/density_avg)/v_work;
		 
		 vel_cost = travel_time*v_travel_avg*5;
		 work_cost = 0; //work_cost = work_time*v_work_avg/3;

		 //std::cout<<" travel_dis : "<<(std::abs(StartingPt_x - init_pos_x) + std::abs(StartingPt_y - init_pos_y)) <<" v_travel : "<<v_travel<<" v_travel_avg : "<<v_travel_avg<<"vel_cost : "<<vel_cost<<std::endl;
		 //std::cout<<" mass : "<<mass<<"density_avg : "<<density_avg<<" v_work : "<<v_work<<" v_work_avg : "<<v_work_avg<<"work_cost : "<<work_cost<<std::endl;
	 }
	 else
	 {
		 vel_cost = 0;
		 work_cost = 0;
	 }
	 
     for (int k = 0; k <4; k ++)
	 {
		 int new_col = col+col_move[k];
		 int new_row = row+row_move[k];
		 
		 if((new_col < 0)||(new_row < 0)||(new_col == this->map_height_)||(new_row == this->map_width_)) continue;
		 
		 vorocell_temp = GetSingleCellByIndex(new_col, new_row);
		 
		float ObsPathPenalty = CulObsCell(agent_cen_x, agent_cen_y, new_col, new_row);
		 
		if((is_propa_connected_area_ == true)&&(ObsPathPenalty > 1))
		{
			/* No action */
		}
		else{
			float sq_dist_raw;
			float sq_cost;
			if(this->is_hete_cov_radius_ == false)
			{
		        sq_dist_raw = this->weight_w_*(agent_cen_x - (new_col))*(agent_cen_x - (new_col)) + this->weight_h_*(agent_cen_y - new_row)*(agent_cen_y - new_row); 
			    sq_cost  = this->lamda_*this->lamda_*ObsPathPenalty*ObsPathPenalty + vel_cost + work_cost;
			}
			else
			{
				sq_dist_raw = (this->weight_w_*(agent_cen_x - (new_col))*(agent_cen_x - (new_col)) + this->weight_h_*(agent_cen_y - new_row)*(agent_cen_y - new_row))/(radius*radius); 
			    sq_cost  = this->lamda_*this->lamda_*ObsPathPenalty*ObsPathPenalty + vel_cost + work_cost;
			}
			
			float sq_dist_new = sq_dist_raw + sq_cost;
			
		    if(vorocell_temp->state_ == init)
		    {
			    vorocell_temp->agentclass_ = agent_index;
		        vorocell_temp->agent_cen_x_ = agent_cen_x;
		        vorocell_temp->agent_cen_y_ = agent_cen_y;	
			    vorocell_temp->sq_dist_ = sq_dist_new;
			    vorocell_temp->state_ = queued;
			    queue_xy.push_back(new_col);
			    queue_xy.push_back(new_row);
			    this->ProccessingQueue_.push(queue_xy);
			    queue_xy.clear();		
		    }
		    else
		    {
			    if(vorocell_temp->sq_dist_ >sq_dist_new)
			    {
			       vorocell_temp->is_edge_ = false;
			       vorocell_temp->agentclass_ = agent_index;
	               vorocell_temp->agent_cen_x_ = agent_cen_x;
	               vorocell_temp->agent_cen_y_ = agent_cen_y;
			       vorocell_temp->sq_dist_ = sq_dist_new;
			       vorocell_temp->state_ = queued;
			       queue_xy.push_back(new_col);
			       queue_xy.push_back(new_row);
			       this->ProccessingQueue_.push(queue_xy);
			       queue_xy.clear();
			    }
                else if(vorocell_temp->sq_dist_  == sq_dist_new)
			    {
				    if(vorocell_temp->agentclass_ < agent_index)
				    {
					    vorocell_temp->agentclass_ = agent_index;
		                vorocell_temp->agent_cen_x_ = agent_cen_x;
		                vorocell_temp->agent_cen_y_ = agent_cen_y;
			            vorocell_temp->sq_dist_ = sq_dist_new;	
				    }
				    //vorocell_temp->is_edge_ = true;
				    //std::cout<<"line 569"<<" col :"<<col-1<<" row :"<<row<<std::endl;
			    }
		    }
		}
	 }
}

void DynamicVoronoi::CentroidCal()
{
	VoroCell* vorocell_temp;
	PartitionInfo* partition_info_single;
	
	std::cout<<" ******************** DynamicVoronoi::CentroidCal start ******************* " << std::endl;
	
	/* initialize variables for centroid calculation */
	for(int i =0; i <this->VoroPartNum_;i++)
	{
		PartitionInfo* partition_info_single = this->partition_info_[i];
		partition_info_single->part_mass_ = 0;
		partition_info_single->part_area_ = 0;
		partition_info_single->part_radius_in_area_ = 0;
		partition_info_single->centroid_momentsum_x_ = 0;
		partition_info_single->centroid_momentsum_y_ = 0;
		partition_info_single->LeftFrontPt[0] = 0;
		partition_info_single->LeftFrontPt[1] = 0;
		partition_info_single->LeftFrontPtDis = this->map_width_ + this->map_height_;
		partition_info_single->RightFrontPt[0] = 0;
		partition_info_single->RightFrontPt[1] = 0;
		partition_info_single->RightFrontPtDis = this->map_width_ + this->map_height_;		
		partition_info_single->LeftRearPt[0] = 0;
		partition_info_single->LeftRearPt[1] = 0;
		partition_info_single->LeftRearPtDis = this->map_width_ + this->map_height_;
		partition_info_single->RightRearPt[0] = 0;
		partition_info_single->RightRearPt[1] = 0;
		partition_info_single->RightRearPtDis = this->map_width_+ this->map_height_;		
		partition_info_single->StartingPt[0] = 0;
		partition_info_single->StartingPt[1] = 0;
		partition_info_single->StartingPtDis = this->map_width_+ this->map_height_;			
	}

    /* calcuate variables for centroid  */
	for(int col= 0;col<this->map_width_;col++)
	{
		for(int row=0;row<this->map_height_;row++)
		{
			vorocell_temp = GetSingleCellByIndex(col, row);
			unsigned short agentclass = vorocell_temp->agentclass_;
			
			if(agentclass == 0xFFFF) continue;
			
			partition_info_single = this->partition_info_[agentclass];
			unsigned char density_temp = GetDensity(col, row);
			partition_info_single->part_mass_ = partition_info_single->part_mass_ + 1*density_temp;
            partition_info_single->part_area_ = partition_info_single->part_area_ + 1;			
			partition_info_single->centroid_momentsum_x_ = partition_info_single->centroid_momentsum_x_ + col*1*density_temp;
			partition_info_single->centroid_momentsum_y_ = partition_info_single->centroid_momentsum_y_ + row*1*density_temp;	
			
			/*calculate starting point */
			int StartingPtDisTemp = std::abs(this->x_datum_ - col) + std::abs(this->y_datum_ - row);			

			if(StartingPtDisTemp <= partition_info_single->StartingPtDis)  
			{
				partition_info_single->StartingPt[0] = col;
				partition_info_single->StartingPt[1] = row;
				partition_info_single->StartingPtDis = StartingPtDisTemp;				
			}
		}
		
	}

	/* calculate centroid */
	for(int i = 0; i < this->VoroPartNum_; i++)
	{
		partition_info_single = this->partition_info_[i];
		
		if(partition_info_single->part_mass_ > 0)
		{
		   partition_info_single->part_centroid_x_ = partition_info_single->centroid_momentsum_x_/partition_info_single->part_mass_;
		   partition_info_single->part_centroid_y_ = partition_info_single->centroid_momentsum_y_/partition_info_single->part_mass_;			
		}
		else
		{
		   partition_info_single->part_centroid_x_ = partition_info_single->part_agent_coor_x_;
		   partition_info_single->part_centroid_y_ = partition_info_single->part_agent_coor_y_;	
		}			
		std::cout<< "class num : "<<i<<" centroid x : "<<partition_info_single->part_centroid_x_<<" centroid y : "<<partition_info_single->part_centroid_y_<<std::endl;
	}
    
	if(this->is_hete_cov_radius_ == true)
	{
	   for(int col= 0;col<this->map_width_;col++)
	   {
		   for(int row=0;row<this->map_height_;row++)
		   {
			   vorocell_temp = GetSingleCellByIndex(col, row);
			   unsigned short agentclass = vorocell_temp->agentclass_;
			
			   if(agentclass == 0xFFFF) continue;
			
			   partition_info_single = this->partition_info_[agentclass];
			   float centroid_x = partition_info_single->part_centroid_x_;
			   float centroid_y = partition_info_single->part_centroid_y_;
			   float radius = partition_info_single->part_cov_radius_;
			   float dis_sq =  (centroid_x - (float)col)*(centroid_x - (float)col) + (centroid_y - (float)row)*(centroid_y - (float)row); 
			   
			   if(dis_sq<=(radius*radius))
			   { partition_info_single->part_radius_in_area_ = partition_info_single->part_radius_in_area_ +1; }
		   }
	   }
	}


	this->CovrdMass_ = 0;
	this->CovrdArea_ = 0;
	this->CvdAreaDivMaxR_ = 0;
	
	for(int i = 0; i < this->VoroPartNum_; i++)
	{
		partition_info_single = this->partition_info_[i];
				
		float mass = partition_info_single->part_mass_;
		this->CovrdMass_ = this->CovrdMass_ + mass;
		
		float area = partition_info_single->part_area_;
		this->CovrdArea_ = this->CovrdArea_ + area;
		
		float in_area = partition_info_single->part_radius_in_area_;
		float radius = partition_info_single->part_cov_radius_;
		this->CvdAreaDivMaxR_ = this->CvdAreaDivMaxR_ + area/(PI*radius*radius);
	}
	//std::cout<<" ******************** DynamicVoronoi::CentroidCal end ******************* " << std::endl;

}

void DynamicVoronoi::MoveAgents()
{
	PartitionInfo* partition_info_single;
	int cnt_project_req = 0;
	std::cout<<" ************DynamicVoronoi::MoveAgents start ***************" <<std::endl;
	
	for(int i = 0; i < this->VoroPartNum_; i++)
	{
		bool is_Partition_inObs = false;
		partition_info_single = this->partition_info_[i];
		
		if( (partition_info_single->part_mass_ > 0))
		{
			is_Partition_inObs = false;
		}
		else
		{
			is_Partition_inObs = true;
			cnt_project_req++;
		}
		
		std::cout << "agent index :" << i << " partition_info_single->part_mass : "<< partition_info_single->part_mass_ <<" is_Partition_inObs : "<< is_Partition_inObs << std::endl;
		
		if(is_Partition_inObs == true)
		{
			
			bool is_proj_success  = false;
            float ref_dist_sq = this->map_width_*this->map_width_ + this->map_height_*this->map_height_; 
		
		    float init_agent_coor_x = partition_info_single->part_agent_coor_x_;
		    float init_agent_coor_y = partition_info_single->part_agent_coor_y_;
		
		    bool is_proj_success_temp = this->FindNearPtNonObs(partition_info_single, init_agent_coor_x, init_agent_coor_y, -1, 0, &ref_dist_sq);  // go to west to find non obstacle region
		    is_proj_success = is_proj_success_temp | is_proj_success;
		    is_proj_success_temp = this->FindNearPtNonObs(partition_info_single, init_agent_coor_x, init_agent_coor_y, 1, 0, &ref_dist_sq);  // go to east to find non obstacle region
		    is_proj_success = is_proj_success_temp | is_proj_success;
		    is_proj_success_temp = this->FindNearPtNonObs(partition_info_single, init_agent_coor_x, init_agent_coor_y, 0, -1, &ref_dist_sq);  // go to north to find non obstacle region
		    is_proj_success = is_proj_success_temp | is_proj_success;
		    is_proj_success_temp = this->FindNearPtNonObs(partition_info_single, init_agent_coor_x, init_agent_coor_y, 0, 1, &ref_dist_sq);  // go to south to find non obstacle region
		    is_proj_success = is_proj_success_temp | is_proj_success;
			
			if(is_proj_success == false)
			{
				is_proj_success_temp = FindNearPtNonObs_R2(partition_info_single, init_agent_coor_x, init_agent_coor_y, &ref_dist_sq); 
				is_proj_success = is_proj_success_temp | is_proj_success;
			}
			
			if(is_proj_success == false)
			{
				//partition_info_single->part_agent_coor_x_ = 0;
				//partition_info_single->part_agent_coor_y_ = 0;
				std::cout<<" Projection failed, check the density map"<<std::endl;
			}
			else
			{
			   /* after running the FindNearPtNonObs function, the agent position may be relocated */
			   float new_agent_coor_x = partition_info_single->part_agent_coor_x_;
		       float new_agent_coor_y = partition_info_single->part_agent_coor_y_;		
			}
		}
		else
		{
			float mass = partition_info_single->part_mass_;
			float agent_coor_x = partition_info_single->part_agent_coor_x_;
			float agent_coor_y = partition_info_single->part_agent_coor_y_;
			float centroid_x = partition_info_single->part_centroid_x_;
			float centroid_y = partition_info_single->part_centroid_y_;
			/*agent coordinate update using gradient descent */ /*instead using the partition mass directly, normalized mass was used (mass/Total mass) or inverse normalized mass ((Total mass - mass)/Total mass)*/
			
			/*float coefficient = (this->TotalMass_- mass)/this->TotalMass_;
			/float min_coeff = 0.5;
			
			if(coefficient < min_coeff)
			{
				coefficient = min_coeff;
			}
			coefficient = min_coeff;*/
		    //agent_coor_x = agent_coor_x - 2*coefficient*(agent_coor_x - centroid_x);     
		   // agent_coor_y = agent_coor_y - 2*coefficient*(agent_coor_y - centroid_y);
		   agent_coor_x = centroid_x;
		   agent_coor_y = centroid_y;
					/*boundary check*/
		    if(agent_coor_x<1)
				
		    {
			    agent_coor_x = 1;
		    }
		    else if(agent_coor_x>(this->map_width_-1))
		    { 
			    agent_coor_x = this->map_width_-1;
		    }
		
		    if(agent_coor_y<1)
		    {
			    agent_coor_y = 1;
		    }
		    else if(agent_coor_y>(this->map_height_-1))
		    { 
			    agent_coor_y = this->map_height_-1;
		    }
		
		    /* allocate new agent coordinate */
		    partition_info_single->part_agent_coor_x_ = agent_coor_x;
		    partition_info_single->part_agent_coor_y_ = agent_coor_y;
			
		    /* insert the index of agent which is on non obstacle region */
			float density = this->GetDensity((int)std::round(partition_info_single->part_agent_coor_x_), (int)std::round(partition_info_single->part_agent_coor_y_));
			if(density>0)
			{
							
				std::vector<float> AgentCoor_temp;
				AgentCoor_temp.push_back((int)std::round(partition_info_single->part_agent_coor_x_));
				AgentCoor_temp.push_back((int)std::round(partition_info_single->part_agent_coor_y_));
								
			    if(this->AgentCoorOpenSp_.empty()) this->AgentCoorOpenSp_.push_back(AgentCoor_temp);
			    	
			}
		    else
			{

                this->AgentPosPostCheck(partition_info_single);				
				
				std::vector<float> AgentCoor_temp;
				AgentCoor_temp.push_back((int)std::round(partition_info_single->part_agent_coor_x_));
				AgentCoor_temp.push_back((int)std::round(partition_info_single->part_agent_coor_y_));
				
				float density_temp = this->GetDensity((int)std::round(partition_info_single->part_agent_coor_x_), (int)std::round(partition_info_single->part_agent_coor_y_));  /* somtimes, the post pos check doesn't work. density should be checked again*/
				
			    if((this->AgentCoorOpenSp_.empty())&&(density_temp > 0)) this->AgentCoorOpenSp_.push_back(AgentCoor_temp);
			}
		}
		

		std::cout<<" agent number : "<<i<<" new agent coordinate x : "<<partition_info_single->part_agent_coor_x_<<" new agent coordinate y : "<<partition_info_single->part_agent_coor_y_<<std::endl;
		
	}
	
	if(cnt_project_req==0)
	{
		this->AgentCoorOpenSp_.clear();
		
	}
}

float DynamicVoronoi::CulObsCell(unsigned short agent_cen_x, unsigned short agent_cen_y, int col, int row)
{

	float increment_x;
	float increment_y;	
	int sign_conv;
	float CulObs = 0;
	
    int index_init = this->GetIndex(col, row);		
	CulObs = CulObs + (float)this->vorocellObsMap_[index_init];
	
	float x_diff = (float)col - agent_cen_x;
	float y_diff = (float)row - agent_cen_y;
	
	 if((x_diff<1)&&(x_diff>-1)&&(y_diff<1)&&(y_diff>-1)) //prevent singularity
	 {
		   return CulObs;
	  }
	
     if(std::abs(y_diff) > std::abs(x_diff))
     {
		 increment_x = x_diff/y_diff;
		 
		 if(y_diff >= 0)
	     { 
		     increment_y = 1;
			 sign_conv = 1;
	     }
	     else
	     {
		     increment_y = -1;
			 sign_conv = -1;
	     }
		 
		 float pos_x = agent_cen_x;
		 
	     for(float pos_y = agent_cen_y; (row - pos_y)*sign_conv > 0;)
	     {
		     pos_x = pos_x + increment_x*sign_conv;
		     pos_y = pos_y + increment_y;
		
		     if((pos_x < 0)||(pos_x >= this->map_width_)||(pos_y < 0)||(pos_y >= this->map_height_))
		     {
			     continue;
		     }
		
		     int index = this->GetIndex((int)std::round(pos_x), (int)std::round(pos_y));		
		     CulObs = CulObs + (float)this->vorocellObsMap_[index]; 		 
	     }
     }
     else
     {
		 increment_y = y_diff/x_diff;
		 
		 if(x_diff>= 0)
	     { 
		     increment_x = 1;
			 sign_conv = 1;
	     }
	     else
	     {
		     increment_x = -1;
			 sign_conv = -1;
	     }
		 
		 float pos_y = agent_cen_y;
		 
	     for(float pos_x = agent_cen_x; (col - pos_x)*sign_conv > 0;)
	     {
		     pos_x = pos_x + increment_x;
		     pos_y = pos_y + increment_y*sign_conv;
		
		     if((pos_x < 0)||(pos_x >= this->map_width_)||(pos_y < 0)||(pos_y >= this->map_height_))
		     {
			     continue;
		     }
		
		     int index = this->GetIndex((int)std::round(pos_x), (int)std::round(pos_y));		
		     CulObs = CulObs + (float)this->vorocellObsMap_[index]; 		 
	     }
     }		 

		
	return CulObs;
}

void DynamicVoronoi::AgentPosPostCheck(PartitionInfo* partition_info_single)
{
    int init_x = std::round(partition_info_single->part_agent_coor_x_);
	int init_y = std::round(partition_info_single->part_agent_coor_y_);
    
	std::queue<std::vector<int>> PosQueue;  
    
	unsigned char* VisitCheck;
	
	VisitCheck = new unsigned char[this->map_height_*this->map_width_];
	memset(VisitCheck,0,map_height_*map_width_*sizeof (unsigned char));
	AgentPosPropagation(PosQueue, VisitCheck, init_x, init_y);
	while (PosQueue.size()>0)
	{
		std::vector<int> pos_front = PosQueue.front(); 
		PosQueue.pop(); 
		 
        float density = this->GetDensity(pos_front[0], pos_front[1]);
	    if(density > 0) 
		{
			partition_info_single->part_agent_coor_x_ = pos_front[0];
			partition_info_single->part_agent_coor_y_ = pos_front[1];
			break;
		}
        else 	AgentPosPropagation(PosQueue, VisitCheck, pos_front[0], pos_front[1]);		
	}

	delete VisitCheck;
}

void DynamicVoronoi::AgentPosPropagation(std::queue<std::vector<int>>& PosQueue, unsigned char* VisitCheck, int init_x, int init_y)
{	
	int x_move[4] ={1, 0, -1, 0};
	int y_move[4] ={0, 1, 0, -1};
	std::vector<int> pos_xy;
	
	for (int k =0; k<4; k++)
	{
		int x = init_x + x_move[k];
		int y = init_y + y_move[k];
		
		if((x == this->map_width_)||(y == this->map_height_)||(x < 0)||(y < 0))
		{
			continue;
		}
		
		unsigned char  visited = VisitCheck[x+y*this->map_width_];
		if (visited == 1) continue;
		VisitCheck[x+y*this->map_width_] = 1;
		pos_xy.push_back(x);
		pos_xy.push_back(y);
		PosQueue.push(pos_xy);
        pos_xy.clear();	
	}
}

bool DynamicVoronoi::FindNearPtNonObs(PartitionInfo* partition_info_single, float init_agent_coor_x_local, float init_agent_coor_y_local, float increment_x, float increment_y, float* ref_dist_sq)
{
	bool end_while_loop = false;
	bool is_success_ = true;
	float new_agent_coor_x = init_agent_coor_x_local;
	float new_agent_coor_y = init_agent_coor_y_local;
	
	while(!end_while_loop)
	{
		new_agent_coor_x = new_agent_coor_x + increment_x;
		new_agent_coor_y = new_agent_coor_y + increment_y;
		
		if((new_agent_coor_x>=0)&&(new_agent_coor_x<=this->map_width_-1)&&(new_agent_coor_y>=0)&&(new_agent_coor_y<=this->map_height_-1))
		{
			/*intentially empty*/
		}
		else
		{
			 is_success_ = false;
             end_while_loop = true;
			 break;
			
		}
		
		
		float density = this->GetDensity((int)std::round(new_agent_coor_x), (int)std::round(new_agent_coor_y));
		
		if(density >0)
		{
			float dist_sq_temp = (new_agent_coor_x - init_agent_coor_x_local)*(new_agent_coor_x - init_agent_coor_x_local)+ (new_agent_coor_y - init_agent_coor_y_local)*(new_agent_coor_y - init_agent_coor_y_local);
			
			if(*ref_dist_sq > dist_sq_temp)
			{
				*ref_dist_sq = dist_sq_temp;
				partition_info_single->part_agent_coor_x_ = (int)std::round(new_agent_coor_x);
				partition_info_single->part_agent_coor_y_ = (int)std::round(new_agent_coor_y);
			}
			else
			{
				/*intentionally empty*/
			}
			
			end_while_loop = true;
			break;
		}
	}
	

	return is_success_;
	
}


bool DynamicVoronoi::FindNearPtNonObs_R2(PartitionInfo* partition_info_single, float init_agent_coor_x_local, float init_agent_coor_y_local, float* ref_dist_sq)
{
	bool is_success_ = true;	

    float gradient;
	float x_increment;
	
	if(!this->AgentCoorOpenSp_.empty())
	{

		std::vector<std::vector<float>>::iterator it;
			
		for(it=this->AgentCoorOpenSp_.begin(); it!=this->AgentCoorOpenSp_.end(); it++)
		{
			float agent_coor_x_notobs = it->at(0);
			float agent_coor_y_notobs = it->at(1);
			
			bool end_while_loop = false;
			float new_agent_coor_x = init_agent_coor_x_local;
	        float new_agent_coor_y = init_agent_coor_y_local;
			
			gradient = (agent_coor_y_notobs - init_agent_coor_y_local)/(agent_coor_x_notobs - init_agent_coor_x_local);
			
			if((agent_coor_x_notobs - init_agent_coor_x_local) >= 0)
			{
				x_increment = 1;
			}
			else
			{
				x_increment = -1;
			}
			
			//std::cout<<" debug  *** "<<"init_agent_coor_x_local : "<<init_agent_coor_x_local<<"init_agent_coor_y_local : "<<init_agent_coor_y_local<<"gradient : "<< gradient <<" agent_coor_x_notobs : "<< agent_coor_x_notobs << " agent_coor_y_notobs : "<< agent_coor_y_notobs <<std::endl;
			
			while(!end_while_loop)
			{
			    new_agent_coor_x = new_agent_coor_x + x_increment;
		        new_agent_coor_y = new_agent_coor_y + gradient;
				
				
				if((new_agent_coor_x>=0)&&(new_agent_coor_x<=this->map_width_-1)&&(new_agent_coor_y>=0)&&(new_agent_coor_y<=this->map_height_-1))
		        {
			         /*intentially empty*/
		        }
		        else
		        {
					std::cout<<" debug  *** "<<"break  "<< std::endl;
			         is_success_ = false;
                     end_while_loop = true;
			         break;
		        }
				
				
				float density = this->GetDensity((int)std::round(new_agent_coor_x), (int)std::round(new_agent_coor_y));
						
		        if(density >0)
		        {
			        float dist_sq_temp = (new_agent_coor_x - init_agent_coor_x_local)*(new_agent_coor_x - init_agent_coor_x_local)+ (new_agent_coor_y - init_agent_coor_y_local)*(new_agent_coor_y - init_agent_coor_y_local);
			        
	
					
			        if(*ref_dist_sq > dist_sq_temp)
			        {
				       *ref_dist_sq = dist_sq_temp;
				       partition_info_single->part_agent_coor_x_ = (int)std::round(new_agent_coor_x);
				       partition_info_single->part_agent_coor_y_ = (int)std::round(new_agent_coor_y);
			        }
			        else
			        {
				        /*intentionally empty*/
			        }
			
			        end_while_loop = true;
			        break;
		        }	
			}
		}	
	}
	else
	{
		is_success_ = false;
	}

	return is_success_;		
	
}

bool DynamicVoronoi::AgentDropOut()
{
	PartitionInfo* partition_info_single;
	float DropOutTolerance = 100000.0;
	
	if(this->is_hete_cov_radius_ == false)  DropOutTolerance = this->CovrdMass_*this->DropOutWeight_/(float)this->VoroPartNum_;
	else DropOutTolerance = this->CvdAreaDivMaxR_*this->DropOutWeight_/(float)this->VoroPartNum_;
	
	std::cout<<" ************DynamicVoronoi::AgentDropOut precheck ***************" <<std::endl;

	bool active = false;
	
	for(int i = 0; i < this->VoroPartNum_; i++)
	{
		PartitionInfo* partition_info_single = this->partition_info_[i];
		 /* Initialize agentDropoutCheck_ array */ 
		this->agentDropoutCheck_[i] = false;
	}
    

     for(int i = 0; i < this->VoroPartNum_; i++)
	{
		float DropOutVar;
	    PartitionInfo* partition_info_single = this->partition_info_[i];
		
		if(this->is_hete_cov_radius_ == false) DropOutVar = partition_info_single->part_mass_;
		else 
		{
			float area = partition_info_single->part_area_; 
			float radius = partition_info_single->part_cov_radius_; 
			DropOutVar = area/(PI*radius*radius);
		}
		
		if(DropOutVar > DropOutTolerance)
		{
		     active = true; 
             this->agentDropoutCheck_[i] = true;
			 std::cout<<" On dropout " <<"agent_num : "<< (i+1) <<" DropOutVar : "  << DropOutVar<< " DropOutTolerance" << DropOutTolerance <<std::endl;
	    }
		else std::cout<<" No dropout " <<"agent_num : "<< (i+1) <<" DropOutVar : "  << DropOutVar<< " DropOutTolerance" << DropOutTolerance <<std::endl;
	}

	return active;
}
