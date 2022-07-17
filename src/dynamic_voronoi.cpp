#include "dynamic_voronoi/dynamic_voronoi.h"


std::vector<std::string> split(std::string str, char Delimiter) {
    std::istringstream iss(str);             
    std::string buffer;                      
 
    std::vector<std::string> result;
 
 
    while (std::getline(iss, buffer, Delimiter)) {
        result.push_back(buffer);             
    }
 
    return result;
}


int propagation_frame_interval = 100;
std::ofstream img_list;

bool is_optimize_animation = false;

/*constructor and destructor*/
DynamicVoronoi::DynamicVoronoi(unsigned short map_height, unsigned short map_width, bool is_uniform_density, bool is_point_optimization, unsigned char* vorocellDenseMapExtPtr):
map_height_(map_height),map_width_(map_width),
is_uniform_density_(is_uniform_density), is_point_optimization_(is_point_optimization),vorocellDenseMapExtPtr_(vorocellDenseMapExtPtr)
{
   /*c++ style */	
   this->vorocell_ = new VoroCell*[map_height*map_width];
   
   for (int i = 0; i < map_height*map_width; ++i) {
    this->vorocell_[i] = new VoroCell();
   }
   
   this->vorocellDenseMap_ = new unsigned char[map_height*map_width];
   
   /*c style */
   //this->vorocell_ = (VoroCell*)malloc(sizeof(VoroCell)*((size_t)map_height)*((size_t)map_width));
   //this->vorocellDenseMap_ = (unsigned char*)malloc(sizeof(unsigned char)*((size_t)map_height)*((size_t)map_width));
   
   this->is_propa_completed_ = false;
   this->VoroPartNum_ = 0;
   this->TotalArea_ = 0;
   
   this->InitializeCell();
   this->InitializeDensityMap();
}

DynamicVoronoi::~DynamicVoronoi()
{
	
	 /*c++ style */	
	 
    for (int i = 0; i < this->map_height_*this->map_width_; ++i) {
    delete this->vorocell_[i];
    }
	
    for (int i = 0; i < this->VoroPartNum_; ++i) {
    delete this->partition_info_[i];
    }	
	 
	 
	delete[] this->vorocell_;
	delete[] this->vorocellDenseMap_;
	delete[] this->partition_info_;
	/*c style */
	//free(this->vorocell_);
	//free(this->vorocellDenseMap_);
}

int DynamicVoronoi::GetIndex(int x, int y)
{
	if((x>=this->map_width_)||(y>=this->map_height_))
	{
		std::cout<<"x or y index is out of bound in DynamicVoronoi"<<std::endl;
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

bool DynamicVoronoi::PushPoint(float x, float y)
{
	
	if(x >= this->map_width_)
	{
		return false;
	}
	
	if(y >= this->map_height_)
	{
		return false;
	}
	
	
	if(this->VoroPartNum_ == 0)
	{
		this->partition_info_ = new PartitionInfo*[this->VoroPartNum_+1];
		this->partition_info_[0] = new PartitionInfo();
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
		
	}
	
	PartitionInfo* partition_info_single = this->partition_info_[this->VoroPartNum_];
    partition_info_single->part_agentclass_ = this->VoroPartNum_;
	partition_info_single->centroid_momentsum_x_ = 0;
	partition_info_single->centroid_momentsum_y_ = 0;
	partition_info_single->part_area = 0;
	partition_info_single->part_centroid_x_ = 0;
	partition_info_single->part_centroid_y_ = 0;
	partition_info_single->part_agent_coor_x_ = x;
	partition_info_single->part_agent_coor_y_ = y;
	partition_info_single->agent_parllel_momentsum_x_ = 0;
	partition_info_single->agent_parllel_momentsum_y_ = 0;

	
	//std::vector<float>  xy_coor;
    
    //xy_coor.push_back(x);
    //xy_coor.push_back(y);	
	
	//this->VoroPartCenters_.insert(std::make_pair(this->VoroPartNum_, xy_coor));

	this->is_propa_completed_ = false;
	
	this->VoroPartNum_ = this->VoroPartNum_ + 1;
	
	//std::cout<<" this->VoroPartNum_:  "<< this->VoroPartNum_<<std::endl;
	//for(int j = 0; j<this->VoroPartNum_;j++)
	//{
	//	PartitionInfo* partition_info_single = this->partition_info_[j];	
	//	std::cout<<j<<" th : "<<" part_agentclass_: "<< partition_info_single->part_agentclass_ << " part_agent_coor_x_: "<<partition_info_single->part_agent_coor_x_<< " part_agent_coor_y_: "<<partition_info_single->part_agent_coor_y_ <<std::endl;	
	//}
	
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
			 else if(density == 0)  // if the point is obstacle, set wight color
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
		 }
		 
	 }


	for(int i =0; i <this->VoroPartNum_;i++)
	{
		PartitionInfo* partition_info_single = this->partition_info_[i];
		
		unsigned short centroid_x = (unsigned short)std::round(partition_info_single->part_centroid_x_);
		unsigned short centroid_y = (unsigned short)std::round(partition_info_single->part_centroid_y_);
		
		labelImg.at<cv::Vec3b>(centroid_y,centroid_x)[0] = 0;
		labelImg.at<cv::Vec3b>(centroid_y,centroid_x)[1] = 0;
		labelImg.at<cv::Vec3b>(centroid_y,centroid_x)[2] = 255;
		
		unsigned short agent_cen_x = (unsigned short)std::round(partition_info_single->part_agent_coor_x_);
		unsigned short agent_cen_y = (unsigned short)std::round(partition_info_single->part_agent_coor_y_);
		
		labelImg.at<cv::Vec3b>(agent_cen_y,agent_cen_x)[0] = 0;
		labelImg.at<cv::Vec3b>(agent_cen_y,agent_cen_x)[1] = 0;
		labelImg.at<cv::Vec3b>(agent_cen_y,agent_cen_x)[2] = 0;

	}


	 cv::imwrite(img_save_dir, labelImg);
	 
	 
	 std::string gray = "_edge";
	 std::string extention = ".png";
	 std::vector<std::string> image_save_dir_split = split(img_save_dir,'.');
	 std::string img_save_edge_dir = image_save_dir_split[0] + gray + extention;
	 
	 std::cout << img_save_edge_dir << std::endl;
	 
	 cv::Mat labelImg_gray, labelImg_edge;
	 cv::cvtColor( labelImg, labelImg_gray, CV_RGB2GRAY );
	 cv::Canny(labelImg_gray, labelImg_edge, 1, 10, 3);
	 
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
			
			    this->TotalArea_ = this->TotalArea_ + density;
		   }	
	   }
	}
	else
	{
		memcpy(this->vorocellDenseMap_, this->vorocellDenseMapExtPtr_, sizeof(unsigned char)*(this->map_height_)*(this->map_width_));
		//directly memcopy;
		
		for(int row=0; row<map_height_; row++)  
	     {
         for(int col=0; col<map_width_; col++)
	     {
		   int index =col + row*map_width_;		   
		   //std::cout<<" row: "<< row << " col: "<< col << " index: "<< index << " vorocellDenseMap_[index] : "<< (float)vorocellDenseMap_[index]  <<std::endl;
	     }
	     }
		
		for(x= 0;x<this->map_width_;x++)
		{
			for(y=0;y<this->map_height_;y++)
			{
				density = this->GetDensity(x,y);
				//std::cout<<" row: "<< y << " col: "<< x << " density: "<< (float)density <<std::endl;
				this->TotalArea_ = this->TotalArea_ + density;
			}
		}
	}
	
	std::cout<<" ************Initialize density map success ***************" <<std::endl;
}


void DynamicVoronoi::UpdateDensityMap(unsigned char* vorocellDenseMapExtPtr)
{
	memcpy(this->vorocellDenseMap_, vorocellDenseMapExtPtr, sizeof(unsigned char)*(this->map_height_)*(this->map_width_));
    //memcopy
}

unsigned char DynamicVoronoi::GetDensity(int x, int y)
{
	int index = this->GetIndex(x, y);
	
	return this->vorocellDenseMap_[index];
}

void DynamicVoronoi::MainOptProcess(bool is_optimize_animation, std::string img_dir, std::string label_dir, int max_step_size, float terminate_criteria)
{

    std::cout<<" DynamicVoronoi::MainOptProcess start " << std::endl;

	if(is_optimize_animation == true)
	{
	   std::string list_dir = img_dir + "img_file_list.txt";
	   img_list.open(list_dir);   
	}
	
	for(int i = 0; i < max_step_size; i++)
	{
	   this->ExpandedVoronoi();
	   this->CentroidCal();
	   
	   
	   if((is_optimize_animation == true)&&(i==0))
	   {
		   std::string img_save_dir = img_dir + "map" +".png";
		   this->Colorized(img_save_dir,label_dir);
		   img_list<<"map.png"<<std::endl;
	   }
	   
	   float error = this->MoveAgents();
	   
	   if(is_optimize_animation == true)
	   {
		   	std::string img_save_dir = img_dir + "map" + std::to_string(i) +".png";
			this->Colorized(img_save_dir,label_dir);
			img_list<<"map" + std::to_string(i) +".png"<<std::endl;
	   }
	   
	   this->InitializeCell();
	   
	   if (error <= terminate_criteria) 
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

    std::cout<<" DynamicVoronoi::MainOptProcess end " << std::endl;

}

bool DynamicVoronoi::ExpandedVoronoi(bool is_propagation_animation, std::string img_dir, std::string label_dir)
{
	int agent_index;
	VoroCell* vorocell_temp;
	
	std::cout<<" ******************** DynamicVoronoi::ExpandedVoronoi start ******************* " << std::endl;
	
	for(int i =0; i <this->VoroPartNum_;i++)
	{
		PartitionInfo* partition_info_single = this->partition_info_[i];
		agent_index = partition_info_single->part_agentclass_;
	
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
        this->Propagatation(agent_index, agent_cen_x, agent_cen_y, col, row);		
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
			
		int col = vorocell_temp->col_index_; // equavalent to x
		int row = vorocell_temp->row_index_; // equavalent to y
		
		// Propagate the cells from the initial cell for each agent
        this->Propagatation(agent_index, agent_cen_x, agent_cen_y, col, row);
		
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

void DynamicVoronoi::Propagatation(int agent_index, unsigned short agent_cen_x, unsigned short agent_cen_y, int col, int row)
{
	VoroCell* vorocell_temp;
	std::vector<int> queue_xy;
     
   
	 // check if the column (or x ) is out of left bound
     if(col>0)
	 {
		vorocell_temp = GetSingleCellByIndex(col-1, row);
			
		int sq_dist_new = (agent_cen_x - (col - 1))*(agent_cen_x - (col - 1)) + (agent_cen_y - row)*(agent_cen_y - row);
			
		if(vorocell_temp->state_ == init)
		{
			vorocell_temp->agentclass_ = agent_index;
		    vorocell_temp->agent_cen_x_ = agent_cen_x;
		    vorocell_temp->agent_cen_y_ = agent_cen_y;	
			vorocell_temp->sq_dist_ = sq_dist_new;
			vorocell_temp->state_ = queued;
			queue_xy.push_back(col-1);
			queue_xy.push_back(row);
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
			   queue_xy.push_back(col-1);
			   queue_xy.push_back(row);
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
		
	// check if the column (or x ) is out of right bound
	if(col<(this->map_width_ -1))
	{
		vorocell_temp = GetSingleCellByIndex(col+1, row);
			
		int sq_dist_new = (agent_cen_x - (col + 1))*(agent_cen_x - (col + 1)) + (agent_cen_y - row)*(agent_cen_y - row);

		if(vorocell_temp->state_ == init)
		{
			vorocell_temp->agentclass_ = agent_index;
		    vorocell_temp->agent_cen_x_ = agent_cen_x;
		    vorocell_temp->agent_cen_y_ = agent_cen_y;	
			vorocell_temp->sq_dist_ = sq_dist_new;
			vorocell_temp->state_ = queued;
			queue_xy.push_back(col+1);
			queue_xy.push_back(row);
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
			   queue_xy.push_back(col+1);
			   queue_xy.push_back(row);
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
				//std::cout<<"line 612"<<" col :"<<col+1<<" row :"<<row<<std::endl;
			}
		}
	}
		
	// check if the row (or y ) is out of upper bound
	if(row>0)
	{
		vorocell_temp = GetSingleCellByIndex(col, row-1);

		int sq_dist_new = (agent_cen_x - col)*(agent_cen_x - col) + (agent_cen_y - (row - 1))*(agent_cen_y - (row - 1));

		if(vorocell_temp->state_ == init)
		{
			vorocell_temp->agentclass_ = agent_index;
		    vorocell_temp->agent_cen_x_ = agent_cen_x;
	        vorocell_temp->agent_cen_y_ = agent_cen_y;	
			vorocell_temp->sq_dist_ = sq_dist_new;
			vorocell_temp->state_ = queued;
			queue_xy.push_back(col);
			queue_xy.push_back(row-1);
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
			   queue_xy.push_back(col);
			   queue_xy.push_back(row-1);
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
			}
		}				
	}
		
	// check if the row (or y ) is out of lower bound
			
	if(row<(this->map_height_-1))
	{
		vorocell_temp = GetSingleCellByIndex(col, row+1);
			
		int sq_dist_new = (agent_cen_x - col)*(agent_cen_x - col) + (agent_cen_y - (row + 1))*(agent_cen_y - (row + 1));
			
		if(vorocell_temp->state_ == init)
		{
			vorocell_temp->agentclass_ = agent_index;
		    vorocell_temp->agent_cen_x_ = agent_cen_x;
		    vorocell_temp->agent_cen_y_ = agent_cen_y;	
			vorocell_temp->sq_dist_ = sq_dist_new;
			vorocell_temp->state_ = queued;
			queue_xy.push_back(col);
			queue_xy.push_back(row+1);
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
			   queue_xy.push_back(col);
			   queue_xy.push_back(row+1);
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
		partition_info_single->part_area = 0;
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
	}

    /* calcuate variables for centroid  */
	for(int col= 0;col<this->map_width_;col++)
	{
		for(int row=0;row<this->map_height_;row++)
		{
			vorocell_temp = GetSingleCellByIndex(col, row);
			unsigned short agentclass = vorocell_temp->agentclass_;
			
			if(agentclass == 0xFFFF) continue;
			
			PartitionInfo* partition_info_single = this->partition_info_[agentclass];
			partition_info_single->part_area = partition_info_single->part_area + 1*GetDensity(col, row); 
			partition_info_single->centroid_momentsum_x_ = partition_info_single->centroid_momentsum_x_ + col*1*GetDensity(col, row);
			partition_info_single->centroid_momentsum_y_ = partition_info_single->centroid_momentsum_y_ + row*1*GetDensity(col, row);
			
			/*calculate points close to each corners */ /* using Manhattan distance to reduce the computation cost*/
			int LeftFrontPtDisTemp = (col - 0) + (row - 0);			
			if(LeftFrontPtDisTemp <= partition_info_single->LeftFrontPtDis)  
			{
				partition_info_single->LeftFrontPtDis = LeftFrontPtDisTemp;
				partition_info_single->LeftFrontPt[0] = col;
				partition_info_single->LeftFrontPt[1] = row;
			}
			
			int RightFrontPtDisTemp = (this->map_width_ - col) + (row - 0);			
			if(RightFrontPtDisTemp <= partition_info_single->RightFrontPtDis)  
			{
				partition_info_single->RightFrontPtDis = RightFrontPtDisTemp;
				partition_info_single->RightFrontPt[0] = col;
				partition_info_single->RightFrontPt[1] = row;
			}				

			int LeftRearPtDisTemp = (col - 0) + (this->map_height_ - row);			
			if(LeftRearPtDisTemp <= partition_info_single->LeftRearPtDis)  
			{
				partition_info_single->LeftRearPt[0] = col;
				partition_info_single->LeftRearPt[1] = row;
				partition_info_single->LeftRearPtDis = LeftRearPtDisTemp;				
			}

			int RightRearPtDisTemp = (this->map_width_ - col) + (this->map_height_ - row);			
			if(RightRearPtDisTemp <= partition_info_single->RightRearPtDis)  
			{
				partition_info_single->RightRearPt[0] = col;
				partition_info_single->RightRearPt[1] = row;
				partition_info_single->RightRearPtDis = RightRearPtDisTemp;				
			}
			
		}
		
	}
	
	/* calculate centroid */
	for(int i = 0; i < this->VoroPartNum_; i++)
	{
		partition_info_single = this->partition_info_[i];
		
		if(partition_info_single->part_area > 0)
		{
		   partition_info_single->part_centroid_x_ = partition_info_single->centroid_momentsum_x_/partition_info_single->part_area;
		   partition_info_single->part_centroid_y_ = partition_info_single->centroid_momentsum_y_/partition_info_single->part_area;			
		}
		else
		{
		   partition_info_single->part_centroid_x_ = partition_info_single->part_agent_coor_x_;
		   partition_info_single->part_centroid_y_ = partition_info_single->part_agent_coor_y_;	
		}			
			
		std::cout<< "class num : "<<i<<" centroid x : "<<partition_info_single->part_centroid_x_<<" centroid y : "<<partition_info_single->part_centroid_y_<<std::endl;
	}

	//std::cout<<" ******************** DynamicVoronoi::CentroidCal end ******************* " << std::endl;

}

float DynamicVoronoi::MoveAgents()
{
	PartitionInfo* partition_info_single;
	float error = 0;
	int cnt_project_req = 0;
	std::cout<<" ************DynamicVoronoi::MoveAgents start ***************" <<std::endl;
	
	for(int i = 0; i < this->VoroPartNum_; i++)
	{
		bool is_Partition_inObs = false;
		partition_info_single = this->partition_info_[i];
		
		if( partition_info_single->part_area > 0)
		{
			is_Partition_inObs = false;
		}
		else
		{
			is_Partition_inObs = true;
			cnt_project_req++;
		}
		
		std::cout << "agent index :" << i << " partition_info_single->part_area : "<< partition_info_single->part_area <<" is_Partition_inObs : "<< is_Partition_inObs << std::endl;
		
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
				partition_info_single->part_agent_coor_x_ = 0;
				partition_info_single->part_agent_coor_y_ = 0;
				
				error = 100;
				
				std::cout<<" Projection failed, check the density map"<<std::endl;
			}
			else
			{
			   /* after running the FindNearPtNonObs function, the agent position may be relocated */
			   float new_agent_coor_x = partition_info_single->part_agent_coor_x_;
		       float new_agent_coor_y = partition_info_single->part_agent_coor_y_;
		
		       error = error + (new_agent_coor_x - init_agent_coor_x)*(new_agent_coor_x - init_agent_coor_x) + (new_agent_coor_y - init_agent_coor_y)*(new_agent_coor_y - init_agent_coor_y);						
			}
			

			
		}
		else
		{
			float area = partition_info_single->part_area;
			float agent_coor_x = partition_info_single->part_agent_coor_x_;
			float agent_coor_y = partition_info_single->part_agent_coor_y_;
			float centroid_x = partition_info_single->part_centroid_x_;
			float centroid_y = partition_info_single->part_centroid_y_;
			/*agent coordinate update using gradient descent */ /*instead using the partition area directly, normalized area was used (area/Total area) or inverse normalized area ((Total area - area)/Total area)*/
			
			float coefficient = (this->TotalArea_- area)/this->TotalArea_;
			float min_coeff = 0.5;
			
			if(coefficient < min_coeff)
			{
				coefficient = min_coeff;
			}
			
		    agent_coor_x = agent_coor_x - 2*coefficient*(agent_coor_x - centroid_x);     
		    agent_coor_y = agent_coor_y - 2*coefficient*(agent_coor_y - centroid_y);
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
			float density = this->GetDensity((int)std::round(agent_coor_x), (int)std::round(agent_coor_y));
			if(density>0)
			{
				std::vector<float> AgentCoor_temp;
				AgentCoor_temp.push_back(partition_info_single->part_agent_coor_x_);
				AgentCoor_temp.push_back(partition_info_single->part_agent_coor_y_ );
				
			    this->AgentCoorOpenSp_.push_back(AgentCoor_temp);	
			}
		    else
			{

                this->AgentPosPostCheck(partition_info_single);				
				
				std::vector<float> AgentCoor_temp;
				AgentCoor_temp.push_back(partition_info_single->part_agent_coor_x_);
				AgentCoor_temp.push_back(partition_info_single->part_agent_coor_y_ );
				
			    this->AgentCoorOpenSp_.push_back(AgentCoor_temp);	
			}
		
		    /* calculate error */
		    error = error + (agent_coor_x - centroid_x)*(agent_coor_x - centroid_x) + (agent_coor_y - centroid_y)*(agent_coor_y - centroid_y);
		}
		

		std::cout<<" agent number : "<<i<<" new agent coordinate x : "<<partition_info_single->part_agent_coor_x_<<" new agent coordinate y : "<<partition_info_single->part_agent_coor_y_<<" new error : "<<error<<std::endl;
		
	}
	
	if(cnt_project_req==0)
	{
		this->AgentCoorOpenSp_.clear();
	}
	
	return error;
}


void DynamicVoronoi::AgentPosPostCheck(PartitionInfo* partition_info_single)
{
	bool is_proj_success  = false;
	float ref_dist_sq = this->map_width_*this->map_width_ + this->map_height_*this->map_height_; 

    float init_agent_coor_x = partition_info_single->part_agent_coor_x_;
	float init_agent_coor_y = partition_info_single->part_agent_coor_y_;

	float gradient_L1;
	float increment_x_temp;
	if(partition_info_single->LeftRearPt[0] == partition_info_single->LeftFrontPt[0])
	{
	    gradient_L1 = 1;
		increment_x_temp = 0;
	}
	else
	{
	    gradient_L1 = (partition_info_single->LeftRearPt[1] - partition_info_single->LeftFrontPt[1])/(partition_info_single->LeftRearPt[0] - partition_info_single->LeftFrontPt[0]);
		increment_x_temp = 1;
	}
				
	bool is_proj_success_temp = this->FindNearPtNonObs(partition_info_single, init_agent_coor_x, init_agent_coor_y, increment_x_temp, gradient_L1, &ref_dist_sq);  // go to right along the L1 line 
	is_proj_success = is_proj_success_temp | is_proj_success;
	is_proj_success_temp = this->FindNearPtNonObs(partition_info_single, init_agent_coor_x, init_agent_coor_y, -increment_x_temp, -gradient_L1, &ref_dist_sq);  // go to left along the L1 line
	is_proj_success = is_proj_success_temp | is_proj_success;
				
	float gradient_L2;
				
	if(partition_info_single->RightRearPt[0] == partition_info_single->RightFrontPt[0])
	{
	   gradient_L2 = 1;
	   increment_x_temp = 0;
	}
	else
	{
	    gradient_L2 = (partition_info_single->RightRearPt[1] - partition_info_single->RightFrontPt[1])/(partition_info_single->RightRearPt[0] - partition_info_single->RightFrontPt[0]);
	    increment_x_temp = 1;
	}
				
	is_proj_success_temp = this->FindNearPtNonObs(partition_info_single, partition_info_single->part_agent_coor_x_, partition_info_single->part_agent_coor_y_, increment_x_temp, gradient_L2, &ref_dist_sq);  // go to right along the L1 line 
	is_proj_success = is_proj_success_temp | is_proj_success;
	is_proj_success_temp = this->FindNearPtNonObs(partition_info_single, partition_info_single->part_agent_coor_x_, partition_info_single->part_agent_coor_y_, -increment_x_temp, -gradient_L2, &ref_dist_sq);  // go to left along the L1 line
	is_proj_success = is_proj_success_temp | is_proj_success;	
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
				partition_info_single->part_agent_coor_x_ = new_agent_coor_x;
				partition_info_single->part_agent_coor_y_ = new_agent_coor_y;
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
			
			
			while(!end_while_loop)
			{
			    new_agent_coor_x = new_agent_coor_x + 1;
		        new_agent_coor_y = new_agent_coor_y + gradient;
				
				
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
				       partition_info_single->part_agent_coor_x_ = new_agent_coor_x;
				       partition_info_single->part_agent_coor_y_ = new_agent_coor_y;
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