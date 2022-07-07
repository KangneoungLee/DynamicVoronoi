#include "dynamic_voronoi/dynamic_voronoi.h"

bool is_propagation_animation = true;
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
   
   this->is_completed_ = false;
   this->VoroPartNum_ = 0;
   
   this->InitializeCell();
   this->InitializeDensityMap();
}

DynamicVoronoi::~DynamicVoronoi()
{
	
	 /*c++ style */	
	 
    for (int i = 0; i < map_height*map_width; ++i) {
    delete this->vorocell_[i];
    }
	 
	 
	delete[] this->vorocell_;
	delete[] this->vorocellDenseMap_;
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
	
	std::vector<float>  xy_coor;
    
    xy_coor.push_back(x);
    xy_coor.push_back(y);	
	
	this->VoroPartCenters_.insert(std::make_pair(this->VoroPartNum_, xy_coor));

	this->is_completed_ = false;
	
	this->VoroPartNum_ = this->VoroPartNum_ + 1;
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
		 }
		 
	 }
	 
	 std::vector<float>  xy_point;
	
	std::map<int,std::vector<float>>::iterator it;
	for(it=VoroPartCenters_.begin(); it!=VoroPartCenters_.end(); ++it)
	{
		xy_point =  it->second;
		
		unsigned short agent_cen_x = (unsigned short)std::round(xy_point[0]);
		unsigned short agent_cen_y = (unsigned short)std::round(xy_point[1]);
		
		labelImg.at<cv::Vec3b>(agent_cen_y,agent_cen_x)[0] = 0;
		labelImg.at<cv::Vec3b>(agent_cen_y,agent_cen_x)[1] = 0;
		labelImg.at<cv::Vec3b>(agent_cen_y,agent_cen_x)[2] = 0;

	}
	
	
	 cv::imwrite(img_save_dir, labelImg);
	 
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
			
		   }	
	   }
	}
	else
	{
		memcpy(this->vorocellDenseMapExtPtr_, this->vorocellDenseMap_, sizeof(unsigned char)*((size_t)this->map_height_)*((size_t)this->map_width_));
		//directly memcopy;
	}
}


void DynamicVoronoi::UpdateDensityMap(unsigned char* vorocellDenseMapExtPtr)
{
	memcpy(vorocellDenseMapExtPtr, this->vorocellDenseMap_, sizeof(unsigned char)*((size_t)this->map_height_)*((size_t)this->map_width_));
    //memcopy
}


bool DynamicVoronoi::ExpandedVoronoi()
{
	int agent_index;
	std::vector<float>  xy_point;
	VoroCell* vorocell_temp;
	
	std::map<int,std::vector<float>>::iterator it;
	for(it=VoroPartCenters_.begin(); it!=VoroPartCenters_.end(); ++it)
	{
		agent_index = it->first;
		xy_point =  it->second;
		
		//std::cout<<"xy_point "<<xy_point[0]<<"  "<<xy_point[1]<<std::endl;
		
		unsigned short agent_cen_x = (unsigned short)std::round(xy_point[0]);
		unsigned short agent_cen_y = (unsigned short)std::round(xy_point[1]);
		
		vorocell_temp = GetSingleCellByIndex(agent_cen_x,agent_cen_y);
			
		vorocell_temp->sq_dist_ = 0;
		vorocell_temp->agentclass_ = agent_index;
		vorocell_temp->agent_cen_x_ = agent_cen_x;
		vorocell_temp->agent_cen_y_ = agent_cen_y;
		vorocell_temp->state_ = completed;
		
		//int sq_dist_prev = vorocell_temp->sq_dist_;
		
		int col = vorocell_temp->col_index_; // equavalent to x
		int row = vorocell_temp->row_index_; // equavalent to y
		
		std::vector<int> queue_xy;
		
		// Propagate the cells from the initial cell for each agent
		
		// check if the column (or x ) is out of left bound
		if(col>0)
		{
			vorocell_temp = GetSingleCellByIndex(col-1, row);
            
			int sq_dist_new = (agent_cen_x - (col - 1))*(agent_cen_x - (col - 1)) + (agent_cen_y - row)*(agent_cen_y - row);
			
			if(vorocell_temp->state_ == init)
			{
				vorocell_temp->agentclass_ = agent_index;
				vorocell_temp->sq_dist_ = sq_dist_new;
		        vorocell_temp->agent_cen_x_ = agent_cen_x;
		        vorocell_temp->agent_cen_y_ = agent_cen_y;
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
					vorocell_temp->is_edge_ = true;
					//std::cout<<"line 363"<<std::endl;
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
					vorocell_temp->is_edge_ = true;
					//std::cout<<"line 405"<<std::endl;
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
					vorocell_temp->is_edge_ = true;
					//std::cout<<"line 448"<<std::endl;
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
					vorocell_temp->is_edge_ = true;
					//std::cout<<"line 492"<<std::endl;
				}
			}	
		}			
		
	}

   // Propagate the cells from the queued cell 	
	std::vector<int>  xy_point_int;
	
	int i = 0;
	int f = 0;
    
	
	if(is_propagation_animation == true)
	{		
       std::string img_dir = "/home/kangneoung/sw_repo/dynamic_voronoi/src/dynamic_voronoi/test/animation/";
	   std::string list_dir = img_dir + "img_file_list.txt";
	   img_list.open(list_dir);
	   
	}

 
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
		//int sq_dist_prev = vorocell_temp->sq_dist_;
		
		int col = vorocell_temp->col_index_; // equavalent to x
		int row = vorocell_temp->row_index_; // equavalent to y
		
		//std::cout<<"agent_index "<<agent_index<<" sq_dist_prev : "<<sq_dist_prev<<" col :"<<col<<" row : "<<row<<std::endl;
		
		std::vector<int> queue_xy;
		
		// Propagate the cells from the initial cell for each agent
		
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
					//std::cout<<"line 654"<<" col :"<<col<<" row :"<<row-1<<std::endl;
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
					//std::cout<<"line 697"<<" col :"<<col<<" row :"<<row+1<<std::endl;
				}
			}	
			
		}
		
					
			
		if((is_propagation_animation==true)&&((i%propagation_frame_interval) == 0))
		{	
	
		    std::string img_dir = "/home/kangneoung/sw_repo/dynamic_voronoi/src/dynamic_voronoi/test/animation/";
	        std::string label_dir = "/home/kangneoung/sw_repo/dynamic_voronoi/src/dynamic_voronoi/label/label.txt";
			
			img_dir = img_dir + "map" + std::to_string(f) +".png";
			
			this->Colorized(img_dir,label_dir);
			
			img_list<<"map" + std::to_string(f) +".png"<<std::endl;
			
			f = f +1;
			
		}

        i = i +1;
	}

     this->is_completed_ = true;

	return true;

}

void DynamicVoronoi::Propagatation(int agent_index, unsigned short agent_cen_x, unsigned short agent_cen_y, int col, int row)
{
	VoroCell* vorocell_temp;
	std::vector<int> queue_xy;

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