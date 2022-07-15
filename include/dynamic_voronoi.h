#ifndef DYNAMICVORONOI_H_
#define DYNAMICVORONOI_H_

#include <chrono>
#include <vector>
#include <queue>
#include <map>
#include <utility>  //pair 
#include <cmath>        // std::abs

#include <iostream>
#include <fstream>
#include <string>

#include <stdio.h>      /* printf, scanf, NULL */
#include <stdlib.h>     /* malloc, free, rand */

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ml.hpp"

#include "dynamic_voronoi/voro_cell.h"



class DynamicVoronoi{
	
	private:
	   	VoroCell** vorocell_;
		PartitionInfo** partition_info_;
	    unsigned char* vorocellDenseMap_;
		
		unsigned char* vorocellDenseMapExtPtr_;
		
        unsigned short map_height_;
        unsigned short map_width_;		
		bool is_uniform_density_;
		bool is_point_optimization_;
		bool is_propa_completed_;
		
		int VoroPartNum_;
		float TotalArea_;
		
		std::vector<std::vector<float>> AgentCoorOpenSp_;
		
		std::map<int,std::vector<float>> VoroPartCenters_;
		
		
		std::queue<std::vector<int>> ProccessingQueue_;   //std::queue is for FIFO
		
		
		std:: ifstream label_text_;
		
		std::vector<std::vector<int>> color_map_label_;
	
	public: 
	/*constructor and destructor*/
	   DynamicVoronoi(unsigned short map_height, unsigned short map_width, bool is_uniform_density, bool is_point_optimization, unsigned char* vorocellDenseMapExtPtr = NULL);
	   ~DynamicVoronoi();
	   
	   int GetIndex(int x, int y);
	   int MapPointToIndex(float x, float y);
	   VoroCell* GetSingleCellByIndex(int x, int y);
	   VoroCell* GetSingleCellByPoint(float x, float y);
	   bool PushPoint(float x, float y);
	   void InitializeCell();
	   void InitializeDensityMap();
	   void UpdateDensityMap(unsigned char* vorocellDenseMapExtPtr);
	   unsigned char GetDensity(int x, int y);
	   bool saveDensityMap(std::string file_name);
	   bool saveAgentMap(std::string file_name);
	   bool saveSingleVoro(std::string file_name, int x, int y);
	   bool Colorized(std::string img_save_dir, std::string label_txt_dir);
	   
	   bool ExpandedVoronoi(bool is_propagation_animation = false, std::string img_dir ="/home/dummy", std::string label_dir = "/home/dummy.txt");
	   void Propagatation(int agent_index, unsigned short agent_cen_x, unsigned short agent_cen_y, int col, int row);
	   float MoveAgents();
	   bool FindNearPtNonObs(PartitionInfo* partition_info_single, float init_agent_coor_x_local, float init_agent_coor_y_local, float increment_x, float increment_y, float* ref_dist_sq);
	   bool FindNearPtNonObs_R2(PartitionInfo* partition_info_single, float init_agent_coor_x_local, float init_agent_coor_y_local, float* ref_dist_sq);
	   bool AgentPosPostCheck();
	   void MainOptProcess(bool is_optimize_animation = false, std::string img_dir ="/home/dummy", std::string label_dir = "/home/dummy.txt", int max_step_size = 10, float terminate_criteria = 0.1);
	   void CentroidCal();
	 
//	VoroCell* DynamicVoronoi::operator[](int index)
//   {
//    if (index >= (map_height_*map_width_ -1)) {
//        cout << "index out of bound, exiting";
 //       exit(0);
 //      }
 //      return   this->vorocell_ + index;
 //   }

};



#endif