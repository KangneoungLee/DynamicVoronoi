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
		bool* agentDropoutCheck_;
		
		unsigned char* vorocellObsMap_;
		
        unsigned short map_height_;
        unsigned short map_width_;		
		bool is_uniform_density_;
		bool is_workeff_constraint_;
		bool is_point_optimization_;
		bool is_propa_completed_;
		bool dropout_active_ = false;
		bool inhibit_dropout_ = false;
		int inhibit_dropout_cnt = 0;
		
		int VoroPartNum_;
		float TotalArea_;
		float DropOutToleranceRate_;
		float weight_w_;
		float weight_h_;
		float lamda_;
		float x_datum_;
		float y_datum_;
		
		std::vector<std::vector<float>> AgentCoorOpenSp_;
		
		std::map<int,std::vector<float>> VoroPartCenters_;
		
		
		std::queue<std::vector<int>> ProccessingQueue_;   //std::queue is for FIFO
		
		
		std:: ifstream label_text_;
		
		std::vector<std::vector<int>> color_map_label_;
	
	public: 
	/*constructor and destructor*/
	   DynamicVoronoi(unsigned short map_height, unsigned short map_width, float DropOutToleranceRate, float weight_w, float weight_h, float lamda, bool is_uniform_density, bool is_workeff_constraint, bool is_point_optimization, unsigned char* vorocellDenseMapExtPtr = NULL);
	   ~DynamicVoronoi();
	   
	   int GetIndex(int x, int y);
	   int MapPointToIndex(float x, float y);
	   VoroCell* GetSingleCellByIndex(int x, int y);
	   VoroCell* GetSingleCellByPoint(float x, float y);
	   bool PushPoint(float x, float y, float v_travel, float v_work);
	   bool PushDatum(float x_datum, float y_datum);
	   void InitializeCell();
	   void InitializeDensityMap();
	   void UpdateDensityMap(unsigned char* vorocellDenseMapExtPtr);
	   unsigned char GetDensity(int x, int y);
	   bool saveDensityMap(std::string file_name);
	   bool saveAgentMap(std::string file_name);
	   bool saveSingleVoro(std::string file_name, int x, int y);
	   bool Colorized(std::string img_save_dir, std::string label_txt_dir);
	   bool ExpandedVoronoi(bool is_propagation_animation = false, bool is_workeff_constraint = false, std::string img_dir ="/home/dummy", std::string label_dir = "/home/dummy.txt");
	   void Propagatation(int agent_index, unsigned short agent_cen_x, unsigned short agent_cen_y, int col, int row, int StartingPt_x, int StartingPt_y, float init_pos_x, float init_pos_y, float v_travel, float v_work, int area, bool is_workeff_constraint);
	   float MoveAgents();
	   int CulObsCell(unsigned short agent_cen_x, unsigned short agent_cen_y, int col, int row);
	   bool FindNearPtNonObs(PartitionInfo* partition_info_single, float init_agent_coor_x_local, float init_agent_coor_y_local, float increment_x, float increment_y, float* ref_dist_sq);
	   bool FindNearPtNonObs_R2(PartitionInfo* partition_info_single, float init_agent_coor_x_local, float init_agent_coor_y_local, float* ref_dist_sq);
       void AgentPosPostCheck(PartitionInfo* partition_info_single);
	   void MainOptProcess(bool is_optimize_animation = false, std::string img_dir ="/home/dummy", std::string label_dir = "/home/dummy.txt", int max_step_size = 10, float terminate_criteria = 0.1);
	   void CentroidCal();
	   void AgentDropOut();
	 
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