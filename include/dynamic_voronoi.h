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

#ifndef DYNAMICVORONOI_H_
#define DYNAMICVORONOI_H_

#include <chrono>
#include <vector>
#include <queue>
#include <map>
#include <utility>  //pair 
#include <cmath>        // std::abs
#include <chrono>
#include <cstdlib>

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

#define PI 3.14159265

class DynamicVoronoi{
	
	private:
	   	VoroCell** vorocell_ = NULL;
		PartitionInfo** partition_info_ = NULL;
	    unsigned char* vorocellDenseMap_ = NULL;
		
		unsigned char* vorocellDenseMapExtPtr_ = NULL;
		bool* agentDropoutCheck_;
		
		unsigned char* vorocellObsMap_ = NULL;
		
        unsigned short map_height_;
        unsigned short map_width_;		
		bool is_uniform_density_;
		bool is_dropout_use_;
		bool is_hete_cov_radius_;
		bool is_workeff_constraint_;
		bool is_point_optimization_;
		bool is_propa_connected_area_;
		bool is_propa_completed_;
		bool is_img_save_;

		bool dropout_active_ = false;
		bool inhibit_dropout_ = false;
		int inhibit_dropout_cnt = 0;
		
		int VoroPartNum_;
		float TotalMass_;
		float TotalArea_;
		float CovrdMass_;
		float CovrdArea_;
		float CvdAreaDivMaxR_;
		float DropOutWeight_;
		float weight_w_;
		float weight_h_;
		float lamda_;
		float x_datum_;
		float y_datum_;
		
		float cc_metric_ = 100;
		float cc_metric_prev_= 100;
		
		float cc_metric_diff_prev_ = 0;
		float cc_metric_diff_final_ = 0;
		float cc_metric_diff_rate_ = 0.3;
		
		std::vector<std::vector<float>> AgentCoorOpenSp_;
		
		std::map<int,std::vector<float>> VoroPartCenters_;
		
		
		std::queue<std::vector<int>> ProccessingQueue_;   //std::queue is for FIFO
		
		
		std:: ifstream label_text_;
		
		std::vector<std::vector<int>> color_map_label_;
	
	public: 
	/*constructor and destructor*/
	   DynamicVoronoi(unsigned short map_height, unsigned short map_width, float DropOutToleranceRate, float weight_w, float weight_h, float lamda, bool is_uniform_density, 
	                                           bool is_dropout_use, bool is_workeff_constraint, bool is_hete_cov_radius, bool is_propa_connected_area, bool is_img_save, unsigned char* vorocellDenseMapExtPtr = NULL);
	   ~DynamicVoronoi();
	   
	   VoroCell** GetVoroCellMap();
	   int GetIndex(int x, int y);
	   int MapPointToIndex(float x, float y);
	   VoroCell* GetSingleCellByIndex(int x, int y);
	   VoroCell* GetSingleCellByPoint(float x, float y);
	   bool PushPoint(float x, float y, float v_travel=0, float v_work=0, float cov_radius = 1);
	   bool AgentPoseUpdate(float x, float y, int agent_num);
	   bool AgentPoseGet(float& x, float& y, int agent_num);
	   bool PushDatum(float x_datum, float y_datum);
	   void ResizeMap(int map_height_new, int map_width_new, unsigned char* vorocellDenseMapExtPtr_new);
	   void InitializeCell();
	   void InitializeDensityMap();
	   void UpdateDensityMap(unsigned char* vorocellDenseMapExtPtr);
	   float GetDensity(int x, int y);
	   bool saveDensityMap(std::string file_name);
	   bool saveAgentMap(std::string file_name);
	   bool saveSingleVoro(std::string file_name, int x, int y);
	   bool Colorized(std::string img_save_dir, std::string label_txt_dir);
	   bool ExpandedVoronoi(bool is_propagation_animation = false, std::string img_dir ="/home/dummy", std::string label_dir = "/home/dummy.txt");
	   void Propagatation(int agent_index, unsigned short agent_cen_x, unsigned short agent_cen_y, int col, int row, float radius, int StartingPt_x, int StartingPt_y, float init_pos_x, float init_pos_y, float v_travel, float v_work, int area);
	   void MoveAgents();
	   float CoverageMetric();
	   void ResetDropout();
	   float CulObsCell(unsigned short agent_cen_x, unsigned short agent_cen_y, int col, int row);
	   bool FindNearPtNonObs(PartitionInfo* partition_info_single, float init_agent_coor_x_local, float init_agent_coor_y_local, float increment_x, float increment_y, float* ref_dist_sq);
	   bool FindNearPtNonObs_R2(PartitionInfo* partition_info_single, float init_agent_coor_x_local, float init_agent_coor_y_local, float* ref_dist_sq);
       bool ProjNearPoint(PartitionInfo* partition_info_single);
	   void AgentPosPropagation(std::queue<std::vector<int>>& PosQueue, unsigned char* VisitCheck, int init_x, int init_y);
	   bool FindNearPtNonOvL(PartitionInfo* partition_info_single);
	   void MainOfflineProcess(bool is_optimize_animation = false, std::string img_dir ="/home/dummy", std::string label_dir = "/home/dummy.txt", int max_step_size = 10, float terminate_criteria = 0.1);
	   void CentroidCal();
	   bool AgentDropOut();
	 
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
