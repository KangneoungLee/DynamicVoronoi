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

#ifndef VOROCELL_H_
#define VOROCELL_H_


enum CellStat{init = 0, queued = 1, completed = 2};

struct PivotP{
	
	float x;
	float y;
	unsigned char index;
	
};

class VoroCell{
	
	private:
	
	public: 
	   bool is_edge_;
	   float sq_dist_;
	   unsigned short agentclass_;
	   unsigned short col_index_;
	   unsigned short row_index_;
	   unsigned short agent_cen_x_;
	   unsigned short agent_cen_y_;
	   enum CellStat state_;
       
	
	/*constructor and destructor*/
	   VoroCell();
	   ~VoroCell();

};

class PartitionInfo{

	private:
	
	public:
	   float part_area;
	   unsigned short part_agentclass_;
	   float centroid_momentsum_x_;
	   float centroid_momentsum_y_;
	   float part_centroid_x_;
	   float part_centroid_y_; 
	   float part_agent_init_x_;
	   float part_agent_init_y_;
	   float part_agent_coor_x_;
	   float part_agent_coor_y_;
	   float part_v_travel_;
	   float part_v_work_;
	   float agent_parllel_momentsum_x_;
	   float agent_parllel_momentsum_y_;
	   int LeftFrontPt[2];
	   int LeftFrontPtDis;
	   int RightFrontPt[2];
	   int RightFrontPtDis;
	   int LeftRearPt[2];
	   int LeftRearPtDis;
	   int RightRearPt[2];
	   int RightRearPtDis;
	   int StartingPt[2];
	   int StartingPtDis;	
	   
	
	/*constructor and destructor*/
	 PartitionInfo();
	 ~PartitionInfo();
	
};


#endif
