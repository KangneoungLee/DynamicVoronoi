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
	   unsigned int sq_dist_;
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
	   float part_agent_coor_x_;
	   float part_agent_coor_y_;
	   float agent_parllel_momentsum_x_;
	   float agent_parllel_momentsum_y_;
	
	/*constructor and destructor*/
	 PartitionInfo();
	 ~PartitionInfo();
	
};


#endif