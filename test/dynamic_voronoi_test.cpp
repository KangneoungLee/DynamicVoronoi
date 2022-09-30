
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <boost/bind.hpp>
#include <sys/stat.h> /*directory check*/
//#include <windows.h>  /*directory check*/
#include <ros/ros.h>

#include "dynamic_voronoi/dynamic_voronoi.h"

/************Function architecture*******************/


class DynamicVoronoiRosUnitTest{
	
	private:
	
		ros::NodeHandle main_nh_;
        ros::NodeHandle param_nh_;

	
	public:
		 
		/*constructor and destructor*/
	    DynamicVoronoiRosUnitTest(ros::NodeHandle m_nh, ros::NodeHandle p_nh);
	    ~DynamicVoronoiRosUnitTest();
		
		void testrun();
		void anima_play();
	    unsigned char* read_density(unsigned char*  density_array, std::string img_dir, int height, int width);

}; // class DynamicVoronoiRosUnitTest end

DynamicVoronoiRosUnitTest::DynamicVoronoiRosUnitTest(ros::NodeHandle m_nh, ros::NodeHandle p_nh):main_nh_(m_nh),param_nh_(p_nh)
{
}	
	
void DynamicVoronoiRosUnitTest::testrun()
{
	
	 int map_height = 100;
	 int map_width = 100;
	 int agent_num = 5;
	 float weight_w = 1;
	 float weight_h = 1;
	 float dataum_x = 1;
	 float dataum_y = 1;
	 float lamda = 0;
	 
	 std::vector<std::vector<int> > agent_attribute_vec;
	 XmlRpc::XmlRpcValue agent_attribute_list;
	 
	 
	 param_nh_.getParam("map_height",map_height);
	 param_nh_.getParam("map_width",map_width);
	 param_nh_.getParam("agent_num",agent_num);
	 param_nh_.getParam("agent_attribute_list",agent_attribute_list);	 
	 
	 param_nh_.getParam("weight_width",weight_w);
	 param_nh_.getParam("weight_height",weight_h);
	 param_nh_.getParam("dataum_x",dataum_x);
	 param_nh_.getParam("dataum_y",dataum_y);
	 param_nh_.getParam("lamda",lamda);
	 
	 ROS_ASSERT(agent_attribute_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	 
	 for (int i = 0; i < agent_num; i++)
	 {
		 std::vector<int> agent_attribute;
		 for (int j =0; j< 4; j++)
		 {
			 // These matrices can cause problems if all the types
			 // aren't specified with decimal points. Handle that
		    std::ostringstream ostr;
            ostr << agent_attribute_list[4 * i + j];    
			std::istringstream istr(ostr.str()); // istringstream parses the data corresponding to the data type
			
			int value_temp;
			istr >> value_temp;
			//std::cout<< " agent num index : " << i << "coordinate index : " << j << " Array index : " << agent_num * i + j << " value : " << value_temp << std::endl;
			//std::cout<< "agent_attribute_list " << agent_attribute_list.size() << std::endl;	
			agent_attribute.push_back(value_temp);
		 }
		 
		 //std::cout<< " agent_attribute[0]  : " << agent_attribute[0]  << "agent_attribute[1] :  " << agent_attribute[1]  << std::endl;
		 agent_attribute_vec.push_back(agent_attribute);
	 }

	 
	 std::string density_img_dir = "/home/kangneoung/sw_repo/dynamic_voronoi/src/dynamic_voronoi/test/density/density_200_200_ver3.png"; 
     param_nh_.getParam("density_img_dir",density_img_dir);
	 
     unsigned char* density_array = new unsigned char[map_height*map_width];
	
   
	 unsigned char* densityPtr = this->read_density(density_array, density_img_dir, map_height,map_width);
   
     float DropOutToleranceRate = 0.2;
	 
	 bool work_eff_flag = true;
   
	 DynamicVoronoi dynamicVoronoi(map_height, map_width, DropOutToleranceRate, weight_w, weight_h, lamda, false, work_eff_flag, false,densityPtr);
	 
	 std::string densitymap_dir = "/home/kangneoung/sw_repo/dynamic_voronoi/src/dynamic_voronoi/test/densitymap.txt";
	 std::string agentmap_dir = "/home/kangneoung/sw_repo/dynamic_voronoi/src/dynamic_voronoi/test/agentmap.txt";
	 std::string singlevoro_dir = "/home/kangneoung/sw_repo/dynamic_voronoi/src/dynamic_voronoi/test/singlevoro.txt";
		
	 dynamicVoronoi.saveDensityMap(densitymap_dir);
	 //dynamicVoronoi.saveAgentMap(agentmap_dir);
	 //dynamicVoronoi.saveSingleVoro(singlevoro_dir,0,0);
	 
     std::vector<std::vector<int>>::iterator iter;
	 
	 for (iter = agent_attribute_vec.begin(); iter != agent_attribute_vec.end(); iter++) {
		
		std::vector<int> agent_attribute;
		agent_attribute = *iter;
		dynamicVoronoi.PushPoint(agent_attribute.at(0), agent_attribute.at(1), agent_attribute.at(2), agent_attribute.at(3));   // agent_attribute.at(0)  is x_pose, agent_attribute.at(1)  is y_pose, agent_attribute.at(2)  is v_travel (pixel/time), agent_attribute.at(3)  is v_work (pixel/time)
     }   
	 
	 //dynamicVoronoi.PushPoint(15,15);
	 //dynamicVoronoi.PushPoint(15,30);
	 //dynamicVoronoi.PushPoint(30,30);
	 //dynamicVoronoi.PushPoint(45,45);
	 //dynamicVoronoi.PushPoint(15,70);
	 //dynamicVoronoi.PushPoint(70,70);
	 //dynamicVoronoi.PushPoint(70,15);
	 
	 dynamicVoronoi.PushDatum(dataum_x, dataum_y);
	 
	 std::string img_anima_dir = "/home/kangneoung/sw_repo/dynamic_voronoi/src/dynamic_voronoi/test/animation/";
	 std::string img_dir = "/home/kangneoung/sw_repo/dynamic_voronoi/src/dynamic_voronoi/test/map.png";
	 std::string img_anima_opt_dir = "/home/kangneoung/sw_repo/dynamic_voronoi/src/dynamic_voronoi/test/opt_animation/";
	 std::string label_dir = "/home/kangneoung/sw_repo/dynamic_voronoi/src/dynamic_voronoi/label/label.txt";
	 
	 bool is_optimize_animation = true;
	 
	 dynamicVoronoi.MainOptProcess(is_optimize_animation, img_anima_opt_dir, label_dir, 300, 0.1);
	 
	 //bool is_propagation_animation = true;
	 
	 //dynamicVoronoi.ExpandedVoronoi(is_propagation_animation, img_anima_dir, label_dir);
	 
	 //bool dummy = dynamicVoronoi.Colorized(img_dir, label_dir);
	 
     //dynamicVoronoi.saveAgentMap(agentmap_dir);
	
     //dynamicVoronoi.CentroidCal();
}

unsigned char* DynamicVoronoiRosUnitTest::read_density(unsigned char*  density_array, std::string img_dir, int height, int width)
{
	cv::Mat density_img = imread(img_dir, cv::IMREAD_GRAYSCALE);
	
	cv::resize(density_img, density_img, cv::Size(height, width));
    cv::Mat  density_img_thresh;

    //cv::threshold(density_img, density_img_thresh, 254, 254, cv::THRESH_TRUNC);
    density_img_thresh = density_img.clone();
	
	cv::Mat mat_255(cv::Size(density_img_thresh.rows, density_img_thresh.cols), CV_8UC1, cv::Scalar(255));
	
	//std::cout<< "debug 1" << "density_img_thresh.rows  : " << density_img_thresh.rows << " density_img_thresh.cols : "  << density_img_thresh.cols << " density_img_thresh.channels : "  << density_img_thresh.channels() << std::endl;
	
	cv::Mat converted_density;
	cv::absdiff(mat_255, density_img_thresh, converted_density);
	
	//std::cout<< "debug 2" << std::endl;
	cv::Mat density_resized;
	cv::resize(converted_density, density_resized, cv::Size(height, width));
	
	//std::cout<< "debug 3" << std::endl;
	//cv::imshow("validation", density_resized);
	//cv::waitKey(0);
	
	density_resized.convertTo(density_resized, CV_32FC1);
	
	
	//std::cout<< "debug 4" << std::endl;
	for(int row=0; row<density_resized.rows; row++)  
	{
      for(int col=0; col<density_resized.cols; col++)
	  {
		   int index =col + row*width;
		   density_array[index] =  (unsigned char)density_resized.at<float>(row, col);
		   
		   //std::cout<<" row: "<< row << " col: "<< col << " index: "<< index << " density_array[index] : "<< (float)density_array[index]  << "  density_resized.at<uchar>(row, col): "<<  density_resized.at<float>(row, col) <<std::endl;
	  }
	}		
	 
	std::cout<<" ************Read density information ***************" <<std::endl;
	
	return density_array;
	
}

void DynamicVoronoiRosUnitTest::anima_play()
{
	
    std::string testing_set_rgb_dir = "/home/kangneoung/sw_repo/dynamic_voronoi/src/dynamic_voronoi/test/animation_2_2";
	std::string testing_set_rgb_image_list_file = "img_file_list.txt";
	std::string full_dir_rgb_list = testing_set_rgb_dir +"/" +  testing_set_rgb_image_list_file ;
	
	std:: ifstream _rgb_in;
	
	_rgb_in.open(full_dir_rgb_list);
	
	std::string single_img_name;
	while(std::getline(_rgb_in,single_img_name))
	{
		std::string _rgb_image_file_full_path = testing_set_rgb_dir + "/" + single_img_name;
		
		cv::Mat img = cv::imread(_rgb_image_file_full_path,cv::IMREAD_COLOR);
		
		cv::resize(img, img, cv::Size(500, 500), cv::INTER_LINEAR);
		
        cv::imshow("Voronoi", img);
		cv::namedWindow("Voronoi",cv::WINDOW_NORMAL);
		cv::resizeWindow("Voronoi", 500,500);
	    cv::waitKey(100);
	}
	      
}

DynamicVoronoiRosUnitTest::~DynamicVoronoiRosUnitTest()
{
	
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_voronoi_test");
  ros::NodeHandle nh;
  ros::NodeHandle _nh("~");

  DynamicVoronoiRosUnitTest  dynamicvoronoi_rosunit_test(nh,_nh);
  dynamicvoronoi_rosunit_test.testrun();
  //dynamicvoronoi_rosunit_test.anima_play();
 
   return 0;
}