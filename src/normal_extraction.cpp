#include "normalextractor.h"

std::string topic_pc;
std::string topic_imu;

int main(int argc, char **argv)
{
	ros::init(argc,argv,"normal_extraction");
	normal_extractor obj;
	ros::NodeHandle pnh("~");
	pnh.param("topic_pc",topic_pc,std::string("/camera/lepton/depth_registered/points"));
	pnh.param("topic_imu",topic_imu,std::string("/mti/imuX"));
	ros::spin();
	return(0);
}
