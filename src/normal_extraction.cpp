#include "normalextractor.h"

int main(int argc, char **argv)
{
	ros::init(argc,argv,"normal_extraction");
	normal_extractor obj;
	ros::spin();
	return(0);
}
