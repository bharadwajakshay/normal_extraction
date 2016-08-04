/*
 * normalextractor.cpp
 *
 *  Created on: Jul 23, 2016
 *      Author: nuc
 */

#include <normalextractor.h>

normal_extractor::normal_extractor()
{
	sub_pc = nh.subscribe("/structure/depth/points",1000,&normal_extractor::callbackpointcloud,this);
	sub_imu = nh.subscribe("/imuX",1000,&normal_extractor::callbackimu,this);
	pub_pcy = nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("plane_y", 1000);
	pub_pcz = nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("plane_z", 1000);
	pub_coeff = nh.advertise<pcl_msgs::ModelCoefficients>("plane_coeff_y",1000);
	pub_angle_plane = nh.advertise<std_msgs::Float64>("angle_plane",1000);
	pub_angle_imu = nh.advertise<std_msgs::Float64>("angle_imu",1000);
	angle_imu.data=0;
	angle_plane.data = 0;
	first_msg = true;
	first_msg_imu = true;
	rms_value = 0;
	current_time =0;
	previous_time =0;
	angle_first = 0;


}

normal_extractor::~normal_extractor()
{
	// TODO Auto-generated destructor stub
}


void normal_extractor::callbackpointcloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	pcl::PCLPointCloud2 pcl_pc2;

	// Convert to PCP PointCloud2 structure
	pcl_conversions::toPCL(*msg,pcl_pc2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_find_pc(new pcl::PointCloud<pcl::PointXYZ>);

    // Convert to PCL PointCloud structure
	pcl::fromPCLPointCloud2(pcl_pc2,*plane_find_pc);

	//Clean up the point cloud
	std::vector<int> indices_nan;
	pcl::removeNaNFromPointCloud(*plane_find_pc,*plane_find_pc,indices_nan);

	//Plane extraction
	pcl::PointCloud<pcl::PointXYZ>::Ptr z_plane_pc(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr y_plane_pc(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	z_plane_pc->header = plane_find_pc->header;
	y_plane_pc->header = plane_find_pc->header;
	tmp_cloud->header = plane_find_pc->header;
	ros_coeff.header = msg->header;
	z_plane_pc->clear();
	y_plane_pc->clear();
	tmp_cloud->clear();

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	Eigen::Vector3f axis;

	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.05);
	seg.setInputCloud(plane_find_pc);
	seg.setEpsAngle(0.261799); //Setting the angle epsilon (delta) threshold

	for (int ii=0;ii<2;ii++)
	{
		if (ii==0)
			//Set the axis along which we need to search for a model perpendicular to.
			//We want to fit line parallel to y - axis
			axis = Eigen::Vector3f(1.0,0.0,1.0);
		else
			//We want to fit line parallel to 0 - axis
			axis = Eigen::Vector3f(1.0,1.0,0.0);
		seg.setAxis(axis);
		seg.segment(*inliers,*coefficients );
		if(inliers->indices.size()==0)
		{
			if(ii==0)
				ROS_INFO("There are no planes parallel to y-axis in the point cloud\n");
			else
				ROS_INFO("There are no planes parallel to z-axis in the point cloud\n");
		}
		else
			if(ii==0)
				{
					std::cout<<"The Y Plane coefficents are\t"<<coefficients->values[0]<<"\t"
							<<coefficients->values[1]<<"\t"<<coefficients->values[2]<<"\t"
							<<coefficients->values[3]<<std::endl;
					pcl_conversions::fromPCL(*coefficients,ros_coeff);
					if(first_msg)
					{
						first_coeff = ros_coeff;
						//first_msg = false;
						rms_value = sqrt((pow((double)first_coeff.values[1],2)+pow((double)first_coeff.values[2],2)+pow((double)first_coeff.values[3],2)));
					}
					pub_coeff.publish(ros_coeff);
				}
			else
				std::cout<<"The Z Plane coefficents are\t"<<coefficients->values[0]<<"\t"
											<<coefficients->values[1]<<"\t"<<coefficients->values[2]<<"\t"
											<<coefficients->values[3]<<std::endl;

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud (plane_find_pc);
		extract.setIndices (inliers);
		extract.setNegative (false);
		if (ii==0)
			extract.filter(*y_plane_pc);
		else
			extract.filter(*z_plane_pc);
		extract.setNegative(true);
		extract.filter(*tmp_cloud);
		*plane_find_pc = *tmp_cloud;

	}
	pcl::copyPointCloud(*y_plane_pc, y_plane);
	pcl::copyPointCloud(*z_plane_pc, z_plane);

	for (int i=0;i<y_plane.size();i++)
		y_plane.points[i].intensity = 200;
	for (int i=0;i<z_plane.size();i++)
		z_plane.points[i].intensity = 50;

	pub_pcy.publish(y_plane);
	pub_pcz.publish(z_plane);

	//claulating the angle between 1st normal and consequent normals
	double num=abs((double)((first_coeff.values[1]*ros_coeff.values[1])+(first_coeff.values[2]*ros_coeff.values[2])+(first_coeff.values[3]*ros_coeff.values[3])));
	double rms_value2 =sqrt((pow((double)ros_coeff.values[1],2)+pow((double)ros_coeff.values[2],2)+pow((double)ros_coeff.values[3],2)));
	double den = rms_value * rms_value2;
	double angle = acos(num/den) * (180/M_PI);
	// Correcting for the offset (Check why you are getting an offset)
	if(first_msg)
	{
		angle_first = angle;
		first_msg = false;
	}
	angle_plane.data = angle - angle_first;
	pub_angle_plane.publish(angle_plane);

}

void normal_extractor::callbackimu(const xsens_slim::imuX::ConstPtr& msg)
{
	double yaw_vel = msg->gyro.z - imu_bias_yaw;
	current_time = msg->header.stamp.toSec();
	if (first_msg_imu)
	{
		previous_time=current_time;
		first_msg_imu = false;
	}
	else
	{
		double dt=current_time - previous_time;
		previous_time = current_time;
		angle_imu.data = angle_imu.data-((dt * yaw_vel)* (180/M_PI));
		pub_angle_imu.publish(angle_imu);
	}

}
