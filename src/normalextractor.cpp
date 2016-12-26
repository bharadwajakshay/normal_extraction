/*
 * normalextractor.cpp
 *
 *  Created on: Jul 23, 2016
 *      Author: nuc
 */

#include <normalextractor.h>

normal_extractor::normal_extractor()
{
	sub_pc = nh.subscribe("/camera/lepton/depth_registered/points",1000,&normal_extractor::callbackpointcloud,this);
	sub_imu = nh.subscribe("/mti/imuX",1000,&normal_extractor::callbackimu,this);
	pub_pcy = nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("plane_y", 1000);
	pub_pcz = nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("plane_z", 1000);
	pub_coeff = nh.advertise<pcl_msgs::ModelCoefficients>("plane_coeff_y",1000);
	pub_angle_plane = nh.advertise<geometry_msgs::Vector3Stamped>("angle_plane",1000);
	pub_angle_imu = nh.advertise<geometry_msgs::Vector3Stamped>("angle_imu",1000);
	angle_imu.data=0;
	angle_plane.data = 0;
	first_msg = true;
	first_msg_imu = true;
	rms_value = 0;
	current_time =0;
	previous_time =0;
	angle_first = 0;
	angle_first_imu = 0;
	rpy_timetag = 0;


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

	//add time tag into the inertial message
	geometry_msgs::Vector3Stamped plane_angle;
	plane_angle.header = msg->header;

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
						rms_value = sqrt((pow((double)first_coeff.values[0],2)+pow((double)first_coeff.values[1],2)+pow((double)first_coeff.values[2],2)));
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
	//debugging
	std::cout<<"The first_coeff coefficents are\t"<<first_coeff.values[0]<<"\t"
								<<first_coeff.values[1]<<"\t"<<first_coeff.values[2]<<"\t"
								<<first_coeff.values[3]<<std::endl;
	std::cout<<"The ros_coeff coefficents are\t"<<ros_coeff.values[0]<<"\t"
									<<ros_coeff.values[1]<<"\t"<<ros_coeff.values[2]<<"\t"
									<<ros_coeff.values[3]<<std::endl;
	double num=std::fabs((float)((first_coeff.values[0]*ros_coeff.values[0])+(first_coeff.values[1]*ros_coeff.values[1])+(first_coeff.values[2]*ros_coeff.values[2])));
	double rms_value2 =sqrt((pow((double)ros_coeff.values[0],2)+pow((double)ros_coeff.values[1],2)+pow((double)ros_coeff.values[2],2)));
	double den = rms_value * rms_value2;
	double angle = acos(num/den) * (180/M_PI);
	// Correcting for the offset (Check why you are getting an offset)
	if(first_msg)
	{
		angle_first = angle;
		first_msg = false;
	}
	plane_angle.vector.x = 0.0;
	plane_angle.vector.y = 0.0;
	plane_angle.vector.z = angle - angle_first;
	pub_angle_plane.publish(plane_angle);

}

Matrix3d normal_extractor::NormalizeDCM(Matrix3d DCM)
{
	float renorm=0;
	int problem=0;

	Vector3d Xrow = DCM.row(0);
	Vector3d Yrow = DCM.row(1);

	float error = Xrow.dot(Yrow);

	Vector3d Xortho = Xrow - ( 0.5 * error * Yrow );
	Vector3d Yortho = Yrow - ( 0.5 * error * Xrow );
	Vector3d Zortho = Xortho.cross(Yrow);

	renorm= Xortho.dot(Xortho);
	if (renorm < 1.5625f && renorm > 0.64f)
	{
	    renorm= .5 * (3-renorm);                                                 //eq.21
	} else if (renorm < 100.0f && renorm > 0.01f)
		{
			renorm= 1. / sqrt(renorm);
			//cout << "Square root called in renormalization" << endl;
	  } else
		  {
			  problem = 1;
		  }
	Vector3d Xnorm = renorm * Xortho;

	renorm= Yortho.dot(Yortho);
	if (renorm < 1.5625f && renorm > 0.64f)
	{
	    renorm= .5 * (3-renorm);                                                 //eq.21
	} else if (renorm < 100.0f && renorm > 0.01f)
		{
			renorm= 1. / sqrt(renorm);
			//cout << "Square root called in renormalization" << endl;
	  } else
		  {
			  problem = 1;
		  }
	Vector3d Ynorm = renorm * Yortho;


	renorm= Zortho.dot(Zortho);
	if (renorm < 1.5625f && renorm > 0.64f)
	{
	    renorm= .5 * (3-renorm);                                                 //eq.21
	}
	else if (renorm < 100.0f && renorm > 0.01f)
		{
			renorm= 1. / sqrt(renorm);
			//cout << "Square root called in renormalization" << endl;
		}
	else
	{
	  problem = 1;
	}
	Vector3d Znorm = renorm * Zortho;

	Matrix3d normDCM;

	if (!problem)
	{
		normDCM.row(0) = Xnorm;
		normDCM.row(1) = Ynorm;
		normDCM.row(2) = Znorm;
	}
	else
	{
		// Our solution is blowing up and we will force back to initial condition.  Hope we are not upside down!
		normDCM.setIdentity();
		// cout << "Our solution is blowing up and we will force back to initial condition.  Hope we are not upside down" << endl;
	}

	return normDCM;
}

Eigen::Vector3d normal_extractor::DCMToEuler(Matrix3d Cbn)
{
	Vector3d EulerAngles;

 	EulerAngles(0) = atan2(Cbn(2,1),Cbn(2,2)); 	// Roll
  	EulerAngles(1) = asin(-Cbn(2,0));			// Pitch
  	EulerAngles(2) = atan2(Cbn(1,0),Cbn(0,0));  // Yaw

  	return(EulerAngles);
}

Eigen::Matrix3d normal_extractor::EulerToDCM(Vector3d rpy)
{
	Matrix3d Cbn;
	double	phi, theta, psi;
	double	cphi, sphi, ctheta, stheta, cpsi, spsi;

	phi 	= rpy(0);
	theta 	= rpy(1);
	psi 	= rpy(2);

	cphi = cos(phi);
	sphi = sin(phi);
	ctheta = cos(theta);
	stheta = sin(theta);
	cpsi = cos(psi);
	spsi = sin(psi);

	Cbn(0,0) 	=	ctheta*cpsi;
	Cbn(0,1) 	=	-cphi*spsi + sphi*stheta*cpsi;
	Cbn(0,2)	=	sphi*spsi +  cphi*stheta*cpsi;

	Cbn(1,0)	=	ctheta*spsi;
	Cbn(1,1)	=	cphi*cpsi + sphi*stheta*spsi;
	Cbn(1,2)	=	-sphi*cpsi +  cphi*stheta*spsi;

	Cbn(2,0)	=	-stheta;
	Cbn(2,1)	=	sphi*ctheta;
	Cbn(2,2)	=	cphi*ctheta;

  	return(Cbn);
}
void normal_extractor::callbackimu(const xsens_slim::imuX::ConstPtr& msg)
{
	double q[4];
	static bool first_message = true;
	static bool leveling_on = true;
	static double intial_time;
	static int num_accum = 0;
	geometry_msgs::Vector3Stamped inertial_angle;
	inertial_angle.header = msg->header;

	Eigen::Vector3d chi;
	Eigen::Matrix3d chi_cross;
	Eigen::Matrix3d Cbb;
	Eigen::Matrix3d I;
	I.setIdentity();

	if(first_message)
	{
		rpy.setZero();

		rpy_timetag = msg->header.stamp.sec + msg->header.stamp.nsec/1.0e9;

		intial_time  = rpy_timetag;

		Cbn.setIdentity();

		chi_bias(0) = 0.0;
		chi_bias(1) = 0.0;
		chi_bias(2) = 0.0;

		accel_bias(0) = 0.0;
		accel_bias(1) = 0.0;
		accel_bias(2) = 0.0;

		first_message = false;

		cout << "START LEVELLING ..." << endl;
	}

	if(leveling_on)
	{
		// Accumulate the
		chi_bias(0) += msg->gyro.x;
		chi_bias(1) += msg->gyro.y;
		chi_bias(2) += msg->gyro.z;

		accel_bias(0) += msg->acc.x;
		accel_bias(1) += msg->acc.y;
		accel_bias(2) += msg->acc.z;

		num_accum++;

		rpy_timetag = msg->header.stamp.sec + msg->header.stamp.nsec/1.0e9;

		if( (rpy_timetag - intial_time) > 10.0)
		{
			cout << "DONE LEVELLING ..." << endl;
			leveling_on = false;

			chi_bias(0) /= (double)num_accum;
			chi_bias(1) /= (double)num_accum;
			chi_bias(2) /= (double)num_accum;

			accel_bias(0) /= (double)num_accum;
			accel_bias(1) /= (double)num_accum;
			accel_bias(2) /= (double)num_accum;

			// Use Delta V bias to compute the gravity
			double fnorm 	= sqrt(	accel_bias(1)*accel_bias(1)+
									accel_bias(2)*accel_bias(2));

			// Compute inital Euler angle estimates
			Vector3d rpyTemp;
			rpyTemp(0) 	= atan2(accel_bias(1), accel_bias(2));
			rpyTemp(1) 	= atan(accel_bias(0)/fnorm);
			rpyTemp(2) 	= 0.0;

			Cbn = this->EulerToDCM(rpyTemp);
		}
	}
	else
	{
		double current_timetag = msg->header.stamp.sec + msg->header.stamp.nsec/1.0e9;
		double dt = 0.01; //current_timetag - rpy_timetag;

		// Set up the chi vector
		chi(0) = dt*(msg->gyro.x-chi_bias(0));
		chi(1) = dt*(msg->gyro.y-chi_bias(1));
		chi(2) = dt*(msg->gyro.z-chi_bias(2));

		chi_cross(0,0) = 0.0;
		chi_cross(0,1) = -1.0*chi(2);
		chi_cross(0,2) = chi(1);
		chi_cross(1,0) = chi(2);
		chi_cross(1,1) = 0.0;
		chi_cross(1,2) = -1.0*chi(0);
		chi_cross(2,0) = -1.0*chi(1);
		chi_cross(2,1) = chi(0);
		chi_cross(2,2) = 0.0;

		Cbb = I + chi_cross + 0.5*chi_cross*chi_cross + (1.0/6.0)*chi_cross*chi_cross*chi_cross;

		// Update the DCM
		Cbn = Cbn*Cbb;

		//Normalize DCM
		Cbn = this->NormalizeDCM(Cbn);

		// Obtain the Euler angles
		rpy = this->DCMToEuler(Cbn);  /// Fix for wander angle

		rpy_timetag = current_timetag;
		/**************************************
		 * imu_angle(0) = roll
		 * imu_angle(1) = pitch
		 * imu_angle(2) = yaw
		 */

		imu_angle(0)	= (180.0/3.1415926535898)*rpy(0);
		imu_angle(1) 	= (180.0/3.1415926535898)*rpy(1);
		imu_angle(2) 	= (180.0/3.1415926535898)*rpy(2);

		//cout << "RPY: " << dt << ", " << (180.0/3.1415926535898)*rpy(0) << ", " << (180.0/3.1415926535898)*rpy(1) << ", " << (180.0/3.1415926535898)*rpy(2) << "; " << endl;
	}

	//imu_ready = true;
	// q[0] = msg->orientation.x;
	// q[1] = msg->orientation.y;
	// q[2] = msg->orientation.z;
	// q[3] = msg->orientation.w;

	// // Roll angle
 //    roll = (180.0/3.1415926535898)*atan2(2*(q[2]*q[3]+q[0]*q[1]), 1 - 2.0*(q[1]*q[1] + q[2]*q[2])); //(q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3]));

 //    // Pitch angle
 //    pitch = (180.0/3.1415926535898)*asin(-2.0*(q[1]*q[3]-q[0]*q[2]));

 //    // Yaw angle
 //    yaw = (180.0/3.1415926535898)*atan2( 2*(q[1]*q[2]+q[0]*q[3]), 1 - 2.0*(q[2]*q[2] + q[3]*q[3])); //(q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3]));

    //cout << "RPY: " << roll << ", " << pitch << ", " << yaw << "; " << (q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]) << endl;

	//publish imu angles
	//inertial_angle.vector = imu_angle.cast<float>();
	inertial_angle.vector.x = imu_angle(0);
	inertial_angle.vector.y = imu_angle(1);
	inertial_angle.vector.z = imu_angle(2);
	pub_angle_imu.publish(inertial_angle);
}
