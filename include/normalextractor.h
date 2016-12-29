/*
 * normalextractor.h
 *
 *  Created on: Jul 23, 2016
 *      Author: nuc
 */
#ifndef NORMAL_EXTRACTION_INCLUDE_NORMALEXTRACTOR_H_
#define NORMAL_EXTRACTION_INCLUDE_NORMALEXTRACTOR_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float64.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Imu.h>

#include <pcl/common/common_headers.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <pcl/octree/octree.h>
#include "pcl_ros/point_cloud.h"

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/registration/icp.h>
#include "pcl_ros/point_cloud.h"

#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <math.h>
#include <xsens_slim/imuX.h>

#define imu_bias_yaw 0.017156139726883

extern std::string topic_pc;
extern std::string topic_imu;

using namespace Eigen;

class normal_extractor {
public:
	normal_extractor();
	virtual ~normal_extractor();
	void callbackpointcloud(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void callbackimu(const xsens_slim::imuX::ConstPtr& msg);
	//Additional function for verification
	pcl::ModelCoefficients plane_est_svd(pcl::PointCloud<pcl::PointXYZ>::ConstPtr point_cloud);

	// Addidng functions to calcluate the iMU rpy
	Eigen::Matrix3d EulerToDCM(Vector3d rpy);
	Eigen::Vector3d DCMToEuler(Matrix3d Cbn);
	Eigen::Matrix3d NormalizeDCM(Matrix3d DCM);
	pcl::PointCloud<pcl::PointXYZI> y_plane;
	pcl::PointCloud<pcl::PointXYZI> z_plane;
	pcl_msgs::ModelCoefficients ros_coeff;
	pcl_msgs::ModelCoefficients first_coeff;
	std_msgs::Float64 angle_plane;
	bool first_msg, first_msg_imu;
	double rms_value;
	double previous_time, current_time;
	std_msgs::Float64 angle_imu;
	double angle_first, angle_first_imu;
	Eigen::Matrix3d Cbn;
	Eigen::Vector3d rpy;
	double rpy_timetag;
	Eigen::Vector3d chi_bias;
	Eigen::Vector3d accel_bias;
	Eigen::Vector3d imu_angle;

private:
	ros::NodeHandle nh;
	ros::Subscriber sub_pc;
	ros::Subscriber sub_imu;
	ros::Publisher pub_pcy;
	ros::Publisher pub_pcz;
	ros::Publisher pub_coeff;
	ros::Publisher pub_angle_plane;
	ros::Publisher pub_angle_imu;


};

#endif /* NORMAL_EXTRACTION_INCLUDE_NORMALEXTRACTOR_H_ */
