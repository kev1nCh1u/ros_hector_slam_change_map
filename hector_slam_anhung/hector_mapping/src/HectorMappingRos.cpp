//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include "HectorMappingRos.h"

#include "map/GridMap.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include "sensor_msgs/PointCloud2.h"

#include "HectorDrawings.h"
#include "HectorDebugInfoProvider.h"
#include "HectorMapMutex.h"


#include <time.h>

#ifndef TF_SCALAR_H
typedef btScalar tfScalar;
#endif




clock_t pre_allmsg = 0;
clock_t now_allmsg = 0;


HectorMappingRos::HectorMappingRos()
	: debugInfoProvider(0)
	, hectorDrawings(0)
	, lastGetMapUpdateIndex(-100)
	, tfB_(0)
	, map__publish_thread_(0)
	  , initial_pose_set_(false)
{
	LoadTitlePath();

	//========================================================================================//


	//=====Anhung==========Read parameter from mapping_default.launch===============//
	ros::NodeHandle private_nh_("~");

	std::string mapTopic_ = "map";

	private_nh_.param("pub_drawings", p_pub_drawings, false);
	private_nh_.param("pub_debug_output", p_pub_debug_output_, false);
	private_nh_.param("pub_map_odom_transform", p_pub_map_odom_transform_,true);
	private_nh_.param("pub_odometry", p_pub_odometry_,false);
	private_nh_.param("advertise_map_service", p_advertise_map_service_,true);
	private_nh_.param("scan_subscriber_queue_size", p_scan_subscriber_queue_size_, 5);

	private_nh_.param("map_resolution", p_map_resolution_, 0.025);
	private_nh_.param("map_size", p_map_size_, 1024);
	private_nh_.param("map_start_x", p_map_start_x_, 0.5);
	private_nh_.param("map_start_y", p_map_start_y_, 0.5);
	private_nh_.param("map_multi_res_levels", p_map_multi_res_levels_, 3);

	private_nh_.param("update_factor_free", p_update_factor_free_, 0.4);
	private_nh_.param("update_factor_occupied", p_update_factor_occupied_, 0.9);

	private_nh_.param("map_update_distance_thresh", p_map_update_distance_threshold_, 0.4);
	private_nh_.param("map_update_angle_thresh", p_map_update_angle_threshold_, 0.9);

	private_nh_.param("scan_topic", p_scan_topic_, std::string("scan"));
	private_nh_.param("sys_msg_topic", p_sys_msg_topic_, std::string("syscommand"));
	private_nh_.param("pose_update_topic", p_pose_update_topic_, std::string("poseupdate"));

	private_nh_.param("use_tf_scan_transformation", p_use_tf_scan_transformation_,true);
	private_nh_.param("use_tf_pose_start_estimate", p_use_tf_pose_start_estimate_,false);
	private_nh_.param("map_with_known_poses", p_map_with_known_poses_, false);

	private_nh_.param("base_frame", p_base_frame_, std::string("base_link"));
	private_nh_.param("map_frame", p_map_frame_, std::string("map"));
	private_nh_.param("odom_frame", p_odom_frame_, std::string("odom"));

	private_nh_.param("pub_map_scanmatch_transform", p_pub_map_scanmatch_transform_,true);
	private_nh_.param("tf_map_scanmatch_transform_frame_name", p_tf_map_scanmatch_transform_frame_name_, std::string("scanmatcher_frame"));

	private_nh_.param("output_timing", p_timing_output_,false);

	private_nh_.param("map_pub_period", p_map_pub_period_, 2.0);

	double tmp = 0.0;
	private_nh_.param("laser_min_dist", tmp, 0.4);
	p_sqr_laser_min_dist_ = static_cast<float>(tmp*tmp);

	private_nh_.param("laser_max_dist", tmp, 30.0);
	//private_nh_.param("laser_max_dist", tmp, 5.0);
	p_sqr_laser_max_dist_ = static_cast<float>(tmp*tmp);

	private_nh_.param("laser_z_min_value", tmp, -1.0);
	p_laser_z_min_value_ = static_cast<float>(tmp);

	private_nh_.param("laser_z_max_value", tmp, 1.0);
	p_laser_z_max_value_ = static_cast<float>(tmp);
	//=====Anhung==========Read parameter from mapping_default.launch===============//

	//=====Anhung========================Draw Interface=============================//
	if (p_pub_drawings)
	{
		ROS_INFO("HectorSM publishing debug drawings");
		hectorDrawings = new HectorDrawings();
	}

	if(p_pub_debug_output_)
	{
		ROS_INFO("HectorSM publishing debug info");
		debugInfoProvider = new HectorDebugInfoProvider();
	}

	if(p_pub_odometry_)
	{
		odometryPublisher_ = node_.advertise<nav_msgs::Odometry>("scanmatch_odom", 50);
	}
	//=====Anhung========================Draw Interface=============================//


	scan_data_publisher_ = node_.advertise<sensor_msgs::LaserScan>("Debug_scan_1", 10);
	scan_data_publisher_2 = node_.advertise<sensor_msgs::LaserScan>("Debug_scan_2", 10);

	slamProcessor = NULL;

	p_control_state_ = P_SLAM_STATE_Idle;


	std::cout<<"=================================Anhung==============================="<<std::endl;
	ROS_INFO("HectorSM p_base_frame_: %s", p_base_frame_.c_str());
	ROS_INFO("HectorSM p_map_frame_: %s", p_map_frame_.c_str());
	ROS_INFO("HectorSM p_odom_frame_: %s", p_odom_frame_.c_str());
	ROS_INFO("HectorSM p_scan_topic_: %s", p_scan_topic_.c_str());
	ROS_INFO("HectorSM p_use_tf_scan_transformation_: %s", p_use_tf_scan_transformation_ ? ("true") : ("false"));
	ROS_INFO("HectorSM p_pub_map_odom_transform_: %s", p_pub_map_odom_transform_ ? ("true") : ("false"));
	ROS_INFO("HectorSM p_scan_subscriber_queue_size_: %d", p_scan_subscriber_queue_size_);
	ROS_INFO("HectorSM p_map_pub_period_: %f", p_map_pub_period_);
	ROS_INFO("HectorSM p_update_factor_free_: %f", p_update_factor_free_);
	ROS_INFO("HectorSM p_update_factor_occupied_: %f", p_update_factor_occupied_);
	ROS_INFO("HectorSM p_map_update_distance_threshold_: %f ", p_map_update_distance_threshold_);
	ROS_INFO("HectorSM p_map_update_angle_threshold_: %f", p_map_update_angle_threshold_);
	ROS_INFO("HectorSM p_laser_z_min_value_: %f", p_laser_z_min_value_);
	ROS_INFO("HectorSM p_laser_z_max_value_: %f", p_laser_z_max_value_);
	std::cout<<"=================================Anhung==============================="<<std::endl;


	//=====Anhung=====================odometry predict==========================//
	isReceiveOdom = false;
	isInitialTimeSample = false;
	p_time = 0;
	odom_vx = 0.0;
	odom_vy = 0.0;
	odom_w = 0.0;

	scanSubscriber_ = node_.subscribe(p_scan_topic_, p_scan_subscriber_queue_size_, &HectorMappingRos::scanCallback, this);
	sysMsgSubscriber_ = node_.subscribe(p_sys_msg_topic_, 2, &HectorMappingRos::sysMsgCallback, this);

	poseUpdatePublisher_ = node_.advertise<geometry_msgs::PoseWithCovarianceStamped>(p_pose_update_topic_, 1, false);
	posePublisher_ = node_.advertise<geometry_msgs::PoseStamped>("slam_out_pose", 1, false);




	//=====Anhung=============================UnKnow================================//
	scan_point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud>("slam_cloud",1,false);

	tfB_ = new tf::TransformBroadcaster();
	ROS_ASSERT(tfB_);

	//=====Anhung=============================UnKnow================================//



	//=====Anhung=======================bool navigation ============================//
	p_navigation_ = true;
	//=====Anhung=======================bool navigation ============================//


	//=====Anhung=======================bool re-location ===========================//
	p_Relocation_ = false;
	//=====Anhung=======================bool re-location ===========================//



	//=====Anhung====================Socket Initialization =========================//
	/*
	   p_slen_=sizeof(p_si_other_);

	   if ((p_s_socket_=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
	   diep("socket");

	   memset((char *) &p_si_me_, 0, sizeof(p_si_me_));

	   p_si_me_.sin_family = AF_INET;

	   p_si_me_.sin_port = htons(P_SOCKET_PORT);

	   p_si_me_.sin_addr.s_addr = htonl(INADDR_ANY);

	   if (bind(p_s_socket_, (struct sockaddr *)&p_si_me_, sizeof(p_si_me_))==-1)
	   diep("bind");
	 */
	//=====Anhung====================Socket Initialization =========================//



	//=====Anhung====================Subscribe Initial pose=========================//
	initial_pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(node_, "initialpose", 2);
	initial_pose_filter_ = new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(*initial_pose_sub_, tf_, p_map_frame_, 2);
	initial_pose_filter_->registerCallback(boost::bind(&HectorMappingRos::initialPoseCallback, this, _1));
	//=====Anhung====================Subscribe Initial pose=========================//


	//=====Anhung=========================Publish Goal pose ======================//
	goal_publisher_ = node_.advertise<geometry_msgs::PoseStamped>("Nav_goal",10);

	//=====Anhung====================Publish missing signal ======================//
	miss_publisher_ = node_.advertise<std_msgs::Int16>("Missing",10);



	//=====Anhung====================Subscribe Goal pose=========================//
	goal_pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "move_base_simple/goal", 2);
	goal_pose_filter_ = new tf::MessageFilter<geometry_msgs::PoseStamped>(*goal_pose_sub_, tf_, p_map_frame_, 2);
	goal_pose_filter_->registerCallback(boost::bind(&HectorMappingRos::goalPoseCallback, this, _1));
	//=====Anhung====================Subscribe Goal pose=========================//



	//=====Anhung=================Subscribe Relocation pose======================//
	Relocation_pose_sub_ = node_.subscribe("RePose", 5, &HectorMappingRos::RePoseCallback, this);
	//=====Anhung=================Subscribe Relocation pose======================//



	//=====Anhung===================Receive odometry speed ======================//
	OdomSpeedSubscriber_ = node_.subscribe("odom_speed", 5, &HectorMappingRos::OdomSpeedCallback, this);
	//=====Anhung===================Receive odometry speed ======================//



	map__publish_thread_ = new boost::thread(boost::bind(&HectorMappingRos::publishMapLoop, this, p_map_pub_period_));

	//=====Anhung====================control thread ================================//
	/*
	   anhung_control_thread_ = new boost::thread(boost::bind(&HectorMappingRos::AnHung_Control, this, 0.5));
	 */


	commandSubscriber_ = node_.subscribe("Command", 5, &HectorMappingRos::commandCallback, this);

	//=====Anhung====================control thread ================================//

	map_to_odom_.setIdentity();

	lastMapPublishTime = ros::Time(0,0);

	//========================================================================================//
	LoadMap(); //kevin
	Error_Publisher_= node_.advertise<std_msgs::Int8>("Error",10);

	protect_map = "";

}

HectorMappingRos::~HectorMappingRos()
{
	delete slamProcessor;

	if (hectorDrawings)
		delete hectorDrawings;

	if (debugInfoProvider)
		delete debugInfoProvider;

	if (tfB_)
		delete tfB_;

	if(map__publish_thread_)
		delete map__publish_thread_;

	//=====Anhung====================control thread ================================//
	/*
	   if(anhung_control_thread_)
	   delete anhung_control_thread_;
	 */
	//=====Anhung=========================Socket close ============================//
	//close(p_s_socket_);

}

void HectorMappingRos::scanCallback(const sensor_msgs::LaserScan& scan)
{

	//=====Anhung=================================================================================//
	//20181202

	ros::WallTime startTime = ros::WallTime::now();

	double time_sample = ros::Time::now().toSec();


	if(slamProcessor != NULL){

		if(p_control_state_ == P_SLAM_STATE_RUN || p_control_state_ == P_SLAM_STATE_Re_Location){

			ros::Duration dur (0.5);

			device_num = checkDeviceNum(scan);

			if(device_num == 1){
				std::cout<<"scan.header "<<scan.header<<std::endl;

				device_scan_1.header          = scan.header;
				device_scan_1.angle_min       = scan.angle_min;
				device_scan_1.angle_max       = scan.angle_max;
				device_scan_1.angle_increment = scan.angle_increment;
				device_scan_1.time_increment  = scan.time_increment;
				device_scan_1.scan_time       = scan.scan_time;
				device_scan_1.range_min       = scan.range_min;
				device_scan_1.range_max       = scan.range_max;
				device_scan_1.ranges.resize(scan.ranges.size());
				device_scan_1.intensities.resize(scan.ranges.size());

				for(int i=0; i<scan.ranges.size(); i++)
					device_scan_1.ranges[i] = scan.ranges[i];



			}
			else{

				device_scan_1.header          = scan.header;
				device_scan_1.angle_min       = scan.angle_min;
				device_scan_1.angle_max       = scan.angle_max;
				device_scan_1.angle_increment = scan.angle_increment;
				device_scan_1.time_increment  = scan.time_increment;
				device_scan_1.scan_time       = scan.scan_time;
				device_scan_1.range_min       = scan.range_min;
				device_scan_1.range_max       = scan.range_max;
				device_scan_1.ranges.resize(scan.ranges.size()/2);
				device_scan_1.intensities.resize(scan.ranges.size()/2);

				device_scan_2.header          = scan.header;
				std::string frame_id_2_       = std::string("base_laser_link_2");
				device_scan_2.header.frame_id = frame_id_2_;
				device_scan_2.angle_min       = scan.angle_min;
				device_scan_2.angle_max       = scan.angle_max;
				device_scan_2.angle_increment = scan.angle_increment;
				device_scan_2.time_increment  = scan.time_increment;
				device_scan_2.scan_time       = scan.scan_time;
				device_scan_2.range_min       = scan.range_min;
				device_scan_2.range_max       = scan.range_max;
				device_scan_2.ranges.resize(scan.ranges.size()/2);
				device_scan_2.intensities.resize(scan.ranges.size()/2);

				for(int i=0; i<(scan.ranges.size()/2); i++){
					device_scan_1.ranges[i] = scan.ranges[i];
					device_scan_2.ranges[i] = scan.ranges[i+(scan.ranges.size()/2)];
				}

			}


			if(device_num == 1)
			{
				scan_data_publisher_.publish(device_scan_1);

				std::cout<<"scan.ranges.size() "<<scan.ranges.size()<<std::endl;
				std::cout<<"p_base_frame_ "<<p_base_frame_<<std::endl;
				std::cout<<"device_scan_1.header.frame_id "<<device_scan_1.header.frame_id<<std::endl;
				std::cout<<"device_scan_1.header.stamp "<<device_scan_1.header.stamp<<std::endl;
				if(tf_.waitForTransform(p_base_frame_,device_scan_1.header.frame_id, device_scan_1.header.stamp,dur))
				{
					std::cout<<"GG "<<std::endl;
					tf::StampedTransform laserTransform_1;
					tf::StampedTransform laserTransform_2;

					tf_.lookupTransform(p_base_frame_,device_scan_1.header.frame_id, device_scan_1.header.stamp, laserTransform_1);

					std::cout<<"GG "<<std::endl;
					projector_1_.projectLaser(device_scan_1, laser_point_cloud_1_,20.0);

					Eigen::Vector3f startEstimate(Eigen::Vector3f::Zero());

											if(rosPointCloudToDataContainer(laser_point_cloud_1_, laser_point_cloud_2_,
									laserTransform_1, laserTransform_2,
									laserScanContainer_1, laserScanContainer_2,
									slamProcessor->getScaleToMap(),
									device_num))
						{

							if(p_control_state_ == P_SLAM_STATE_RUN){

								if (initial_pose_set_){
									initial_pose_set_ = false;
									startEstimate = initial_pose_;
								}
								else{
									startEstimate = slamProcessor->getLastScanMatchPose();
								}


								//predict control
								if(isInitialTimeSample && isReceiveOdom && p_navigation_){

									//control input
									float delta_t = float(time_sample - p_time);

									float p_x2 = startEstimate.x() + (odom_vx*cos(startEstimate.z()) - odom_vy*sin(startEstimate.z()))*delta_t;
									float p_y2 = startEstimate.y() + (odom_vx*sin(startEstimate.z()) + odom_vy*cos(startEstimate.z()))*delta_t;
									float yaw2 = startEstimate.z() + odom_w*delta_t;

									NormalizeAngle(yaw2);

									startEstimate[0] = p_x2;
									startEstimate[1] = p_y2;
									startEstimate[2] = yaw2;


								}

								slamProcessor->update(laserScanContainer_1, laserScanContainer_2, device_num, startEstimate, p_Relocation_, false, p_navigation_);

								slamProcessor->UpdateMap(laserScanContainer_1, laserScanContainer_2, device_num, false, p_navigation_);

								poseInfoContainer_.update(slamProcessor->getLastScanMatchPose(), slamProcessor->getLastScanMatchCovariance(), ros::Time(0), p_map_frame_);

								poseUpdatePublisher_.publish(poseInfoContainer_.getPoseWithCovarianceStamped());

								posePublisher_.publish(poseInfoContainer_.getPoseStamped());


								if(p_pub_odometry_)
								{
									nav_msgs::Odometry tmp;
									tmp.pose = poseInfoContainer_.getPoseWithCovarianceStamped().pose;

									tmp.header = poseInfoContainer_.getPoseWithCovarianceStamped().header;
									tmp.child_frame_id = p_base_frame_;
									odometryPublisher_.publish(tmp);
								}

								if (p_pub_map_odom_transform_)
								{
									tf::StampedTransform odom_to_base;

									try
									{
										tf_.waitForTransform(p_odom_frame_, p_base_frame_, device_scan_1.header.stamp, ros::Duration(0.5));
										tf_.lookupTransform(p_odom_frame_, p_base_frame_, device_scan_1.header.stamp, odom_to_base);
									}
									catch(tf::TransformException e)
									{
										ROS_ERROR("Transform failed during publishing of map_odom transform: %s",e.what());
										odom_to_base.setIdentity();
									}
									map_to_odom_ = tf::Transform(poseInfoContainer_.getTfTransform() * odom_to_base.inverse());
									tfB_->sendTransform( tf::StampedTransform (map_to_odom_, device_scan_1.header.stamp, p_map_frame_, p_odom_frame_));
								}


								if(p_navigation_){
									std_msgs::Int16 msg_missing;

									std::cout<<"slamProcessor->ProbMapping"<<slamProcessor->ProbMapping<<std::endl;
									if(slamProcessor->ProbMapping <= 0.35){
										msg_missing.data = 1;
										std::cout<<"missing  "<<std::endl;
									}
									else{
										msg_missing.data = 0;
										std::cout<<"no_missing  "<<msg_missing<<std::endl;
									}

									miss_publisher_.publish(msg_missing);
								}


							}
							else if(p_control_state_ == P_SLAM_STATE_Re_Location){

								if (initial_pose_set_){
									initial_pose_set_ = false;
									startEstimate = initial_pose_;
								}
								else{
									startEstimate = slamProcessor->getLastScanMatchPose();
								}

								slamProcessor->update(laserScanContainer_1, laserScanContainer_2, device_num, startEstimate, p_Relocation_, false, p_navigation_);

								slamProcessor->UpdateMap(laserScanContainer_1, laserScanContainer_2, device_num, false, p_navigation_);

								poseInfoContainer_.update(slamProcessor->getLastScanMatchPose(), slamProcessor->getLastScanMatchCovariance(), ros::Time(0), p_map_frame_);

								poseUpdatePublisher_.publish(poseInfoContainer_.getPoseWithCovarianceStamped());

								posePublisher_.publish(poseInfoContainer_.getPoseStamped());


								if(p_pub_odometry_)
								{
									nav_msgs::Odometry tmp;
									tmp.pose = poseInfoContainer_.getPoseWithCovarianceStamped().pose;

									tmp.header = poseInfoContainer_.getPoseWithCovarianceStamped().header;
									tmp.child_frame_id = p_base_frame_;
									odometryPublisher_.publish(tmp);
								}

								if (p_pub_map_odom_transform_)
								{
									tf::StampedTransform odom_to_base;

									try
									{
										tf_.waitForTransform(p_odom_frame_, p_base_frame_, device_scan_1.header.stamp, ros::Duration(0.5));
										tf_.lookupTransform(p_odom_frame_, p_base_frame_, device_scan_1.header.stamp, odom_to_base);
									}
									catch(tf::TransformException e)
									{
										ROS_ERROR("Transform failed during publishing of map_odom transform: %s",e.what());
										odom_to_base.setIdentity();
									}
									map_to_odom_ = tf::Transform(poseInfoContainer_.getTfTransform() * odom_to_base.inverse());
									tfB_->sendTransform( tf::StampedTransform (map_to_odom_, device_scan_1.header.stamp, p_map_frame_, p_odom_frame_));
								}

								if(slamProcessor->ProbMapping >= 0.80){
									ROS_INFO("Finish Position Initial");
									p_control_state_ = P_SLAM_STATE_RUN;
									p_Relocation_ = false;
								}
							}
						}
				}

			}
			else
			{
				scan_data_publisher_.publish(device_scan_1);
				scan_data_publisher_2.publish(device_scan_2);

				if(tf_.waitForTransform(p_base_frame_,device_scan_1.header.frame_id, device_scan_1.header.stamp,dur))
				{

					tf::StampedTransform laserTransform_1;

					tf_.lookupTransform(p_base_frame_,device_scan_1.header.frame_id, device_scan_1.header.stamp, laserTransform_1);



					if (tf_.waitForTransform(p_base_frame_,device_scan_2.header.frame_id, device_scan_2.header.stamp,dur))
					{
						tf::StampedTransform laserTransform_2;

						tf_.lookupTransform(p_base_frame_,device_scan_2.header.frame_id, device_scan_2.header.stamp, laserTransform_2);

						projector_1_.projectLaser(device_scan_1, laser_point_cloud_1_,20.0);

						projector_2_.projectLaser(device_scan_2, laser_point_cloud_2_,20.0);


						Eigen::Vector3f startEstimate(Eigen::Vector3f::Zero());

						if(rosPointCloudToDataContainer(laser_point_cloud_1_, laser_point_cloud_2_,
									laserTransform_1, laserTransform_2,
									laserScanContainer_1, laserScanContainer_2,
									slamProcessor->getScaleToMap(),
									device_num))
						{

							if(p_control_state_ == P_SLAM_STATE_RUN){

								if (initial_pose_set_){
									initial_pose_set_ = false;
									startEstimate = initial_pose_;
								}
								else{
									startEstimate = slamProcessor->getLastScanMatchPose();
								}


								//predict control
								if(isInitialTimeSample && isReceiveOdom && p_navigation_){

									//control input
									float delta_t = float(time_sample - p_time);

									float p_x2 = startEstimate.x() + (odom_vx*cos(startEstimate.z()) - odom_vy*sin(startEstimate.z()))*delta_t;
									float p_y2 = startEstimate.y() + (odom_vx*sin(startEstimate.z()) + odom_vy*cos(startEstimate.z()))*delta_t;
									float yaw2 = startEstimate.z() + odom_w*delta_t;

									NormalizeAngle(yaw2);

									startEstimate[0] = p_x2;
									startEstimate[1] = p_y2;
									startEstimate[2] = yaw2;


								}

								slamProcessor->update(laserScanContainer_1, laserScanContainer_2, device_num, startEstimate, p_Relocation_, false, p_navigation_);

								slamProcessor->UpdateMap(laserScanContainer_1, laserScanContainer_2, device_num, false, p_navigation_);

								poseInfoContainer_.update(slamProcessor->getLastScanMatchPose(), slamProcessor->getLastScanMatchCovariance(), ros::Time(0), p_map_frame_);

								poseUpdatePublisher_.publish(poseInfoContainer_.getPoseWithCovarianceStamped());

								posePublisher_.publish(poseInfoContainer_.getPoseStamped());


								if(p_pub_odometry_)
								{
									nav_msgs::Odometry tmp;
									tmp.pose = poseInfoContainer_.getPoseWithCovarianceStamped().pose;

									tmp.header = poseInfoContainer_.getPoseWithCovarianceStamped().header;
									tmp.child_frame_id = p_base_frame_;
									odometryPublisher_.publish(tmp);
								}

								if (p_pub_map_odom_transform_)
								{
									tf::StampedTransform odom_to_base;

									try
									{
										tf_.waitForTransform(p_odom_frame_, p_base_frame_, device_scan_1.header.stamp, ros::Duration(0.5));
										tf_.lookupTransform(p_odom_frame_, p_base_frame_, device_scan_1.header.stamp, odom_to_base);
									}
									catch(tf::TransformException e)
									{
										ROS_ERROR("Transform failed during publishing of map_odom transform: %s",e.what());
										odom_to_base.setIdentity();
									}
									map_to_odom_ = tf::Transform(poseInfoContainer_.getTfTransform() * odom_to_base.inverse());
									tfB_->sendTransform( tf::StampedTransform (map_to_odom_, device_scan_1.header.stamp, p_map_frame_, p_odom_frame_));
								}


								if(p_navigation_){
									std_msgs::Int16 msg_missing;
									std::cout<<"slamProcessor->ProbMapping"<<slamProcessor->ProbMapping<<std::endl;
									if(slamProcessor->ProbMapping <= 0.35){
										msg_missing.data = 1;
										std::cout<<"missing  "<<std::endl;
									}
									else{
										msg_missing.data = 0;
										std::cout<<"no_missing  "<<std::endl;
									}
									miss_publisher_.publish(msg_missing);
								}


							}
							else if(p_control_state_ == P_SLAM_STATE_Re_Location){

								if (initial_pose_set_){
									initial_pose_set_ = false;
									startEstimate = initial_pose_;
								}
								else{
									startEstimate = slamProcessor->getLastScanMatchPose();
								}

								slamProcessor->update(laserScanContainer_1, laserScanContainer_2, device_num, startEstimate, p_Relocation_, false, p_navigation_);

								slamProcessor->UpdateMap(laserScanContainer_1, laserScanContainer_2, device_num, false, p_navigation_);

								poseInfoContainer_.update(slamProcessor->getLastScanMatchPose(), slamProcessor->getLastScanMatchCovariance(), ros::Time(0), p_map_frame_);

								poseUpdatePublisher_.publish(poseInfoContainer_.getPoseWithCovarianceStamped());

								posePublisher_.publish(poseInfoContainer_.getPoseStamped());


								if(p_pub_odometry_)
								{
									nav_msgs::Odometry tmp;
									tmp.pose = poseInfoContainer_.getPoseWithCovarianceStamped().pose;

									tmp.header = poseInfoContainer_.getPoseWithCovarianceStamped().header;
									tmp.child_frame_id = p_base_frame_;
									odometryPublisher_.publish(tmp);
								}

								if (p_pub_map_odom_transform_)
								{
									tf::StampedTransform odom_to_base;

									try
									{
										tf_.waitForTransform(p_odom_frame_, p_base_frame_, device_scan_1.header.stamp, ros::Duration(0.5));
										tf_.lookupTransform(p_odom_frame_, p_base_frame_, device_scan_1.header.stamp, odom_to_base);
									}
									catch(tf::TransformException e)
									{
										ROS_ERROR("Transform failed during publishing of map_odom transform: %s",e.what());
										odom_to_base.setIdentity();
									}
									map_to_odom_ = tf::Transform(poseInfoContainer_.getTfTransform() * odom_to_base.inverse());
									tfB_->sendTransform( tf::StampedTransform (map_to_odom_, device_scan_1.header.stamp, p_map_frame_, p_odom_frame_));
								}

								if(slamProcessor->ProbMapping >= 0.35){
									ROS_INFO("Finish Position Initial");
									p_control_state_ = P_SLAM_STATE_RUN;
									p_Relocation_ = false;
								}
							}
						}
					}
				}
			}


		}
	}

	if(!isInitialTimeSample){
		isInitialTimeSample = true;
	}

	p_time = time_sample;
}

void HectorMappingRos::sysMsgCallback(const std_msgs::String& string)
{
	ROS_INFO("HectorSM sysMsgCallback, msg contents: %s", string.data.c_str());

	if (string.data == "reset")
	{
		ROS_INFO("HectorSM reset");
		slamProcessor->reset();
	}
}

bool HectorMappingRos::mapCallback(nav_msgs::GetMap::Request  &req,
		nav_msgs::GetMap::Response &res)
{

	//=====Anhung==================================================================//
	if(mapPubContainer.size() > 0){
		ROS_INFO("HectorSM Map service called");
		res = mapPubContainer[0].map_;
		return true;
	}
	return false;
	//=====Anhung==================================================================//

	/*
	   ROS_INFO("HectorSM Map service called");
	   res = mapPubContainer[0].map_;
	   return true;
	 */
}

void HectorMappingRos::publishMap(MapPublisherContainer& mapPublisher, const hectorslam::GridMap& gridMap, ros::Time timestamp, MapLockerInterface* mapMutex)
{
	nav_msgs::GetMap::Response& map_ (mapPublisher.map_);

	//only update map if it changed
	if (lastGetMapUpdateIndex != gridMap.getUpdateIndex() || 1) //kevin
	{

		int sizeX = gridMap.getSizeX();
		int sizeY = gridMap.getSizeY();

		int size = sizeX * sizeY;

		std::vector<int8_t>& data = map_.map.data;

		//std::vector contents are guaranteed to be contiguous, use memset to set all to unknown to save time in loop
		memset(&data[0], -1, sizeof(int8_t) * size);

		if (mapMutex)
		{
			mapMutex->lockMap();
		}

		for(int i=0; i < size; ++i)
		{
			if(gridMap.isFree(i))
			{
				data[i] = 0;
			}
			else if (gridMap.isOccupied(i))
			{
				data[i] = 100;
			}
		}

		lastGetMapUpdateIndex = gridMap.getUpdateIndex();

		if (mapMutex)
		{
			mapMutex->unlockMap();
		}
	}

	map_.map.header.stamp = timestamp;

	mapPublisher.mapPublisher_.publish(map_.map);
}

bool HectorMappingRos::rosLaserScanToDataContainer(const sensor_msgs::LaserScan& scan, hectorslam::DataContainer& dataContainer, float scaleToMap)
{
	size_t size = scan.ranges.size();

	float angle = scan.angle_min;

	dataContainer.clear();

	dataContainer.setOrigo(Eigen::Vector2f::Zero());

	float maxRangeForContainer = scan.range_max - 0.1f;

	for (size_t i = 0; i < size; ++i)
	{
		float dist = scan.ranges[i];

		if ( (dist > scan.range_min) && (dist < maxRangeForContainer))
		{
			dist *= scaleToMap;
			dataContainer.add(Eigen::Vector2f(cos(angle) * dist, sin(angle) * dist));
		}

		angle += scan.angle_increment;
	}

	return true;
}

//bool HectorMappingRos::rosPointCloudToDataContainer(const sensor_msgs::PointCloud& pointCloud, const tf::StampedTransform& laserTransform, hectorslam::DataContainer& dataContainer, float scaleToMap)
bool HectorMappingRos::rosPointCloudToDataContainer(const sensor_msgs::PointCloud& pointCloud_1,
		const sensor_msgs::PointCloud& pointCloud_2,
		const tf::StampedTransform& laserTransform_1,
		const tf::StampedTransform& laserTransform_2,
		hectorslam::DataContainer& dataContainer_1,
		hectorslam::DataContainer& dataContainer_2,
		float scaleToMap,
		int device_num)
{

	size_t laser_size_1 = pointCloud_1.points.size();
	size_t laser_size_2 = pointCloud_2.points.size();


	dataContainer_1.clear();
	dataContainer_2.clear();

	tf::Vector3 laserPos_1 (laserTransform_1.getOrigin());
	dataContainer_1.setOrigo(Eigen::Vector2f(laserPos_1.x(), laserPos_1.y())*scaleToMap);

	tf::Vector3 laserPos_2 (laserTransform_2.getOrigin());
	dataContainer_2.setOrigo(Eigen::Vector2f(laserPos_2.x(), laserPos_2.y())*scaleToMap);


	for (size_t i = 0; i < laser_size_1; ++i)
	{

		const geometry_msgs::Point32& currPoint(pointCloud_1.points[i]);

		float dist_sqr = currPoint.x*currPoint.x + currPoint.y* currPoint.y;

		if ( (dist_sqr > p_sqr_laser_min_dist_) && (dist_sqr < p_sqr_laser_max_dist_) ){

			if ( (currPoint.x < 0.0f) && (dist_sqr < 0.50f)){
				continue;
			}

			tf::Vector3 pointPosBaseFrame(laserTransform_1 * tf::Vector3(currPoint.x, currPoint.y, currPoint.z));

			float pointPosLaserFrameZ = pointPosBaseFrame.z() - laserPos_1.z();

			if (pointPosLaserFrameZ > p_laser_z_min_value_ && pointPosLaserFrameZ < p_laser_z_max_value_)
			{
				dataContainer_1.add(Eigen::Vector2f(pointPosBaseFrame.x(),pointPosBaseFrame.y())*scaleToMap);
			}
		}
	}

	if(device_num == 2){

		for (size_t i = 0; i < laser_size_2; ++i)
		{

			const geometry_msgs::Point32& currPoint(pointCloud_2.points[i]);

			float dist_sqr = currPoint.x*currPoint.x + currPoint.y* currPoint.y;

			if ( (dist_sqr > p_sqr_laser_min_dist_) && (dist_sqr < p_sqr_laser_max_dist_) ){

				if ( (currPoint.x < 0.0f) && (dist_sqr < 0.50f)){
					continue;
				}

				tf::Vector3 pointPosBaseFrame(laserTransform_2 * tf::Vector3(currPoint.x, currPoint.y, currPoint.z));

				float pointPosLaserFrameZ = pointPosBaseFrame.z() - laserPos_2.z();

				if (pointPosLaserFrameZ > p_laser_z_min_value_ && pointPosLaserFrameZ < p_laser_z_max_value_)
				{
					dataContainer_2.add(Eigen::Vector2f(pointPosBaseFrame.x(),pointPosBaseFrame.y())*scaleToMap);
				}
			}
		}

	}


	return true;
}

void HectorMappingRos::setServiceGetMapData(nav_msgs::GetMap::Response& map_, const hectorslam::GridMap& gridMap)
{
	Eigen::Vector2f mapOrigin (gridMap.getWorldCoords(Eigen::Vector2f::Zero()));
	mapOrigin.array() -= gridMap.getCellLength()*0.5f;

	map_.map.info.origin.position.x = mapOrigin.x();
	map_.map.info.origin.position.y = mapOrigin.y();
	map_.map.info.origin.orientation.w = 1.0;

	map_.map.info.resolution = gridMap.getCellLength();

	map_.map.info.width = gridMap.getSizeX();
	map_.map.info.height = gridMap.getSizeY();

	map_.map.header.frame_id = p_map_frame_;
	map_.map.data.resize(map_.map.info.width * map_.map.info.height);
}



void HectorMappingRos::publishMapLoop(double map_pub_period)
{

	//=====Anhung==================================================================//
	ros::Rate r(1.0 / map_pub_period);

	while(ros::ok())
	{
		// std::cout << "publishMapLoop" << std::endl; //kevin debug
		if(mapPubContainer.size() > 0){

			ros::Time mapTime (ros::Time::now());

			publishMap(mapPubContainer[0],slamProcessor->getGridMap(0), mapTime, slamProcessor->getMapMutex(0));
			
			// std::cout << "publishMapLoop if" << std::endl;
		}

		r.sleep();

	}

}



void HectorMappingRos::commandCallback(const std_msgs::String& command)
{

	bool determine = false;

	if(command.data == protect_map)
	{
		std::cout<<"stop"<<std::endl;
		determine = true;

		std_msgs::Int8 msg;
		msg.data = 8;
		Error_Publisher_.publish(msg);
	}

	if(!determine)
	{
		if(command.data == "Create Map"){

			ROS_INFO("Start Create Map");
			p_navigation_ = false;
			CreateMap();
			p_control_state_ = P_SLAM_STATE_RUN;

		}
		else if(command.data == "Save Map"){

			ROS_INFO("Save Map");
			SaveMap();
			p_control_state_ = P_SLAM_STATE_RUN;

		}
		else if(command.data == "Load Map"){

			ROS_INFO("Start Load Map");
			LoadMap();
			p_control_state_ = P_SLAM_STATE_Idle;
			p_navigation_ = true;

		}
		else if(command.data == "ReLoad Map"){

			// p_navigation_ = false;
			// p_control_state_ = P_SLAM_STATE_RUN;
			// HectorMappingRos::KevinTest();

			ROS_INFO("Start ReLoad Map");
			ReLoadMap();
			p_control_state_ = P_SLAM_STATE_Idle;
			p_navigation_ = true;
			
		}
		else if(command.data == "navigation=true"){

			ROS_INFO("Start Navigation");
			p_navigation_ = true;
			p_control_state_ = P_SLAM_STATE_RUN;

		}
		else if(command.data == "navigation=false"){

			ROS_INFO("End Navigation");
			p_navigation_ = false;
			p_control_state_ = P_SLAM_STATE_RUN;


		}
		// else if(command.data == "Relocation"){

		// 	ROS_INFO("Start Re-Location");
		// 	p_navigation_ = true;
		// 	p_Relocation_ = true;
		// 	p_control_state_ = P_SLAM_STATE_Re_Location;

		// }
	}

	protect_map = command.data; //kevin


}



void HectorMappingRos::staticMapCallback(const nav_msgs::OccupancyGrid& map)
{

}

void HectorMappingRos::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{

	initial_pose_set_ = true;
	p_Relocation_ = true;
	tf::Pose pose;
	tf::poseMsgToTF(msg->pose.pose, pose);
	initial_pose_ = Eigen::Vector3f(msg->pose.pose.position.x, msg->pose.pose.position.y, tf::getYaw(pose.getRotation()));
	ROS_INFO("Setting initial pose with world coords x: %f y: %f yaw: %f", initial_pose_[0], initial_pose_[1], initial_pose_[2]);
	ROS_INFO("Start Position Initial");
	p_control_state_ = P_SLAM_STATE_Re_Location;

}


void HectorMappingRos::goalPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{

	tf::Pose pose;
	tf::poseMsgToTF(msg->pose, pose);
	Eigen::Vector3f goal_pose_ = Eigen::Vector3f(msg->pose.position.x, msg->pose.position.y, tf::getYaw(pose.getRotation()));


	geometry_msgs::PoseStamped msg_goal_pose;

	msg_goal_pose.pose.position.x = goal_pose_.x();
	msg_goal_pose.pose.position.y = goal_pose_.y();
	msg_goal_pose.pose.orientation.w = cos(goal_pose_.z()*0.5f);
	msg_goal_pose.pose.orientation.z = sin(goal_pose_.z()*0.5f);

	goal_publisher_.publish(msg_goal_pose);

	ROS_INFO("Setting Goal pose with world coords x: %f y: %f yaw: %f", goal_pose_[0], goal_pose_[1], goal_pose_[2]);




}

int HectorMappingRos::socket_recvtimeout(int ss, char *buf, int len, int timeout)
{
	fd_set fds;
	int n;
	struct timeval tv;

	FD_ZERO(&fds);
	FD_SET(ss, &fds);

	tv.tv_sec = 0;
	tv.tv_usec = timeout;

	n = select(ss+1, &fds, NULL, NULL, &tv);
	if(n == 0) return -2; //timeout;
	if(n == -1) return -1; //error

	return recv(ss, buf, len, 0);
}

void HectorMappingRos::diep(char *s)
{
	perror(s);
	exit(1);
}


/*=====Anhung==========================Create Map ==============================*/
void HectorMappingRos::CreateMap()
{
	slamProcessor = new hectorslam::HectorSlamProcessor(static_cast<float>(p_map_resolution_), p_map_size_, p_map_size_, Eigen::Vector2f(p_map_start_x_, p_map_start_y_), p_map_multi_res_levels_, hectorDrawings, debugInfoProvider);
	slamProcessor->setUpdateFactorFree(p_update_factor_free_);
	slamProcessor->setUpdateFactorOccupied(p_update_factor_occupied_);
	slamProcessor->setMapUpdateMinDistDiff(p_map_update_distance_threshold_);
	slamProcessor->setMapUpdateMinAngleDiff(p_map_update_angle_threshold_);

	int mapLevels = slamProcessor->getMapLevels();
	mapLevels = 1;

	std::string mapTopic_ = "map";

	for (int i = 0; i < mapLevels; ++i)
	{
		mapPubContainer.push_back(MapPublisherContainer());
		slamProcessor->addMapMutex(i, new HectorMapMutex());

		std::string mapTopicStr(mapTopic_);

		if (i != 0)
		{
			mapTopicStr.append("_" + boost::lexical_cast<std::string>(i));
		}

		std::string mapMetaTopicStr(mapTopicStr);
		mapMetaTopicStr.append("_metadata");

		MapPublisherContainer& tmp = mapPubContainer[i];
		tmp.mapPublisher_ = node_.advertise<nav_msgs::OccupancyGrid>(mapTopicStr, 1, true);
		tmp.mapMetadataPublisher_ = node_.advertise<nav_msgs::MapMetaData>(mapMetaTopicStr, 1, true);

		if ( (i == 0) && p_advertise_map_service_)
		{
			tmp.dynamicMapServiceServer_ = node_.advertiseService("dynamic_map", &HectorMappingRos::mapCallback, this);
		}

		setServiceGetMapData(tmp.map_, slamProcessor->getGridMap(i));

		if ( i== 0){
			mapPubContainer[i].mapMetadataPublisher_.publish(mapPubContainer[i].map_.map.info);
		}
	}
}

/*=====Anhung===========================Load Map ===============================*/
void HectorMappingRos::LoadMap() //kevin
{
	std::cout << "############### LoadMap ####################" << std::endl;
	std::cout << "test0" << std::endl;
	slamProcessor = NULL;
	// slamProcessor = new hectorslam::HectorSlamProcessor(static_cast<float>(p_map_resolution_), p_map_size_, p_map_size_, Eigen::Vector2f(p_map_start_x_, p_map_start_y_), p_map_multi_res_levels_, hectorDrawings, debugInfoProvider);
	// slamProcessor->setUpdateFactorFree(p_update_factor_free_);
	// slamProcessor->setUpdateFactorOccupied(p_update_factor_occupied_);
	// slamProcessor->setMapUpdateMinDistDiff(p_map_update_distance_threshold_);
	// slamProcessor->setMapUpdateMinAngleDiff(p_map_update_angle_threshold_);

	std::cout << "test1" << std::endl;

	if(slamProcessor == NULL){
		std::cout << "test2" << std::endl;

		char c_yaml_path[1000];

		
		//yaml open
		std::string s_file_name = P_MAP_FILE_NAME;
		
		std::string s_yaml_path = TitlePath + P_MAP_STORE_PATH;
		
		s_yaml_path = s_yaml_path + s_file_name + ".yaml";
		
		std::strcpy(c_yaml_path, s_yaml_path.c_str());
		
		std::fstream fin;
		
		fin.open(c_yaml_path, std::fstream::in);

		if(!fin.is_open())
		{

			std::cout << "Error: the "<<s_yaml_path<<"is not opened!!" << std::endl;

		}
		else{

			std::string s_number_of_map;

			getline(fin,s_number_of_map);

			std::string s_number_of_map_value;

			getline(fin,s_number_of_map_value);

			int number_of_map = std::atoi(s_number_of_map_value.c_str());

			s_number_of_map = s_number_of_map + " " + s_number_of_map_value;

			std::cout<<s_number_of_map<<std::endl;

			std::cout<<"===============Existed Map==============="<<std::endl;

			for(int i=0; i<number_of_map; i++){

				std::string s_image;

				std::getline(fin,s_image);

				std::string s_image_path;

				std::getline(fin,s_image_path);

				s_image_path = TitlePath + s_image_path;

				s_image = s_image + " " + s_image_path;

				std::cout<<s_image<<std::endl;

				std::string s_resolution;

				std::getline(fin,s_resolution);

				std::string s_resolution_value;

				std::getline(fin,s_resolution_value);

				float resolution_value = std::atof(s_resolution_value.c_str());

				std::cout<<s_resolution<<" "<<resolution_value<<std::endl;

				std::string s_origin;

				std::getline(fin,s_origin);

				std::string s_origin_x;

				std::getline(fin,s_origin_x);

				std::string s_origin_y;

				std::getline(fin,s_origin_y);

				std::string s_origin_t;

				std::getline(fin,s_origin_t);

				s_origin = s_origin + " (" + s_origin_x + " , " + s_origin_y + " , " + s_origin_t + ")";

				std::cout<<s_origin<<std::endl;

				std::string s_negate;

				std::getline(fin,s_negate);

				std::string s_negate_value;

				std::getline(fin,s_negate_value);

				int negate_value = std::atoi(s_negate_value.c_str());

				std::cout<<s_negate<<" "<<negate_value<<std::endl;

				std::string s_occupied_thresh;

				std::getline(fin,s_occupied_thresh);

				std::string s_occupied_thresh_value;

				std::getline(fin,s_occupied_thresh_value);

				float occupied_thresh_value = std::atof(s_occupied_thresh_value.c_str());

				std::cout<<s_occupied_thresh<<" "<<occupied_thresh_value<<std::endl;

				std::string s_free_thresh;

				std::getline(fin,s_free_thresh);

				std::string s_free_thresh_value;

				std::getline(fin,s_free_thresh_value);

				float free_thresh_value = std::atof(s_free_thresh_value.c_str());

				std::cout<<s_free_thresh<<" "<<free_thresh_value<<std::endl;

				//======Create Map========/

				//Load Image.
				SDL_Surface* img;

				// Load the image using SDL.  If we get NULL back, the image load failed.mapTopic_
				if(!(img = IMG_Load(s_image_path.c_str())))
				{
					std::string errmsg = std::string("failed to open image file \"") +
						s_image;
					throw std::runtime_error(errmsg);
				}

				std::cout<<"Map Size: "<<img->w<<" , "<<img->h<<std::endl;

				//First we can Create or Map space.
				if(i == 0){

					p_map_resolution_ = resolution_value;

					slamProcessor = new hectorslam::HectorSlamProcessor(static_cast<float>(resolution_value), int(img->w), int(img->h), Eigen::Vector2f(0.5, 0.5), number_of_map, hectorDrawings, debugInfoProvider);
					slamProcessor->setUpdateFactorFree(p_update_factor_free_);
					slamProcessor->setUpdateFactorOccupied(p_update_factor_occupied_);
					slamProcessor->setMapUpdateMinDistDiff(p_map_update_distance_threshold_);
					slamProcessor->setMapUpdateMinAngleDiff(p_map_update_angle_threshold_);

					int mapLevels = slamProcessor->getMapLevels();
					mapLevels = 1;

					std::string mapTopic_ = "map";

					for (int i = 0; i < mapLevels; ++i)
					{
						mapPubContainer.push_back(MapPublisherContainer());
						slamProcessor->addMapMutex(i, new HectorMapMutex());

						std::string mapTopicStr(mapTopic_);

						if (i != 0)
						{
							mapTopicStr.append("_" + boost::lexical_cast<std::string>(i));
						}

						std::string mapMetaTopicStr(mapTopicStr);
						mapMetaTopicStr.append("_metadata");

						MapPublisherContainer& tmp = mapPubContainer[i];
						tmp.mapPublisher_ = node_.advertise<nav_msgs::OccupancyGrid>(mapTopicStr, 1, true);
						tmp.mapMetadataPublisher_ = node_.advertise<nav_msgs::MapMetaData>(mapMetaTopicStr, 1, true);

						if ( (i == 0) && p_advertise_map_service_)
						{
							tmp.dynamicMapServiceServer_ = node_.advertiseService("dynamic_map", &HectorMappingRos::mapCallback, this);
						}

						setServiceGetMapData(tmp.map_, slamProcessor->getGridMap(i));

						if ( i== 0){
							mapPubContainer[i].mapMetadataPublisher_.publish(mapPubContainer[i].map_.map.info);
						}
					}

				}

				//When we have the memory of map, we can initialize it.
				unsigned char* pixels;
				unsigned char* p;
				unsigned char value;
				int rowstride, n_channels, avg_channels;
				int color_sum;
				float color_avg;


				rowstride = img->pitch;

				n_channels = img->format->BytesPerPixel;

				std::cout<<"n_channels: "<<n_channels<<std::endl;


				//Copy pixel data into the map structure.
				pixels = (unsigned char*)(img->pixels);
				for(int l=0; l < img->h; l++){

					for(int k=0; k< img->w; k++){

						p = pixels + l*rowstride + k*n_channels;

						color_sum = 0;

						for(int m=0; m < n_channels; m++)
							color_sum += *(p + (m));
						color_avg = color_sum / (float)n_channels;

						if(color_avg < 50) color_avg = 0;
						else if(color_avg < 230) color_avg = 205;
						else color_avg = 254;

						float occ = float(255 - color_avg) / 255.0;

						int index = (img->h - l - 1) * (img->w) + k;

						float value = 0;

						if(occ > occupied_thresh_value){
							value = 80.0;
							slamProcessor->setMapValue(i, index, value);
						}
						else if(occ < free_thresh_value){
							value =-1;
							slamProcessor->setMapValue(i, index, value);
						}
						else{
							value = 0;
							slamProcessor->setMapValue(i, index, value);
						}


					}

				}


				SDL_FreeSurface(img);
				img = NULL;

			}
		}

		fin.close();

	}
}

/*=====Anhung===========================reLoad Map ===============================*/
void HectorMappingRos::ReLoadMap() //kevin
{
	std::cout << "############### ReLoadMap ####################" << std::endl;
	std::cout << "test0" << std::endl;
	slamProcessor = NULL;
	// slamProcessor = new hectorslam::HectorSlamProcessor(static_cast<float>(p_map_resolution_), p_map_size_, p_map_size_, Eigen::Vector2f(p_map_start_x_, p_map_start_y_), p_map_multi_res_levels_, hectorDrawings, debugInfoProvider);
	// slamProcessor->setUpdateFactorFree(p_update_factor_free_);
	// slamProcessor->setUpdateFactorOccupied(p_update_factor_occupied_);
	// slamProcessor->setMapUpdateMinDistDiff(p_map_update_distance_threshold_);
	// slamProcessor->setMapUpdateMinAngleDiff(p_map_update_angle_threshold_);

	std::cout << "test1" << std::endl;

	if(slamProcessor == NULL){
		std::cout << "test2" << std::endl;

		char c_yaml_path[1000];

		
		//yaml open
		std::string s_file_name = P_MAP_FILE_NAME;
		
		std::string s_yaml_path = TitlePath + P_MAP_STORE_PATH;
		
		s_yaml_path = s_yaml_path + s_file_name + ".yaml";
		
		std::strcpy(c_yaml_path, s_yaml_path.c_str());
		
		std::fstream fin;
		
		fin.open(c_yaml_path, std::fstream::in);

		if(!fin.is_open())
		{

			std::cout << "Error: the "<<s_yaml_path<<"is not opened!!" << std::endl;

		}
		else{

			std::string s_number_of_map;

			getline(fin,s_number_of_map);

			std::string s_number_of_map_value;

			getline(fin,s_number_of_map_value);

			int number_of_map = std::atoi(s_number_of_map_value.c_str());

			s_number_of_map = s_number_of_map + " " + s_number_of_map_value;

			std::cout<<s_number_of_map<<std::endl;

			std::cout<<"===============Existed Map==============="<<std::endl;

			for(int i=0; i<number_of_map; i++){

				std::string s_image;

				std::getline(fin,s_image);

				std::string s_image_path;

				std::getline(fin,s_image_path);

				s_image_path = "/ros_map/myMap_10.pgm"; //kevin

				s_image_path = TitlePath + s_image_path;

				s_image = s_image + " " + s_image_path;

				std::cout<<s_image<<std::endl;

				std::string s_resolution;

				std::getline(fin,s_resolution);

				std::string s_resolution_value;

				std::getline(fin,s_resolution_value);

				float resolution_value = std::atof(s_resolution_value.c_str());

				std::cout<<s_resolution<<" "<<resolution_value<<std::endl;

				std::string s_origin;

				std::getline(fin,s_origin);

				std::string s_origin_x;

				std::getline(fin,s_origin_x);

				std::string s_origin_y;

				std::getline(fin,s_origin_y);

				std::string s_origin_t;

				std::getline(fin,s_origin_t);

				s_origin = s_origin + " (" + s_origin_x + " , " + s_origin_y + " , " + s_origin_t + ")";

				std::cout<<s_origin<<std::endl;

				std::string s_negate;

				std::getline(fin,s_negate);

				std::string s_negate_value;

				std::getline(fin,s_negate_value);

				int negate_value = std::atoi(s_negate_value.c_str());

				std::cout<<s_negate<<" "<<negate_value<<std::endl;

				std::string s_occupied_thresh;

				std::getline(fin,s_occupied_thresh);

				std::string s_occupied_thresh_value;

				std::getline(fin,s_occupied_thresh_value);

				float occupied_thresh_value = std::atof(s_occupied_thresh_value.c_str());

				std::cout<<s_occupied_thresh<<" "<<occupied_thresh_value<<std::endl;

				std::string s_free_thresh;

				std::getline(fin,s_free_thresh);

				std::string s_free_thresh_value;

				std::getline(fin,s_free_thresh_value);

				float free_thresh_value = std::atof(s_free_thresh_value.c_str());

				std::cout<<s_free_thresh<<" "<<free_thresh_value<<std::endl;

				//======Create Map========/

				//Load Image.
				SDL_Surface* img;

				// Load the image using SDL.  If we get NULL back, the image load failed.mapTopic_
				if(!(img = IMG_Load(s_image_path.c_str())))
				{
					std::string errmsg = std::string("failed to open image file \"") +
						s_image;
					throw std::runtime_error(errmsg);
				}

				std::cout<<"Map Size: "<<img->w<<" , "<<img->h<<std::endl;

				//First we can Create or Map space.
				if(i == 0){

					p_map_resolution_ = resolution_value;

					slamProcessor = new hectorslam::HectorSlamProcessor(static_cast<float>(resolution_value), int(img->w), int(img->h), Eigen::Vector2f(0.5, 0.5), number_of_map, hectorDrawings, debugInfoProvider);
					// slamProcessor = new hectorslam::HectorSlamProcessor(static_cast<float>(p_map_resolution_), p_map_size_, p_map_size_, Eigen::Vector2f(p_map_start_x_, p_map_start_y_), p_map_multi_res_levels_, hectorDrawings, debugInfoProvider); //kevin
					slamProcessor->setUpdateFactorFree(p_update_factor_free_);
					slamProcessor->setUpdateFactorOccupied(p_update_factor_occupied_);
					slamProcessor->setMapUpdateMinDistDiff(p_map_update_distance_threshold_);
					slamProcessor->setMapUpdateMinAngleDiff(p_map_update_angle_threshold_);

					int mapLevels = slamProcessor->getMapLevels();
					mapLevels = 1;

					// std::string mapTopic_ = "map";

					// for (int i = 0; i < mapLevels; ++i)
					// {
					// 	std::cout << "test map" << std::endl;
					// 	mapPubContainer.push_back(MapPublisherContainer());
					// 	slamProcessor->addMapMutex(i, new HectorMapMutex());

					// 	std::string mapTopicStr(mapTopic_);

					// 	if (i != 0)
					// 	{
					// 		mapTopicStr.append("_" + boost::lexical_cast<std::string>(i));
					// 	}

					// 	std::string mapMetaTopicStr(mapTopicStr);
					// 	mapMetaTopicStr.append("_metadata");

					// 	MapPublisherContainer& tmp = mapPubContainer[i];
					// 	tmp.mapPublisher_ = node_.advertise<nav_msgs::OccupancyGrid>(mapTopicStr, 1, true);
					// 	tmp.mapMetadataPublisher_ = node_.advertise<nav_msgs::MapMetaData>(mapMetaTopicStr, 1, true);
					// 	std::cout << "test map tmp" << std::endl;

					// 	if ( (i == 0) && p_advertise_map_service_)
					// 	{
					// 		tmp.dynamicMapServiceServer_ = node_.advertiseService("dynamic_map", &HectorMappingRos::mapCallback, this);
					// 	}

					// 	setServiceGetMapData(tmp.map_, slamProcessor->getGridMap(i));

					// 	if ( i== 0){
					// 		mapPubContainer[i].mapMetadataPublisher_.publish(mapPubContainer[i].map_.map.info);
					// 	}
					// }

				}

				//When we have the memory of map, we can initialize it.
				unsigned char* pixels;
				unsigned char* p;
				unsigned char value;
				int rowstride, n_channels, avg_channels;
				int color_sum;
				float color_avg;


				rowstride = img->pitch;

				n_channels = img->format->BytesPerPixel;

				std::cout<<"n_channels: "<<n_channels<<std::endl;


				//Copy pixel data into the map structure.
				pixels = (unsigned char*)(img->pixels);
				for(int l=0; l < img->h; l++){

					for(int k=0; k< img->w; k++){

						p = pixels + l*rowstride + k*n_channels;

						color_sum = 0;

						for(int m=0; m < n_channels; m++)
							color_sum += *(p + (m));
						color_avg = color_sum / (float)n_channels;

						if(color_avg < 50) color_avg = 0;
						else if(color_avg < 230) color_avg = 205;
						else color_avg = 254;

						float occ = float(255 - color_avg) / 255.0;

						int index = (img->h - l - 1) * (img->w) + k;

						float value = 0;

						if(occ > occupied_thresh_value){
							value = 80.0;
							slamProcessor->setMapValue(i, index, value);
						}
						else if(occ < free_thresh_value){
							value =-1;
							slamProcessor->setMapValue(i, index, value);
						}
						else{
							value = 0;
							slamProcessor->setMapValue(i, index, value);
						}


					}

				}


				SDL_FreeSurface(img);
				img = NULL;

			}
		}

		fin.close();

	}
}


/*=====Anhung===========================Save Map ===============================*/
void HectorMappingRos::SaveMap()
{
	if(slamProcessor != NULL){


		char c_yaml_path[1000];

		std::fstream fp;

		//yaml open
		std::string s_file_name = P_MAP_FILE_NAME;

		std::string s_yaml_path =  TitlePath + P_MAP_STORE_PATH;

		s_yaml_path = s_yaml_path + s_file_name + ".yaml";

		std::strcpy(c_yaml_path, s_yaml_path.c_str());

		fp.open(c_yaml_path, std::ios::out);


		//write data

		fp <<"Number of Map:\n";

		fp << slamProcessor->p_multi_res_size_ <<"\n";

		float occupied_thresh = 0.65;

		float free_thresh = 0.196;

		float map_resolution = slamProcessor->p_mapResolution_;

		int map_width = slamProcessor->p_mapSizeX_;

		int map_height = slamProcessor->p_mapSizeY_;



		std::string s_origin_x = float2str(0);

		std::string s_origin_y = float2str(0);

		std::string s_origin_t = float2str(0);

		std::string s_negate = int2str(0);

		std::string s_occupied_thresh = float2str(occupied_thresh);

		std::string s_free_thresh = float2str(free_thresh);

		for(int i=0; i<slamProcessor->p_multi_res_size_; i++){

			//Save .pgm file
			std::string s_image_name = P_MAP_IMAGE_NAME;

			std::string s_image_path = TitlePath + P_MAP_STORE_PATH;

			std::string s_number = int2str(i);

			s_image_path = s_image_path + s_image_name + s_number + ".pgm";

			std::string local_image_path = P_MAP_STORE_PATH + s_image_name + s_number + ".pgm";

			std::cout<<"Writing map occupancy data to "<<s_image_path<<std::endl;

			FILE* out = fopen(s_image_path.c_str(), "w");
			if (!out)
			{
				std::cout<<"Couldn't save map file to "<<s_image_path<<std::endl;
				return;
			}

			std::fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
					map_resolution, map_width, map_height);

			for(int y=0; y < map_height; y++){

				for(int x=0; x < map_width; x++){

					int index = x + (map_height - y - 1) * map_width;

					float map_prob = slamProcessor->getMapProb(i, index);

					if(map_prob < free_thresh)
						std::fputc(254, out);
					else if(map_prob >= occupied_thresh)
						std::fputc(000, out);
					else
						std::fputc(205, out);

				}
			}

			std::fclose(out);

			fp <<"image:\n";

			fp<<local_image_path<<"\n";

			std::string s_resolution = float2str(map_resolution);

			fp <<"resolution:\n";

			fp<<s_resolution<<"\n";

			fp <<"origin:\n";

			fp<<s_origin_x<<"\n";

			fp<<s_origin_y<<"\n";

			fp<<s_origin_t<<"\n";

			fp <<"negate:\n";

			fp<<s_negate<<"\n";

			fp <<"occupied_thresh:\n";

			fp<<s_occupied_thresh<<"\n";

			fp <<"free_thresh:\n";

			fp<<s_free_thresh<<"\n";


			map_resolution *= 2;

			map_width /= 2;

			map_height /= 2;

		}
		std::cout<<"=============Finish Saved!!!============="<<std::endl;
		fp.close();
	}
}

/*=====Anhung=========================int to string ============================*/
std::string HectorMappingRos::int2str(int i)
{
	std::string s;
	std::stringstream ss(s);
	ss << i;

	return ss.str();
}

/*=====Anhung=======================float to string ============================*/
std::string HectorMappingRos::float2str(float i)
{
	std::string s;
	std::stringstream ss(s);
	ss << i;

	return ss.str();
}

/*=====Anhung=======================check how many lasers ======================*/
int HectorMappingRos::checkDeviceNum(const sensor_msgs::LaserScan& scan)
{

	int range_size = scan.ranges.size();

	float angle_min = (scan.angle_min)*180.0/ M_PI;

	float angle_max = (scan.angle_max)*180.0/ M_PI;

	float angle_increment = scan.angle_increment*180.0/ M_PI;

	float angle_range = fabs(angle_max - angle_min);

	float checksize = angle_range/angle_increment;



	if((range_size/checksize) > 1.5){
		return 2;
	}
	else{
		return 1;
	}

}

//=====Anhung=================Subscribe Relocation pose======================//
void HectorMappingRos::RePoseCallback(const geometry_msgs::PoseStamped& msg)
{

	tf::Pose pose_tf;
	tf::poseMsgToTF(msg.pose, pose_tf);
	initial_pose_ = Eigen::Vector3f(msg.pose.position.x, msg.pose.position.y, tf::getYaw(pose_tf.getRotation()));
	ROS_INFO("Setting Re initial pose with world coords x: %f y: %f yaw: %f", initial_pose_[0], initial_pose_[1], initial_pose_[2]);


	ROS_INFO("Start Re-Location");
	p_navigation_ = true;
	p_Relocation_ = true;
	initial_pose_set_ = true;
	p_control_state_ = P_SLAM_STATE_RUN;
}



//=====Anhung===================Receive odometry speed ======================//
void HectorMappingRos::OdomSpeedCallback(const geometry_msgs::PoseStamped& msg)
{
	odom_vx = msg.pose.position.x;
	odom_vy = msg.pose.position.y;
	odom_w = msg.pose.orientation.w;

	if(!isReceiveOdom)
		isReceiveOdom = true;
}


void HectorMappingRos::NormalizeAngle(float& phi)
{
	phi = atan2(sin(phi), cos(phi));
}
void HectorMappingRos::LoadTitlePath()
{
	std::string NOWPath = ros::package::getPath("hector_mapping");

	std::string PATH_par;
	std::string recv_pkg[100];

	int count=0;
	std::stringstream cut(NOWPath);
	while(getline(cut,PATH_par,'/'))
	{
		recv_pkg[count]=PATH_par;
		std::cout<<" recv_pkg= "<<count << " "<< recv_pkg[count]<<std::endl;
		count++;
	}

	TitlePath = "/" + recv_pkg[1] + "/" + recv_pkg[2] + "/" + recv_pkg[3] + "/" + recv_pkg[4];
	std::cout<<"TitlePath  " <<TitlePath <<std::endl;

}

void HectorMappingRos::KevinTest()
{


}
