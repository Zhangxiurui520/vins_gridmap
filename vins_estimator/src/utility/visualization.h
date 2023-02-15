#pragma once

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include "CameraPoseVisualization.h"
#include <eigen3/Eigen/Dense>
#include "../estimator.h"
#include "../parameters.h"
#include <fstream>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/OccupancyGrid.h"
extern ros::Publisher pub_odometry;
extern ros::Publisher pub_path, pub_pose;
extern ros::Publisher pub_cloud, pub_map;
extern ros::Publisher pub_key_poses;
extern ros::Publisher pub_ref_pose, pub_cur_pose;
extern ros::Publisher pub_key;
extern nav_msgs::Path path;
extern ros::Publisher pub_pose_graph;
extern int IMAGE_ROW, IMAGE_COL;

// extern ros::Publisher pub_kf_mp_sub;
extern nav_msgs::OccupancyGrid grid_map_msg;
extern ros::Publisher pub_grid_map;

extern float kf_pos_x;
extern float kf_pos_z;
extern int kf_pos_grid_x;
extern int kf_pos_grid_z;
extern int h;
extern int w;
extern unsigned int n_kf_received;

extern float grid_max_x;
extern float grid_min_x;
extern float grid_max_z;
extern float grid_min_z;

extern float norm_factor_x;
extern float norm_factor_z;

extern double grid_res_x;
extern double grid_res_z;
extern cv::Mat global_occupied_counter;
extern cv::Mat global_visit_counter;
extern cv::Mat local_occupied_counter;
extern cv::Mat local_visit_counter;
extern cv::Mat local_map_pt_mask;
extern cv::Mat grid_map, grid_map_int, grid_map_thresh, grid_map_thresh_resized;



void registerPub(ros::NodeHandle &n);

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, const std_msgs::Header &header);

void printStatistics(const Estimator &estimator, double t);

void pubOdometry(const Estimator &estimator, const std_msgs::Header &header);

void pubInitialGuess(const Estimator &estimator, const std_msgs::Header &header);

void pubKeyPoses(const Estimator &estimator, const std_msgs::Header &header);

void pubCameraPose(const Estimator &estimator, const std_msgs::Header &header);

void pubPointCloud(const Estimator &estimator, const std_msgs::Header &header);

void pubTF(const Estimator &estimator, const std_msgs::Header &header);

void pubKeyframe(const Estimator &estimator);

void pubRelocalization(const Estimator &estimator);

/**        to map *****/
void ptCallback(const geometry_msgs::PoseArray & pts_and_pose);

void updateGridMap(const geometry_msgs::PoseArray & pts_and_pose);

void processMapPts(const std::vector<geometry_msgs::Pose> &pts, unsigned int n_pts,
	unsigned int start_id, int kf_pos_grid_x, int kf_pos_grid_z);

void processMapPt(const geometry_msgs::Point &curr_pt, cv::Mat &occupied,
	cv::Mat &visited, cv::Mat &pt_mask, int kf_pos_grid_x, int kf_pos_grid_z);

void getGridMap();

void pubKeyframeMap(const Estimator &estimator);



