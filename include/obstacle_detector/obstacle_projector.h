/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Poznan University of Technology nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#pragma once

#include <list>
#include <string>
#include <ros/ros.h>
#include <armadillo>
#include <std_srvs/Empty.h>
#include <obstacle_detector/Obstacles.h>

#include "obstacle_detector/utilities/tracked_obstacle.h"
#include "obstacle_detector/utilities/math_utilities.h"

// Added by ROAHMLab
#include <obstacle_detector/Observation.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

// OpenCV includes
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace obstacle_detector
{

class ObstacleProjector {
public:
  ObstacleProjector(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
  ~ObstacleProjector();

private:
  bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void timerCallback(const ros::TimerEvent&);
  void obsPcdCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pred_pcd_ptr);
  void imageCallback(const sensor_msgs::ImageConstPtr& img_ptr);

  void initialize() { std_srvs::Empty empt; updateParams(empt.request, empt.response); }

  // double obstacleCostFunction(const CircleObstacle& new_obstacle, const CircleObstacle& old_obstacle);
  // void calculateCostMatrix(const std::vector<CircleObstacle>& new_obstacles, arma::mat& cost_matrix);
  // void calculateRowMinIndices(const arma::mat& cost_matrix, std::vector<int>& row_min_indices);
  // void calculateColMinIndices(const arma::mat& cost_matrix, std::vector<int>& col_min_indices);

  // bool fusionObstacleUsed(const int idx, const std::vector<int>& col_min_indices, const std::vector<int>& used_new, const std::vector<int>& used_old);
  // bool fusionObstaclesCorrespond(const int idx, const int jdx, const std::vector<int>& col_min_indices, const std::vector<int>& used_old);
  // bool fissionObstacleUsed(const int idx, const int T, const std::vector<int>& row_min_indices, const std::vector<int>& used_new, const std::vector<int>& used_old);
  // bool fissionObstaclesCorrespond(const int idx, const int jdx, const std::vector<int>& row_min_indices, const std::vector<int>& used_new);

  // void fuseObstacles(const std::vector<int>& fusion_indices, const std::vector<int>& col_min_indices,
  //                    std::vector<TrackedObstacle>& new_tracked, const Obstacles::ConstPtr& new_obstacles);
  // void fissureObstacle(const std::vector<int>& fission_indices, const std::vector<int>& row_min_indices,
  //                      std::vector<TrackedObstacle>& new_tracked, const Obstacles::ConstPtr& new_obstacles);

  // void updateObstacles();
  void publishPredictImage();

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::Subscriber obs_pcd_sub_;  // A subcriber for prediction pointclouds
  ros::Subscriber image_sub_;    // A subcriber for camera images


  image_transport::Publisher img_projected_pub_;

  ros::ServiceServer params_srv_;
  ros::Timer timer_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr now_pcd_ptr;

  // define transformation listener
  tf::TransformListener tf_listener;
  tf::StampedTransform transform;

  cv::Mat now_img;
  // Parameters
  bool p_active_;
  bool p_copy_segments_;

  double p_tracking_duration_;
  double p_loop_rate_;
  double p_sampling_time_;
  double p_sensor_rate_;
  double p_min_correspondence_cost_;
  double p_std_correspondence_dev_;
  double p_process_variance_;
  double p_process_rate_variance_;
  double p_measurement_variance_;

  std::string p_frame_id_;
};

} // namespace obstacle_detector
