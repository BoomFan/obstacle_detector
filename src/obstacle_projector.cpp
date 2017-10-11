/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, University of Michigan, ROAHM Lab
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
 *     * Neither the name of the University of Michigan nor the names
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
 * Author: Fan Bu
 */

#include "obstacle_detector/obstacle_projector.h"
#include "pred.h"


using namespace obstacle_detector;
using namespace arma;
using namespace std;

ObstacleProjector::ObstacleProjector(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) {
  p_active_ = false;

  timer_ = nh_.createTimer(ros::Duration(1.0), &ObstacleProjector::timerCallback, this, false, false);
  params_srv_ = nh_local_.advertiseService("params", &ObstacleProjector::updateParams, this);

  initialize();
}

ObstacleProjector::~ObstacleProjector() {
  nh_local_.deleteParam("active");
  nh_local_.deleteParam("copy_segments");

  nh_local_.deleteParam("loop_rate");
  nh_local_.deleteParam("tracking_duration");
  nh_local_.deleteParam("min_correspondence_cost");
  nh_local_.deleteParam("std_correspondence_dev");
  nh_local_.deleteParam("process_variance");
  nh_local_.deleteParam("process_rate_variance");
  nh_local_.deleteParam("measurement_variance");

  nh_local_.deleteParam("frame_id");
}

bool ObstacleProjector::updateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  bool prev_active = p_active_;

  nh_local_.param<bool>("active", p_active_, true);
  nh_local_.param<bool>("copy_segments", p_copy_segments_, true);

  nh_local_.param<double>("loop_rate", p_loop_rate_, 100.0);
  p_sampling_time_ = 1.0 / p_loop_rate_;
  p_sensor_rate_ = 10.0;    // 10 Hz for Hokuyo

  nh_local_.param<double>("tracking_duration", p_tracking_duration_, 2.0);
  nh_local_.param<double>("min_correspondence_cost", p_min_correspondence_cost_, 0.3);
  nh_local_.param<double>("std_correspondence_dev", p_std_correspondence_dev_, 0.15);
  nh_local_.param<double>("process_variance", p_process_variance_, 0.01);
  nh_local_.param<double>("process_rate_variance", p_process_rate_variance_, 0.1);
  nh_local_.param<double>("measurement_variance", p_measurement_variance_, 1.0);

  nh_local_.param<string>("frame_id", p_frame_id_, string("map"));

  timer_.setPeriod(ros::Duration(p_sampling_time_), false);

  if (p_active_ != prev_active) {
    if (p_active_) {

      // build subscriber
      obs_pcd_sub_ = nh_.subscribe("/forecast/output", 1, &ObstacleProjector::obsPcdCallback, this);
      image_sub_ = nh_.subscribe("/camera/left/image_rect_color", 1, &ObstacleProjector::imageCallback, this);
      // TODO: Add a camera info call back
      // TODO: Add a camera info call back
      // TODO: Add a camera info call back

      // build publisher
      image_transport::ImageTransport it(nh_);
      img_projected_pub_ = it.advertise("/forecast/images", 1);

      timer_.start();
    }
    else {
      // No need to send empty message, I guess?

      // Shut down subscribers and publishers
      obs_pcd_sub_.shutdown();
      image_sub_.shutdown();

      img_projected_pub_.shutdown();

      // TODO: Clear image data and pointcloud data
      // tracked_obstacles_.clear();
      // untracked_obstacles_.clear();

      timer_.stop();
    }
  }

  return true;
}

void ObstacleProjector::timerCallback(const ros::TimerEvent&) {
  publishPredictImage();
}

void ObstacleProjector::obsPcdCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pred_pcd_ptr) {
  // If there is data in pointcloud, then transform the frame into "camera_link" frame
  if (pred_pcd_ptr->width > 0) {
    try{
      tf_listener.lookupTransform("camera_link", pred_pcd_ptr->header.frame_id, ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_INFO("Waiting on TF cache to build: %s",ex.what());
      return;
    }
    // Make new point cloud that is transformed into "camera_link" frame
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl_ros::transformPointCloud(*pred_pcd_ptr, *now_pcd_ptr, transform);
  }

}

void ObstacleProjector::imageCallback(const sensor_msgs::ImageConstPtr& img_ptr) {
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_ptr, "rgb8");
  printf("Received one new Image.\n");
  //cv::imwrite("rgb.png", cv_ptr->image);
  //cv::Mat img = cv_ptr->image;
  now_img = cv_ptr->image;
}

void ObstacleProjector::publishPredictImage() {
  ros::Time now = ros::Time::now();
  sensor_msgs::ImagePtr output = cv_bridge::CvImage(std_msgs::Header(), "rgb8", now_img).toImageMsg();
  output->header.stamp = now;
  img_projected_pub_.publish(output);
}

// Ugly initialization of static members of tracked obstacles...
int    TrackedObstacle::s_fade_counter_size_     = 0;
double TrackedObstacle::s_sampling_time_         = 100.0;
double TrackedObstacle::s_process_variance_      = 0.0;
double TrackedObstacle::s_process_rate_variance_ = 0.0;
double TrackedObstacle::s_measurement_variance_  = 0.0;
