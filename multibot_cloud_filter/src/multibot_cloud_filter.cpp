/*
 * Copyright (C) 2016, DFKI GmbH
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
 *     * Neither the name of DFKI GmbH nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
 *
 *      Author:
 *         Martin Günther <martin.guenther@dfki.de>
 *
 */

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

tf::TransformListener* tf_;
ros::Publisher pub_;

std::vector<std::string> other_robot_frames_;

double voxel_size_;
double robot_radius_;

void callback(const PointCloudT::ConstPtr &cloud_in)
{
  PointCloudT::Ptr cloud_vg = boost::make_shared<PointCloudT>();
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(cloud_in);
  vg.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
  vg.setDownsampleAllData(false);
  vg.filter(*cloud_vg);

  std::vector<PointCloudT::Ptr> clouds_transformed;
  for (size_t j = 0; j < other_robot_frames_.size(); ++j)
  {
    PointCloudT::Ptr cloud_transformed = boost::make_shared<PointCloudT>();
    pcl_ros::transformPointCloud(other_robot_frames_[j], *cloud_vg, *cloud_transformed, *tf_);
    cloud_transformed->header.stamp = cloud_vg->header.stamp;
    cloud_transformed->header.frame_id = other_robot_frames_[j];
    assert(cloud_vg->size() == cloud_transformed->size());
    clouds_transformed.push_back(cloud_transformed);
  }

  PointCloudT::Ptr cloud_out = boost::make_shared<PointCloudT>();
  cloud_out->reserve(cloud_vg->size());
  cloud_out->header = cloud_vg->header;

  for (size_t i = 0; i < cloud_vg->size(); i++)
  {
    // only keep point if not close to any other robot frame
    bool keep_point = true;
    for (size_t j = 0; j < clouds_transformed.size(); ++j)
    {
      const PointT &p = clouds_transformed[j]->points[i];

      double dist = sqrt(p.x * p.x + p.y * p.y);
      if (dist <= robot_radius_)
      {
        keep_point = false;
        break;
      }
    }

    if (keep_point)
      cloud_out->push_back(cloud_vg->points[i]);
  }

  pub_.publish(cloud_out);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multibot_cloud_filter");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  voxel_size_ = private_nh.param("voxel_size", 0.05);
  robot_radius_ = private_nh.param("robot_radius", 0.2);

  if (!private_nh.getParam("other_robot_frames", other_robot_frames_))
  {
    ROS_FATAL("Need to set parameter other_robot_frames!");
    return 1;
  }

  std::string tf_prefix(tf::getPrefixParam(private_nh));
  for (size_t j = 0; j < other_robot_frames_.size(); ++j)
    other_robot_frames_[j] = tf::resolve(tf_prefix, other_robot_frames_[j]);

  tf_ = new tf::TransformListener(nh, ros::Duration(3.0));

  message_filters::Subscriber<PointCloudT> sub;
  tf::MessageFilter<PointCloudT> *tf_filter;

  pub_ = nh.advertise<PointCloudT>("cloud_out", 10);

  sub.subscribe(nh, "cloud_in", 10);
  tf_filter = new tf::MessageFilter<PointCloudT>(sub, *tf_, "dummy_frame", 10);
  tf_filter->setTargetFrames(other_robot_frames_);
  tf_filter->registerCallback(boost::bind(callback, _1));

  ros::spin();

  delete tf_filter;
  delete tf_;

  return 0;
}
