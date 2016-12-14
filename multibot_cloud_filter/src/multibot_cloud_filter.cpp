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
std::string map_frame_;

void callback(const PointCloudT::ConstPtr &cloud_in)
{
  PointCloudT::Ptr cloud_vg = boost::make_shared<PointCloudT>();
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(cloud_in);
  vg.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
  vg.setDownsampleAllData(false);
  vg.filter(*cloud_vg);

  PointCloudT::Ptr cloud_map = boost::make_shared<PointCloudT>();
  try
  {
    if (!tf_->waitForTransform(map_frame_,
                               cloud_vg->header.frame_id,
                               ros::Time().fromNSec(cloud_vg->header.stamp * 1000),
                               ros::Duration(5.0)))
    {
      ROS_WARN("Waited for transform from %s to %s unsuccessfully, will ignore point cloud!",
               map_frame_.c_str(),
               cloud_vg->header.frame_id.c_str());
      return;
    }
    if (!pcl_ros::transformPointCloud(map_frame_, *cloud_vg, *cloud_map, *tf_))
    {
      ROS_WARN("Failed to transform cloud from %s to %s, will ignore point cloud!",
               map_frame_.c_str(),
               cloud_vg->header.frame_id.c_str());
      return;
    }
    cloud_map->header.stamp = cloud_vg->header.stamp;
    cloud_map->header.frame_id = map_frame_;
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  std::vector<PointCloudT::Ptr> clouds_transformed;
  for (size_t j = 0; j < other_robot_frames_.size(); ++j)
  {
    PointCloudT::Ptr cloud_transformed = boost::make_shared<PointCloudT>();
    try
    {
      if (!tf_->waitForTransform(other_robot_frames_[j],
                                 cloud_map->header.frame_id,
                                 ros::Time().fromNSec(cloud_map->header.stamp * 1000),
                                 ros::Duration(5.0)))
      {
        ROS_WARN("Waited for transform from %s to %s unsuccessfully, will not filter out other robot!",
                 other_robot_frames_[j].c_str(),
                 cloud_map->header.frame_id.c_str());
        continue;
      }
      if (!pcl_ros::transformPointCloud(other_robot_frames_[j], *cloud_map, *cloud_transformed, *tf_))
      {
        ROS_WARN("Failed to transform cloud from %s to %s, will not filter out other robot!",
                 other_robot_frames_[j].c_str(),
                 cloud_map->header.frame_id.c_str());
        continue;
      }
      cloud_transformed->header.stamp = cloud_map->header.stamp;
      cloud_transformed->header.frame_id = other_robot_frames_[j];
      assert(cloud_map->size() == cloud_transformed->size());
      clouds_transformed.push_back(cloud_transformed);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      return;
    }
  }

  PointCloudT::Ptr cloud_out = boost::make_shared<PointCloudT>();
  cloud_out->reserve(cloud_map->size());
  cloud_out->header = cloud_map->header;

  for (size_t i = 0; i < cloud_map->size(); i++)
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
      cloud_out->push_back(cloud_map->points[i]);
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

  pub_ = nh.advertise<PointCloudT>("cloud_out", 1);

  tf_ = new tf::TransformListener(nh, ros::Duration(10.0));
  ros::Duration(3.0).sleep();   // allow TF buffer to fill up

  private_nh.param<std::string>("map_frame", map_frame_, "/map");

  std::vector<std::string> tmp_other_robot_frames;
  private_nh.getParam("other_robot_frames", tmp_other_robot_frames);
  std::string tf_prefix(tf::getPrefixParam(private_nh));
  for (size_t j = 0; j < tmp_other_robot_frames.size(); ++j)
  {
    tmp_other_robot_frames[j] = tf::resolve(tf_prefix, tmp_other_robot_frames[j]);
    try
    {
      if (!tf_->waitForTransform(tmp_other_robot_frames[j],
                                 map_frame_,
                                 ros::Time(),
                                 ros::Duration(5.0)))
      {
        ROS_WARN("Warning: skipping other robot frame %s", tmp_other_robot_frames[j].c_str());
      }
      else
      {
        other_robot_frames_.push_back(tmp_other_robot_frames[j]);
      }
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ROS_WARN("Warning: skipping other robot frame %s", tmp_other_robot_frames[j].c_str());
    }
  }

  ros::Subscriber sub(nh.subscribe("cloud_in", 1, callback));

  ros::spin();

  delete tf_;
  return 0;
}
