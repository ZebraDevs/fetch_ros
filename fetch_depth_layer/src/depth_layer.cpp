/*
 * Copyright 2015 Fetch Robotics Inc.
 * All Rights Reserved.
 *
 * THIS WORK, IN SOURCE OR BINARY FORMAT IS PROVIDED UNDER THE TERMS
 * OF THE CREATIVE COMMONS ATTRIBUTION-NONCOMMERCIAL-NODERIVATIVES
 * 4.0 INTERNATIONAL LICENSE. A FULL COPY OF THE LICENSE CAN BE FOUND
 * AT https://creativecommons.org/licenses/by-nc-nd/4.0/

 * THE WORK IS PROTECTED BY COPYRIGHT AND/OR OTHER APPLICABLE LAW.
 * ANY USE OF THE WORK OTHER THAN AS AUTHORIZED UNDER THIS LICENSE
 * OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 * Author: Anuj Pasricha, Michael Ferguson
 */

#include <pluginlib/class_list_macros.h>
#include <fetch_depth_layer/depth_layer.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::FetchDepthLayer, costmap_2d::Layer)

namespace costmap_2d
{

FetchDepthLayer::FetchDepthLayer()
{
}

void FetchDepthLayer::onInitialize()
{
  VoxelLayer::onInitialize();

  double observation_keep_time = 0.0;
  double expected_update_rate = 0.0;
  double min_obstacle_height = 0.0;
  double max_obstacle_height = 2.0;
  double transform_tolerance = 0.5;
  double obstacle_range = 2.5;
  double raytrace_range = 3.0;
  std::string topic = "";
  std::string sensor_frame = "";

  marking_buf_ = boost::shared_ptr<costmap_2d::ObservationBuffer> (
  	new costmap_2d::ObservationBuffer(topic, observation_keep_time,
  	  expected_update_rate, min_obstacle_height, max_obstacle_height,
  	  obstacle_range, raytrace_range, *tf_, global_frame_,
  	  sensor_frame, transform_tolerance));
  marking_buffers_.push_back(marking_buf_);

  min_obstacle_height = 0.0;

  clearing_buf_ =  boost::shared_ptr<costmap_2d::ObservationBuffer> (
  	new costmap_2d::ObservationBuffer(topic, observation_keep_time,
  	  expected_update_rate, min_obstacle_height, max_obstacle_height,
  	  obstacle_range, raytrace_range, *tf_, global_frame_,
  	  sensor_frame, transform_tolerance));
  clearing_buffers_.push_back(clearing_buf_);

  ros::NodeHandle private_nh("~/" + name_);

  private_nh.param("publish_observations", publish_observations_, false);
  private_nh.param("ground_orientation_threshold", ground_threshold_, 0.9);
  private_nh.param("observations_separation_threshold", observations_threshold_, 0.06);

  if (publish_observations_)
  {
    clearing_pub_ = private_nh.advertise<sensor_msgs::PointCloud>("clearing_obs", 1);
    marking_pub_ = private_nh.advertise<sensor_msgs::PointCloud>("marking_obs", 1);
  }

  // TODO add params for topic names

  camera_info_sub_ = private_nh.subscribe<sensor_msgs::CameraInfo>(
    "/head_camera/depth_downsample/camera_info",
    10, &FetchDepthLayer::cameraInfoCallback, this);
  depth_image_sub_ = private_nh.subscribe<sensor_msgs::Image>(
    "/head_camera/depth_downsample/image_raw",
    10, &FetchDepthLayer::depthImageCallback, this);
}

FetchDepthLayer::~FetchDepthLayer()
{
}

void FetchDepthLayer::cameraInfoCallback(
  const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  // Lock mutex before updating K
  boost::unique_lock<boost::mutex> lock(mutex_K_);

  float focal_pixels_ = msg->P[0];
  float center_x_ = msg->P[2];
  float center_y_ = msg->P[6];

  if (msg->binning_x == msg->binning_y)
  {
    if (msg->binning_x > 0)
    {
      K_ = (cv::Mat_<double>(3, 3) <<
        focal_pixels_/msg->binning_x, 0.0, center_x_/msg->binning_x,
        0.0, focal_pixels_/msg->binning_x, center_y_/msg->binning_x,
        0.0, 0.0, 1.0);
    }
    else
    {
      K_ = (cv::Mat_<double>(3, 3) <<
        focal_pixels_, 0.0, center_x_,
        0.0, focal_pixels_, center_y_,
        0.0, 0.0, 1.0);
    }
  }
  else
  {
    ROS_ERROR("binning_x is not equal to binning_y");
  }
}

void FetchDepthLayer::depthImageCallback(
  const sensor_msgs::Image::ConstPtr& msg)
{
  // Lock mutex before using K
  boost::unique_lock<boost::mutex> lock(mutex_K_);

  if (K_.empty())
  {
    ROS_DEBUG_NAMED("depth_layer", "Camera info not yet received.");
    return;
  }

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Convert to 3d
  cv::Mat points3d;
  cv::depthTo3d(cv_ptr->image, K_, points3d);

  // Get normals
  if (normals_estimator_.empty())
  {
    normals_estimator_ = new cv::RgbdNormals(cv_ptr->image.rows,
                                             cv_ptr->image.cols,
                                             cv_ptr->image.depth(),
                                             K_);
  }
  cv::Mat normals;
  (*normals_estimator_)(points3d, normals);

  // Find plane(s)
  if (plane_estimator_.empty())
  {
    plane_estimator_ = cv::Algorithm::create<cv::RgbdPlane>("RGBD.RgbdPlane");
    // Model parameters are based on notes in opencv_candidate
    plane_estimator_->set("sensor_error_a", 0.0075);
    plane_estimator_->set("sensor_error_b", 0.0);
    plane_estimator_->set("sensor_error_c", 0.0);
    // Image/cloud height/width must be multiple of block size
    plane_estimator_->set("block_size", 40);
    // Distance a point can be from plane and still be part of it
    plane_estimator_->set("threshold", observations_threshold_);
    // Minimum cluster size to be a plane
    plane_estimator_->set("min_size", 1000);
  }
  cv::Mat planes_mask;
  std::vector<cv::Vec4f> plane_coefficients;
  (*plane_estimator_)(points3d, normals, planes_mask, plane_coefficients);

  cv::Vec4f ground_plane;
  for (size_t i=0; i < plane_coefficients.size(); i++)
  {
    // check plane orientation
    if ((fabs(0.0 - plane_coefficients[i][0]) <= ground_threshold_) &&
        (fabs(1.0 + plane_coefficients[i][1]) <= ground_threshold_) &&
        (fabs(0.0 - plane_coefficients[i][2]) <= ground_threshold_))
    {
      ground_plane = plane_coefficients[i];
      break;
    }
  }

  // check that ground plane actually exists, so walls don't count as clearing observations
  if (ground_plane[0] == 0.0 && ground_plane[1] == 0.0 &&
      ground_plane[2] == 0.0 && ground_plane[3] == 0.0)
  {
    ROS_DEBUG_NAMED("depth_layer", "Invalid ground plane.");
    return;
  }

  cv::Mat channels[3];
  cv::split(points3d, channels);

  sensor_msgs::PointCloud clearing_points;
  clearing_points.header.stamp = msg->header.stamp;
  clearing_points.header.frame_id = msg->header.frame_id;

  sensor_msgs::PointCloud marking_points;
  marking_points.header.stamp = msg->header.stamp;
  marking_points.header.frame_id = msg->header.frame_id;

  // Points at edges of image can be very noisy, exclude them
  int skip = 10;  // TODO should be ROS param

  // Put points in clearing/marking clouds
  for (size_t i=skip; i<points3d.rows-skip; i++)
  {
    for (size_t j=skip; j<points3d.cols-skip; j++)
    {
      // Get next point
      geometry_msgs::Point32 current_point;
      current_point.x = channels[0].at<float>(i, j);
      current_point.y = channels[1].at<float>(i, j);
      current_point.z = channels[2].at<float>(i, j);
      // Check point validity
      if (current_point.x != 0.0 &&
          current_point.y != 0.0 &&
          current_point.z != 0.0 &&
          !isnan(current_point.x) &&
          !isnan(current_point.y) &&
          !isnan(current_point.z))
      {
        // Check if point is part of the ground plane
        if (fabs(ground_plane[0] * current_point.x +
                 ground_plane[1] * current_point.y +
                 ground_plane[2] * current_point.z +
                 ground_plane[3]) <= observations_threshold_)
        {
          clearing_points.points.push_back(current_point);
        }
        else
        {
          // Not inlier, should it be outlier?
          int num_valid = 0;
          for (int x=-1; x < 2; x++)
          {
            for (int y=-1; y < 2; y++)
            {
              if (x == 0 && y == 0)
                continue;
              geometry_msgs::Point32 test_point;
              test_point.x = channels[0].at<float>(i+x, j+y);
              test_point.y = channels[1].at<float>(i+x, j+y);
              test_point.z = channels[2].at<float>(i+x, j+y);
              if (test_point.x != 0.0 &&
                  test_point.y != 0.0 &&
                  test_point.z != 0.0 &&
                  !isnan(test_point.x) &&
                  !isnan(test_point.y) &&
                  !isnan(test_point.z))
              {
                if ( fabs(test_point.x - current_point.x) < 0.1 &&
                     fabs(test_point.y - current_point.y) < 0.1 &&
                     fabs(test_point.z - current_point.z) < 0.1)
                {
                  num_valid++;
                }
              }
            }  // for y
          }  // for x
          if (num_valid >= 7)
          {
            marking_points.points.push_back(current_point);
          }
        }
      }
    }
  }

  if (clearing_points.points.size() > 0)
  {
    if (publish_observations_)
    {
      clearing_pub_.publish(clearing_points);
    }

    sensor_msgs::PointCloud2 clearing_cloud2;
    if (!sensor_msgs::convertPointCloudToPointCloud2(clearing_points, clearing_cloud2))
    {
      ROS_ERROR("Failed to convert a PointCloud to a PointCloud2, dropping message");
      return;
    }

    // buffer the ground plane observation
    clearing_buf_->lock();
    clearing_buf_->bufferCloud(clearing_cloud2);
    clearing_buf_->unlock();
  }

  if (marking_points.points.size() > 0)
  {
    if (publish_observations_)
    {
      marking_pub_.publish(marking_points);
    }

    sensor_msgs::PointCloud2 marking_cloud2;
    if (!sensor_msgs::convertPointCloudToPointCloud2(marking_points, marking_cloud2))
    {
      ROS_ERROR("Failed to convert a PointCloud to a PointCloud2, dropping message");
      return;
    }

    marking_buf_->lock();
    marking_buf_->bufferCloud(marking_cloud2);
    marking_buf_->unlock();
  }
}

}  // namespace costmap_2d
