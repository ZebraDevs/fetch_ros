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

#ifndef FETCH_DEPTH_LAYER_DEPTH_LAYER_H
#define FETCH_DEPTH_LAYER_DEPTH_LAYER_H

#include <boost/thread/mutex.hpp>
#include <costmap_2d/voxel_layer.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/rgbd/rgbd.hpp>
#include <sensor_msgs/image_encodings.h>

namespace costmap_2d
{

/**
 * @class FetchDepthLayer
 * @brief A costmap layer that extracts ground plane and clears it.
 */
class FetchDepthLayer : public VoxelLayer
{
public:
  /**
   * @brief Constructor
   */
  FetchDepthLayer();

  /**
   * @brief Destructor for the depth costmap layer
   */
  virtual ~FetchDepthLayer();

  /**
   * @brief Initialization function for the DepthLayer
   */
  virtual void onInitialize();

private:
  void cameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& msg);
  void depthImageCallback(
    const sensor_msgs::Image::ConstPtr& msg);

  boost::shared_ptr<costmap_2d::ObservationBuffer> marking_buf_;
  boost::shared_ptr<costmap_2d::ObservationBuffer> clearing_buf_;

  bool publish_observations_;
  double ground_threshold_;
  double observations_threshold_;

  // retrieves depth image from head_camera
  // used to fit ground plane to
  ros::Subscriber depth_image_sub_;

  // retrieves camera matrix for head_camera
  // used in calculating ground plane
  ros::Subscriber camera_info_sub_;

  // publishes clearing observations
  ros::Publisher clearing_pub_;

  // publishes marking observations
  ros::Publisher marking_pub_;

  // camera intrinsics
  boost::mutex mutex_K_;
  cv::Mat K_;

  // clean the depth image
  cv::Ptr<cv::DepthCleaner> depth_cleaner_;

  // depth image estimation
  cv::Ptr<cv::RgbdNormals> normals_estimator_;
  cv::Ptr<cv::RgbdPlane> plane_estimator_;
};

}  // namespace costmap_2d

#endif  // FETCH_DEPTH_LAYER_DEPTH_LAYER_H
