#ifndef TREE_POINT_AND_CLICK_GENERATOR_H
#define TREE_POINT_AND_CLICK_GENERATOR_H

// Boost
#include <boost/thread/mutex.hpp>

// ROS
#include <actionlib/client/simple_action_client.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <rail_grasp_calculation_msgs/RankGraspsAction.h>
#include <rail_grasp_calculation_msgs/SampleGraspsAction.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class Generator
{

public:
  /**
   * @brief Initialize ROS message, services, and actions required for grasp suggestion.
   */
  Generator();

private:

  /**
   * @brief Store the full scene point cloud.
   * @param msg incoming point cloud data
   */
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

  void generateCallback(const geometry_msgs::PointStamped::ConstPtr &point);

  ros::NodeHandle n_, pnh_;

  // topics
  ros::Publisher grasps_publisher_;
  ros::Publisher best_grasp_publisher_;
  ros::Subscriber cloud_subscriber_;
  ros::Subscriber generate_subscriber_;

  // actionlib
  actionlib::SimpleActionClient<rail_grasp_calculation_msgs::SampleGraspsAction> sample_grasps_client_;
  actionlib::SimpleActionClient<rail_grasp_calculation_msgs::RankGraspsAction> rank_grasps_poi_client_;

  tf::TransformListener tf_listener_;

  boost::mutex cloud_mutex_;  /// mutex for full scene point cloud

  sensor_msgs::PointCloud2 cloud_;  /// stored full scene point cloud

  bool cloud_received_;  /// true once first point cloud is received

  double roi_radius_;
};

#endif  // TREE_POINT_AND_CLICK_GENERATOR_H
