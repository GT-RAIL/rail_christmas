#ifndef TREE_POINT_AND_CLICK_CLICKER_H_
#define TREE_POINT_AND_CLICK_CLICKER_H_

//ROS
#include <ros/ros.h>
#include <boost/thread/recursive_mutex.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>

//PCL
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class Clicker
{

public:

  Clicker();

  void updateMarker();

private:
  void processInteractiveCloudFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &pc);

  ros::NodeHandle n;
  ros::NodeHandle pnh;

  //topics
  ros::Subscriber cloudSubscriber;
  ros::Publisher test_publisher_;

  boost::recursive_mutex cloudMutex;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> imServer;
  visualization_msgs::InteractiveMarker interactiveCloud;

  sensor_msgs::PointCloud2 cloud;

  bool newCloudReceived;  //flag for whether a new point cloud was received since the previous main loop execution
  bool cloudInitialized;  //flag for first point cloud received from a new topic

  tf::TransformListener tfListener;
};

#endif  // TREE_POINT_AND_CLICK_CLICKER_H_