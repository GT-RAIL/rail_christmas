#include <tree_point_and_click/clicker.h>

using namespace std;

Clicker::Clicker() :
    pnh("~")
{
  ROS_INFO("Initializing interactive marker server...");

  // read parameters
  string cloudTopic;
  //pnh.param<string>("cloud_topic", cloudTopic, "/hw/depth_perch/points");
  pnh.param<string>("cloud_topic", cloudTopic, "/kinect/qhd/points");

  test_publisher_ = n.advertise<geometry_msgs::PointStamped>("generate_grasps", 1);
  cloudSubscriber = n.subscribe(cloudTopic, 1, &Clicker::cloudCallback, this);

  imServer.reset( new interactive_markers::InteractiveMarkerServer("clicker", "clickable_point_markers", false));

  ros::Duration(0.1).sleep();

  imServer->applyChanges();

  interactiveCloud.pose.orientation.w = 1.0;
  interactiveCloud.scale = 1.0;
  interactiveCloud.name = "interactive_cloud_marker";
  interactiveCloud.description = "Clickable Point Cloud";

  newCloudReceived = false;

  ROS_INFO("Initialized.");
}

void Clicker::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &pc)
{
  boost::recursive_mutex::scoped_lock lock(cloudMutex);

  cloud = *pc;

  newCloudReceived = true;
}

void Clicker::processInteractiveCloudFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  boost::recursive_mutex::scoped_lock lock(cloudMutex);
  switch (feedback->event_type)
  {
    //Send a stop command so that when the marker is released the arm stops moving
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      if (feedback->mouse_point.x == 0 && feedback->mouse_point.y == 0 && feedback->mouse_point.z == 0)
      {
        ROS_INFO("invalid click!");
      }
      else
      {
        ROS_INFO("Clicked point (%f, %f, %f), in frame %s...", feedback->mouse_point.x, feedback->mouse_point.y, feedback->mouse_point.z, feedback->header.frame_id.c_str());

        geometry_msgs::PointStamped clickedPoint, convertedPoint;
        clickedPoint.header.frame_id = feedback->header.frame_id;
        clickedPoint.point = feedback->mouse_point;
        tfListener.transformPoint("kinect_rgb_optical_frame", ros::Time(0), clickedPoint, "kinect_rgb_optical_frame", convertedPoint);
        ROS_INFO("In kinect_rgb_optical_frame frame: (%f, %f, %f)", convertedPoint.point.x, convertedPoint.point.y, convertedPoint.point.z);

        test_publisher_.publish(convertedPoint);
      }
      break;
    default:
      break;
  }

  //Update interactive marker server
  imServer->applyChanges();
}

void Clicker::updateMarker()
{
  if (newCloudReceived)
  {
    boost::recursive_mutex::scoped_lock lock(cloudMutex);

    interactiveCloud.controls.clear();


    //create new marker
    visualization_msgs::Marker marker;
    // set header field
    marker.header.frame_id = cloud.header.frame_id;
    // default position
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    // default scale
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    // set the type of marker and our color of choice
    marker.type = visualization_msgs::Marker::POINTS;


    // convert to an easy to use point cloud message
    sensor_msgs::PointCloud pc;
    sensor_msgs::convertPointCloud2ToPointCloud(cloud, pc);

    // place in the marker message
    marker.points.resize(pc.points.size());
    marker.colors.resize(pc.points.size());

//    //create marker for each point
//    visualization_msgs::InteractiveMarkerControl clickControl;
//    clickControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
//    clickControl.name = "interactive_cloud_marker_control";
    for (size_t j = 0; j < pc.points.size(); j++)
    {
//      visualization_msgs::Marker marker;
//      //marker.header.frame_id = cloud.header.frame_id;
//
//      marker.pose.position.x = pc.points[j].x;
//      marker.pose.position.y = pc.points[j].y;
//      marker.pose.position.z = pc.points[j].z;
//      marker.pose.orientation.w = 1.0;
//      marker.scale.x = 0.005;
//      marker.scale.y = 0.0005;
//      marker.scale.z = 0.0005;
//
//      marker.type = visualization_msgs::Marker::SPHERE;
//      // point cloud has no color, fill one in
////      uint32_t rgb = *reinterpret_cast<int *>(&pc.channels[0].values[j]);
////      marker.color.r = (float)((int) ((rgb >> 16) & 0x0000ff))/255.0f;
////      marker.color.g = (float)((int) ((rgb >> 8) & 0x0000ff))/255.0f;
////      marker.color.b = (float)((int) ((rgb) & 0x0000ff))/255.0f;
//      marker.color.r = 0.8f;
//      marker.color.g = 0.8f;
//      marker.color.b = 0.8f;
//      marker.color.a = 1.0;
//
//      clickControl.markers.push_back(marker);

      if (pc.points[j].x > -5.0 && pc.points[j].x < 5.0 && pc.points[j].y > -5.0 && pc.points[j].y < 5.0
          && pc.points[j].z > -5.0 && pc.points[j].z < 5.0)
      {
        marker.points[j].x = pc.points[j].x;
        marker.points[j].y = pc.points[j].y;
        marker.points[j].z = pc.points[j].z;
      }
      else
      {
        marker.points[j].x = 0;
        marker.points[j].y = 0;
        marker.points[j].z = 0;
      }
        // use average RGB
  //      uint32_t rgb = *reinterpret_cast<int *>(&pc.channels[0].values[j]);
  //      marker.colors[j].r = (float)((int) ((rgb >> 16) & 0x0000ff))/255.0f;
  //      marker.colors[j].g = (float)((int) ((rgb >> 8) & 0x0000ff))/255.0f;
  //      marker.colors[j].b = (float)((int) ((rgb) & 0x0000ff))/255.0f;
      uint32_t rgb = *reinterpret_cast<int *>(&pc.channels[0].values[j]);
      marker.colors[j].r = (float)((int) ((rgb >> 16) & 0x0000ff))/255.0f;
      marker.colors[j].g = (float)((int) ((rgb >> 8) & 0x0000ff))/255.0f;
      marker.colors[j].b = (float)((int) ((rgb) & 0x0000ff))/255.0f;
//      marker.colors[j].r = 0.8f;
//      marker.colors[j].g = 0.8f;
//      marker.colors[j].b = 0.8f;
      marker.colors[j].a = 1.0;

    }

    //update interactive marker

    interactiveCloud.header.frame_id = cloud.header.frame_id;
    visualization_msgs::InteractiveMarkerControl clickControl;
    clickControl.markers.push_back(marker);
    clickControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    clickControl.name = "interactive_cloud_marker_control";

    interactiveCloud.header.frame_id = cloud.header.frame_id;
    interactiveCloud.controls.push_back(clickControl);
    imServer->clear();
    imServer->insert(interactiveCloud);
    imServer->setCallback(interactiveCloud.name, boost::bind(&Clicker::processInteractiveCloudFeedback, this, _1));
    imServer->applyChanges();
    newCloudReceived = false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "clicker");

  Clicker c;

  ros::Rate loop_rate(5);
  while (ros::ok())
  {
    c.updateMarker();
    ros::spinOnce();
    loop_rate.sleep();
  }
}
