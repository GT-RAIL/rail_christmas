#include <tree_point_and_click/generator.h>

using std::ios;
using std::string;
using std::stringstream;
using std::vector;

Generator::Generator() :
    pnh_("~"),
    sample_grasps_client_("/rail_agile/sample_grasps"),
    rank_grasps_poi_client_("/grasp_sampler/rank_grasps_poi")
{
  string cloud_topic;
  pnh_.param<string>("cloud_topic", cloud_topic, "/kinect2/qhd/points");
  pnh_.param<double>("roi_radius", roi_radius_, 0.1);

  cloud_received_ = false;

  grasps_publisher_ = pnh_.advertise<geometry_msgs::PoseArray>("grasps", 1);
  best_grasp_publisher_ = pnh_.advertise<geometry_msgs::PoseStamped>("grasp_topic", 1);

  cloud_subscriber_ = n_.subscribe(cloud_topic, 1, &Generator::cloudCallback, this);
  generate_subscriber_ = n_.subscribe("generate_grasps", 1, &Generator::generateCallback, this);
}

void Generator::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  boost::mutex::scoped_lock lock(cloud_mutex_);

  cloud_ = *msg;

  //TODO: not sure why, but it seems this field is wrong
  for (size_t i = 0; i < cloud_.fields.size(); i ++)
  {
    cloud_.fields[i].count = 1;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 temp;
  pcl_conversions::toPCL(cloud_, temp);
  pcl::fromPCLPointCloud2(temp, *pc);

  cloud_received_ = true;
}

void Generator::generateCallback(const geometry_msgs::PointStamped::ConstPtr &point)
{
  if (!cloud_received_)
    return;

  string original_frame = point->header.frame_id;
  string cloud_frame = cloud_.header.frame_id;

  // get point in cloud frame
  geometry_msgs::PointStamped poi = *point;
  if (point->header.frame_id != cloud_frame)
  {
    tf_listener_.transformPoint(cloud_frame, ros::Time(0), *point, original_frame, poi);
  }

  // sample grasps
  ROS_INFO("Sampling grasps...");

  rail_grasp_calculation_msgs::SampleGraspsGoal sample_goal;
  sample_goal.cloud = cloud_;

  for (size_t i = 0; i < cloud_.fields.size(); i ++)
  {
    ROS_INFO("Field %lu: %s", i, cloud_.fields[i].name.c_str());
  }

  sample_goal.workspace.mode = rail_grasp_calculation_msgs::Workspace::WORKSPACE_VOLUME;
  sample_goal.workspace.x_min = poi.point.x - roi_radius_;
  sample_goal.workspace.y_min = poi.point.y - roi_radius_;
  sample_goal.workspace.z_min = poi.point.z - roi_radius_;
  sample_goal.workspace.x_max = poi.point.x + roi_radius_;
  sample_goal.workspace.y_max = poi.point.y + roi_radius_;
  sample_goal.workspace.z_max = poi.point.z + roi_radius_;

  ROS_INFO("Workspace: %f, %f, %f, %f, %f, %f", sample_goal.workspace.x_min, sample_goal.workspace.y_min, sample_goal.workspace.z_min, sample_goal.workspace.x_max, sample_goal.workspace.y_max, sample_goal.workspace.z_max);

  sample_grasps_client_.sendGoal(sample_goal);
  sample_grasps_client_.waitForResult(ros::Duration(10.0));
  geometry_msgs::PoseArray sampled_grasps = sample_grasps_client_.getResult()->graspList;

  if (sampled_grasps.poses.empty())
  {
    ROS_INFO("No grasps sampled!");
    return;
  }
  else
  {
    ROS_INFO("Sampled %lu grasps.  Clustering and ranking...", sampled_grasps.poses.size());
  }

  // rank grasps
  rail_grasp_calculation_msgs::RankGraspsGoal rank_goal;
  rank_goal.sceneCloud = sample_goal.cloud;
  rank_goal.graspList = sampled_grasps;
  rank_goal.workspace = sample_goal.workspace;
  rank_grasps_poi_client_.sendGoal(rank_goal);

  rank_grasps_poi_client_.waitForResult(ros::Duration(10.0));
  rail_grasp_calculation_msgs::RankGraspsResultConstPtr rank_result = rank_grasps_poi_client_.getResult();

  // transform results to original frame
  geometry_msgs::PoseArray ranked_list;
  ranked_list.header.frame_id = original_frame;
  geometry_msgs::PoseStamped temp_pose_in, temp_pose_out;
  temp_pose_in.header.frame_id = ranked_list.header.frame_id;
  ranked_list.poses.resize(rank_result->graspList.poses.size());
  for (size_t i = 0; i < rank_result->graspList.poses.size(); i ++)
  {
    temp_pose_in.pose = rank_result->graspList.poses[i];
    tf_listener_.transformPose(original_frame, ros::Time(0), temp_pose_in, original_frame, temp_pose_out);
    ranked_list.poses[i] = temp_pose_out.pose;
  }

  ROS_INFO("Ranked %lu grasps after clustering.", ranked_list.poses.size());

  // publish results
  grasps_publisher_.publish(ranked_list);

  // debug publishing
  if (!ranked_list.poses.empty())
  {
    geometry_msgs::PoseStamped best_pose;
    best_pose.header.frame_id = original_frame;
    best_pose.pose = ranked_list.poses[0];
    best_grasp_publisher_.publish(best_pose);
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "generator");

  Generator g;

  ros::spin();

  return EXIT_SUCCESS;
}
