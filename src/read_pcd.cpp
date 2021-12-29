#include<ros/ros.h>
#include <string>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h> 
#include<pcl/io/pcd_io.h>//which contains the required definitions to load and store point clouds to PCD and other file formats.
 
main (int argc, char **argv)
{
  ros::init (argc, argv, "UandBdetect");
  ros::NodeHandle nh("~");
  std::string pcd_file;
  nh.param("pcd_file", pcd_file, std::string("/home/jjj/catkin_ws/src/read_pcd/pcd/jjj.pcd"));
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
//   pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2 output;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>); 
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>); 
  // pcl::io::loadPCDFile ("/home/jjj/catkin_ws/src/read_pcd/pcd/jjj_3.pcd", *cloud); //修改自己pcd文件所在路径
  pcl::io::loadPCDFile (pcd_file, *cloud);
  //降采样
  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.05f, 0.05f, 0.05f);
  sor.filter(*cloud_filtered); 
  //Convert the cloud to ROS message
  pcl::toROSMsg(*cloud_filtered, output);
  output.header.frame_id = "world";//this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer    
//！！！这一步需要注意，是后面rviz的 fixed_frame  !!!敲黑板，画重点。
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    pcl_pub.publish(output);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}