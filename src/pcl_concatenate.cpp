#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <time.h>
#include <stdlib.h>

ros::Publisher pub;
pcl::PCLPointCloud2 assembled_cloud;
pcl::PCLPointCloud2 octomap_cloud;

std::string current_date() {
    time_t now = time(NULL);
    struct tm tstruct;
    char buf[40];
    tstruct = *localtime(&now);
    //format: day DD-MM-YYYY
    strftime(buf, sizeof(buf), "%d_%m_%Y", &tstruct);

    std::string s(buf);
    return s;
}

std::string current_time() {
    time_t now = time(NULL);
    struct tm tstruct;
    char buf[40];
    tstruct = *localtime(&now);
    //format: HH:mm:ss
    strftime(buf, sizeof(buf), "%X", &tstruct);

    std::string s(buf);
    return s;
}

void cloud_cb (const sensor_msgs::PointCloud2 cloud_msg) {
    ROS_DEBUG("[pcl_concatenate] cloud callback");

    pcl::PCLPointCloud2 pclmsg;
    pcl_conversions::toPCL(cloud_msg, pclmsg);
    pcl::concatenatePointCloud(assembled_cloud, pclmsg, assembled_cloud);
}

void cloud_octomap (const sensor_msgs::PointCloud2 cloud_msg) {
    ROS_DEBUG("[pcl_concatenate] cloud_octomap callback");

    pcl_conversions::toPCL(cloud_msg, octomap_cloud);
}

int main (int argc, char** argv) {
    ROS_INFO("[pcl_concatenate] starting node\n");        

    // Initialize ROS
    ros::init (argc, argv, "pcl_concatenation_node");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("assembled_pointcloud", 1, cloud_cb);
    ros::Subscriber sub2 = nh.subscribe ("octomap_point_cloud_centers", 1, cloud_octomap);

    bool capture_pc_param = true;
    std::string save_pc_path = "/tmp/";

    ros::param::get("~save_point_cloud_path", save_pc_path);
    ROS_INFO("[pcl_concatenate] saving point cloud to: %s", save_pc_path.c_str());

    // Spin
    ros::Rate loop_rate(10);
    ros::spinOnce();
    while(ros::ok())
    {   
        nh.getParam("capture_point_cloud_360", capture_pc_param);

        if(!capture_pc_param){
            ROS_INFO("[pcl_concatenate] shutting down");
            sub.shutdown();
            sub2.shutdown();
            break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("[pcl_concatenate] saving pount clouds....");

    // pcl::PointCloud<pcl::PointXYZ> point_cloud;
    // pcl::fromPCLPointCloud2(assembled_cloud, point_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(assembled_cloud, *point_cloud);

    // Create the filtering object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(point_cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered);
    ROS_INFO_STREAM("PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl);

    std::string c_date = current_date();
    std::string c_time = current_time();

    std::string assembled_file_path = save_pc_path + "assembled_point_cloud_" + c_date + "_" + c_time;
    ROS_INFO("[pcl_concatenate] saving assembled pount cloud to: %s", assembled_file_path.c_str());
    pcl::io::savePLYFile(assembled_file_path + ".ply", *cloud_filtered);
    pcl::io::savePCDFile(assembled_file_path + ".pcd", *cloud_filtered);
    ROS_INFO("[pcl_concatenate] assembled pointcloud saved");

    pcl::PointCloud<pcl::PointXYZ> octomap_point_cloud;
    pcl::fromPCLPointCloud2(octomap_cloud, octomap_point_cloud);

    std::string octomap_file_path = save_pc_path + "octomap_point_cloud_" + c_date + "_" + c_time;
    ROS_INFO("[pcl_concatenate] saving octomap point cloud to: %s", octomap_file_path.c_str());
    pcl::io::savePLYFile(octomap_file_path + ".ply", octomap_point_cloud);
    pcl::io::savePCDFile(octomap_file_path + ".pcd", octomap_point_cloud);
    ROS_INFO("[pcl_concatenate] octomap pointcloud saved");

    ROS_INFO("[pcl_concatenate] exiting node...");
    system("rosnode kill point_cloud_converter");
    system("rosnode kill custom_pc_assembler");
}