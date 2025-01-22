#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/crop_box.h>

#include <pcl/search/impl/search.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/feature.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>

#include <opencv2/opencv.hpp>

#include <Eigen/Core>

#include <pcl/visualization/cloud_viewer.h> 

#include "pointcloud2_opr/point_cloud_process.h"

#include "tools/dynamic_reconfigure.h"
#include "tools/rviz_draw.h"
#include "tools/ros_topic_manager.h"
#include "tools/conversion_bridge.h"

#include "calibration_tool.h"

using namespace lidar_camera_cal;
using namespace lidar_camera_cal::pointcloud2_opr;


void setShareParam(ros::NodeHandle nh, RQTConfig rqt_config)
{
	nh.setParam("/shared_parameter/center_x", rqt_config.TransformFilterConfig.center_x);
	nh.setParam("/shared_parameter/center_y", rqt_config.TransformFilterConfig.center_y);
	nh.setParam("/shared_parameter/center_z", rqt_config.TransformFilterConfig.center_z);
	nh.setParam("/shared_parameter/length_x", rqt_config.TransformFilterConfig.length_x);
	nh.setParam("/shared_parameter/length_y", rqt_config.TransformFilterConfig.length_y);
	nh.setParam("/shared_parameter/length_z", rqt_config.TransformFilterConfig.length_z);
	nh.setParam("/shared_parameter/rotate_x", rqt_config.TransformFilterConfig.rotate_x);
	nh.setParam("/shared_parameter/rotate_y", rqt_config.TransformFilterConfig.rotate_y);
	nh.setParam("/shared_parameter/rotate_z", rqt_config.TransformFilterConfig.rotate_z);
}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "pointcloud_process_node");
	ros::NodeHandle rosHandle;


	std::string frame_id;
	std::string topic_pc_sub;
    std::string topic_pc_pub;
    std::string topic_cor_pub;
	rosHandle.param("frame_id", frame_id, std::string("livox_frame"));
	rosHandle.param("pointcloud_process_pc_sub_topic", topic_pc_sub, std::string("/rslidar_points"));
    rosHandle.param("pointcloud_process_pc_pub_topic", topic_pc_pub, std::string("/pointcloud_process/proc"));
    rosHandle.param("pointcloud_process_cor_pub_topic", topic_cor_pub, std::string("/pointcloud_process/corners"));


	float caliboard_width;
	float caliboard_height;
	rosHandle.param("caliboard_width", caliboard_width, 800.0f);
	rosHandle.param("caliboard_height", caliboard_height, 600.0f);
	caliboard_width /= 1000;
	caliboard_height /= 1000;


	PointCloudSubscriber pc_sub(rosHandle, 10);
	PointCloudPublisher pc_pub(rosHandle);
	pc_sub.addTopic(topic_pc_sub);
	pc_pub.addTopic(topic_pc_pub, 10);
	pointcloud2_opr::PointCloud2Proc<pcl::PointXYZI> pc_process(true); // Remove Origin Point Published by livox_ros_driver2
	PointcloudFilterReconfigure filterRecfg;
    RQTConfig rqtCfg;
	RvizDraw rviz_draw("pointcloud_process_node/rviz_draw", frame_id);

	CornersPublisher cor_pub(rosHandle);
	cor_pub.addTopic(topic_cor_pub, 10);

	ros::Rate rate(30);

	while(ros::ok())
	{	
		ros::spinOnce();

		auto rcv_pc_packet = pc_sub.getPointCloud(topic_pc_sub);
		if(!rcv_pc_packet) {
			// ROS_INFO("Waiting For Point Cloud Subscribe\n");
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			continue;
		}

		auto rcv_pc = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>(rcv_pc_packet.cloud);

		pc_process.setCloud(rcv_pc);
		// ROS_INFO("%ld\n", pointcloud_SUB_PUB.getPointcloudXYZI()->size());
		rqtCfg.TransformFilterConfig = filterRecfg.getTransformConfigure();
		float center_x = rqtCfg.TransformFilterConfig.center_x;
		float center_y = rqtCfg.TransformFilterConfig.center_y;
		float center_z = rqtCfg.TransformFilterConfig.center_z;
		float length_x = rqtCfg.TransformFilterConfig.length_x;
		float length_y = rqtCfg.TransformFilterConfig.length_y;
		float length_z = rqtCfg.TransformFilterConfig.length_z;
		float rotate_x = rqtCfg.TransformFilterConfig.rotate_x;
		float rotate_y = rqtCfg.TransformFilterConfig.rotate_y;
		float rotate_z = rqtCfg.TransformFilterConfig.rotate_z;
		pc_process.boxFilter(Eigen::Vector3f(center_x, center_y, center_z), length_x, length_y, length_z, rotate_x, rotate_y, rotate_z);

		setShareParam(rosHandle, rqtCfg);

		pc_pub.publish(topic_pc_pub, PointCloudPacket(*pc_process.getProcessedPointcloud(), frame_id, 0, rosTimeToTimestamp(ros::Time::now())));

		if(filterRecfg.isUpdated())
		{
			pcl::PointCloud<pcl::PointXYZI>::Ptr box_corners(new pcl::PointCloud<pcl::PointXYZI>);
			box_corners = pc_process.getFilterBoxCorners(Eigen::Vector3f(center_x, center_y, center_z), length_x, length_y, length_z, rotate_x, rotate_y, rotate_z);
			
			std::vector<geometry_msgs::Point> ros_box_corners;
			for (const auto& box_corner : *box_corners) 
			{
				geometry_msgs::Point ros_point;
				ros_point.x = box_corner.x;
				ros_point.y = box_corner.y;
				ros_point.z = box_corner.z;
				ros_box_corners.push_back(ros_point);
			}
			rviz_draw.addPoints("box_corners", ros_box_corners, 0.1, 1.0, 0.0, 0.0);

			std::vector<geometry_msgs::Point> line_corners;
			line_corners.assign(ros_box_corners.begin() + 0, ros_box_corners.begin() + 4);
			line_corners.emplace_back(*(ros_box_corners.begin() + 0));
			rviz_draw.addLines("box_line_min", line_corners, visualization_msgs::Marker::LINE_STRIP, 0.05, 1.0, 1.0, 0.0);
			line_corners.assign(ros_box_corners.begin() + 4, ros_box_corners.begin() + 8);
			line_corners.emplace_back(*(ros_box_corners.begin() + 4));
			rviz_draw.addLines("box_line_max", line_corners, visualization_msgs::Marker::LINE_STRIP, 0.05, 1.0, 1.0, 0.0);

			rviz_draw.addLine("box_line_middle1", ros_box_corners.at(0), ros_box_corners.at(0 + 4), 0.05, 1.0, 1.0, 0.0);
			rviz_draw.addLine("box_line_middle2", ros_box_corners.at(1), ros_box_corners.at(1 + 4), 0.05, 1.0, 1.0, 0.0);
			rviz_draw.addLine("box_line_middle3", ros_box_corners.at(2), ros_box_corners.at(2 + 4), 0.05, 1.0, 1.0, 0.0);
			rviz_draw.addLine("box_line_middle4", ros_box_corners.at(3), ros_box_corners.at(3 + 4), 0.05, 1.0, 1.0, 0.0);
		}
		
		// Detect caliboard corners
		pcl::PointCloud<pcl::PointXYZI>::Ptr corners;
		corners = pc_process.extractNearestRectangleCorners(false, PointCloud2Proc<pcl::PointXYZI>::OptimizationMethod::None, caliboard_width, caliboard_height, 0.05);
		CalTool::sortPointByNormalWorldFrame<pcl::PointXYZI>(corners, pc_process.getPlaneNormals());

		geometry_msgs::PolygonStamped ros_corners;
		for (const auto& corner : *corners) 
		{
			geometry_msgs::Point32 ros_point;
			ros_point.x = corner.x;
			ros_point.y = corner.y;
			ros_point.z = corner.z;
			ros_corners.polygon.points.push_back(ros_point);
    	}

		std_msgs::Header header;
		cor_pub.publish(topic_cor_pub, CornersPacket(ros_corners, frame_id, 0, rosTimeToTimestamp(ros::Time::now())));
		

		if(corners->size() == 0)
		{
			// rviz_draw.deleteAllObject();
			rviz_draw.deleteObject("corners");

			rviz_draw.deleteObject("corner_1");
			rviz_draw.deleteObject("corner_2");
			rviz_draw.deleteObject("corner_3");
			rviz_draw.deleteObject("corner_4");

			rviz_draw.deleteObject("plane_normals");

			rviz_draw.deleteObject("rect_lines");
		}
		else
		{
			rviz_draw.addPoints("corners", CBridge::rosPoint32ToPointMulti(ros_corners.polygon.points), 0.03, 1.0, 0.0, 0.0);

			rviz_draw.addText("corner_1", CBridge::rosPoint32ToPoint(ros_corners.polygon.points.at(0)), "1", 0.3, 0.0, 1.0, 0.0);
			rviz_draw.addText("corner_2", CBridge::rosPoint32ToPoint(ros_corners.polygon.points.at(1)), "2", 0.3, 0.0, 1.0, 0.0);
			rviz_draw.addText("corner_3", CBridge::rosPoint32ToPoint(ros_corners.polygon.points.at(2)), "3", 0.3, 0.0, 1.0, 0.0);
			rviz_draw.addText("corner_4", CBridge::rosPoint32ToPoint(ros_corners.polygon.points.at(3)), "4", 0.3, 0.0, 1.0, 0.0);

			Eigen::Vector3f plane_normals = pc_process.getPlaneNormals().normalized();
			rviz_draw.addArrow("plane_normals", 
									(ros_corners.polygon.points[0].x + ros_corners.polygon.points[1].x + ros_corners.polygon.points[2].x + ros_corners.polygon.points[3].x) / 4,
									(ros_corners.polygon.points[0].y + ros_corners.polygon.points[1].y + ros_corners.polygon.points[2].y + ros_corners.polygon.points[3].y) / 4,
									(ros_corners.polygon.points[0].z + ros_corners.polygon.points[1].z + ros_corners.polygon.points[2].z + ros_corners.polygon.points[3].z) / 4,
									plane_normals, 0.03, 0.06, 0.06, 0.0, 1.0, 0.0);

			ros_corners.polygon.points.push_back(ros_corners.polygon.points[0]);
			rviz_draw.addLines("rect_lines", CBridge::rosPoint32ToPointMulti(ros_corners.polygon.points), 4, 0.01, 0.0, 1.0, 0.0);
		}

		rviz_draw.publish();

		rate.sleep();
	}
	
	return 0;
}
