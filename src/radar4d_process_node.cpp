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

#include "tools/dynamic_reconfigure.h"
#include "tools/rviz_draw.h"
#include "tools/ros_topic_manager.h"
#include "tools/conversion_bridge.h"

#include "calibration_tool.h"

using namespace lcr_cal;


void setShareParam(ros::NodeHandle nh, RQTConfig rqt_config)
{
	nh.setParam("/shared_parameter/filter_distance", rqt_config.Radar4DFilterConfig.filter_distance);
	nh.setParam("/shared_parameter/filter_snr", rqt_config.Radar4DFilterConfig.filter_snr);
}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "radar4d_process_node");
	ros::NodeHandle rosHandle;


	std::string frame_id;
	std::string topic_rd_sub;
    std::string topic_rd_pub;
    std::string topic_cen_pub;
	rosHandle.param("frame_id", frame_id, std::string("car"));
	rosHandle.param("radar4d_process_pc_sub_topic", topic_rd_sub, std::string("/Target_Radar_1_extreme"));
    rosHandle.param("radar4d_process_pc_pub_topic", topic_rd_pub, std::string("/radar4d_process/proc"));
    rosHandle.param("radar4d_process_cen_pub_topic", topic_cen_pub, std::string("/radar4d_process/center"));


	float caliboard_width;
	float caliboard_height;
	rosHandle.param("caliboard_width", caliboard_width, 1000.0f);
	rosHandle.param("caliboard_height", caliboard_height, 800.0f);
	caliboard_width /= 1000;
	caliboard_height /= 1000;


	Radar4DSubscriber rd_sub(rosHandle, 10);
	Radar4DPublisher rd_pub(rosHandle);
	rd_sub.addTopic(topic_rd_sub);
	rd_pub.addTopic(topic_rd_pub, 10);
	
	Radar4DFilterReconfigure radarRecfg;
    RQTConfig rqtCfg;
	RvizDraw rviz_draw("radar4d_process_node/rviz_draw", frame_id);

	PointsetPublisher pts_pub(rosHandle);
	pts_pub.addTopic(topic_cen_pub, 10);

	ros::Rate rate(30);

	while(ros::ok())
	{	
		ros::spinOnce();

		Radar4DPacket rcv_rd_packet = rd_sub.getRadar4DPacket(topic_rd_sub);
		if(!rcv_rd_packet) {
			// ROS_INFO("Waiting For Point Cloud Subscribe\n");
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			continue;
		}

		
		rqtCfg.Radar4DFilterConfig = radarRecfg.getRadar4DFilterConfigure();
		float filter_distance = rqtCfg.Radar4DFilterConfig.filter_distance;
		float filter_snr = rqtCfg.Radar4DFilterConfig.filter_snr;
		int max_snr_points_num = rqtCfg.Radar4DFilterConfig.max_snr_points_num;
		float valid_distance_threshold = rqtCfg.Radar4DFilterConfig.valid_distance_threshold;

		std::vector<lcr_cal::Radar4DPoint> filtered_points;
        for (const auto& point : rcv_rd_packet.points) {
            float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            if (distance <= filter_distance && point.snr > filter_snr) {
                filtered_points.push_back(point);
            }
			// std::cout << "distance: " << distance << " vs " << filter_distance << " || point.snr: " << point.snr << " vs " << filter_snr << std::endl;
        }

        // 检查是否有足够的点通过过滤
        if (filtered_points.empty()) {
            ROS_WARN("No points passed the filtering criteria.");
			// rviz_draw.deleteAllObject();
			rviz_draw.deleteObject("center");
			rviz_draw.publish();
            continue;
        }

        // 按 SNR 降序排序，并选择 SNR 最大的 max_snr_points_num 个点
        std::sort(filtered_points.begin(), filtered_points.end(), [](const lcr_cal::Radar4DPoint& a, const lcr_cal::Radar4DPoint& b) {
            return a.snr > b.snr;
        });
        int points_to_consider = std::min(static_cast<int>(filtered_points.size()), max_snr_points_num);
        std::vector<lcr_cal::Radar4DPoint> top_points(filtered_points.begin(), filtered_points.begin() + points_to_consider);

        // 判断点之间的两两距离是否小于阈值
        bool valid = true;
        for (size_t i = 0; i < top_points.size(); ++i) {
            for (size_t j = i + 1; j < top_points.size(); ++j) {
                float dx = top_points[i].x - top_points[j].x;
                float dy = top_points[i].y - top_points[j].y;
                float dz = top_points[i].z - top_points[j].z;
                float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (distance > valid_distance_threshold) {
                    valid = false;
                    break;
                }
            }
            if (!valid) break;
        }

        if (max_snr_points_num < 2) {
            valid = true;
        }

        // 如果点无效，跳过本次处理
        if (!valid) {
            ROS_WARN("Top points are too far apart, skipping.");
			// rviz_draw.deleteAllObject();
			rviz_draw.deleteObject("center");
			rviz_draw.publish();
            continue;
        }

        // 计算位置平均值
        float avg_x = 0.0, avg_y = 0.0, avg_z = 0.0;
        for (const auto& point : top_points) {
            avg_x += point.x;
            avg_y += point.y;
            avg_z += point.z;
        }
        avg_x /= top_points.size();
        avg_y /= top_points.size();
        avg_z /= top_points.size();

        // 构造输出的 geometry_msgs::PolygonStamped
        geometry_msgs::PolygonStamped ros_center;
        geometry_msgs::Point32 ros_point;
        ros_point.x = avg_x;
        ros_point.y = avg_y;
        ros_point.z = avg_z;
        ros_center.polygon.points.push_back(ros_point);

		std_msgs::Header header;
		pts_pub.publish(topic_cen_pub, PointsPacket(ros_center, frame_id, 0, rosTimeToTimestamp(ros::Time::now())));
		
		rviz_draw.addPoints("center", CBridge::rosPoint32ToPointMulti(ros_center.polygon.points), 0.08, 1.0, 0.0, 0.0);

		rviz_draw.publish();

		rate.sleep();
	}
	
	return 0;
}
