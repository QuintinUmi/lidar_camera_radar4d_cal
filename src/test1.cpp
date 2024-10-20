#include "tools/ros_topic_manager.h"
#include "pointcloud2_opr/point_cloud_process.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <Eigen/Geometry>

#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"    

#include "calibration_tool.h"

using namespace lidar_camera_cal;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test1");
    ros::NodeHandle nh;

    ImageSubscriber sub(nh, 10);

    sub.addTopic("/hikcamera/image_0/compressed");
    sub.addTopic("/hikcamera/image_1/compressed");
    cv::namedWindow("test1", cv::WINDOW_NORMAL);

    ImagePublisher pub(nh);
    pub.addTopic("/hikcamera/image_1/test", 10);

    while (ros::ok()) {
        ros::spinOnce();
        ImagePacket img_pack = sub.getImage("/hikcamera/image_1/compressed");
        if (img_pack) {
            cv::imshow("test1", *img_pack.image);
            cv::waitKey(1);
            pub.publish("/hikcamera/image_1/test", img_pack, "bgr8");
        }
        
    }

    /* code */
    return 0;
}
