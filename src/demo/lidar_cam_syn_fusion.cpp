#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <map>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>

#include "image_opr/image_process.h"
#include "image_opr/image_draw.h"

#include "pointcloud2_opr/point_cloud_process.h"

#include "tools/dynamic_reconfigure.h"
#include "tools/rviz_draw.h"
#include "tools/ros_topic_manager.h"
#include "tools/conversion_bridge.h"
#include "tools/file_operator.h"

using namespace lcr_cal;
using namespace lcr_cal::pointcloud2_opr;
using namespace lcr_cal::image_opr;

std::map<ros::Time, sensor_msgs::Image> image_cache;
std::map<ros::Time, sensor_msgs::PointCloud2> cloud_cache;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    image_cache[msg->header.stamp] = *msg;
}

void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
    try {
        // 将CompressedImage消息转换为OpenCV图像格式
        cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

        // 使用cv_bridge将OpenCV图像转换为ROS图像消息
        sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(msg->header, "bgr8", image).toImageMsg();

        // 发布转换后的图像
        image_cache[out_msg->header.stamp] = *out_msg;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->format.c_str());
    }
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    cloud_cache[msg->header.stamp] = *msg;
}

cv::Mat cameraMatrix, distCoeffs, newCameraMatrix, newDistCoeffes;
PointCloud2Proc<pcl::PointXYZI> pc_process(true);
Eigen::Matrix3f R = Eigen::Matrix3f::Identity(); 
Eigen::Vector3f t(0.0f, 0.0f, 0.0f); 
void fusionProcessPub(sensor_msgs::Image img_msg, sensor_msgs::PointCloud2 pc_msg, image_transport::Publisher& lidar_cam_fusion_image) {
    cv::Mat cv_image;
    try {
        // 将ROS图像消息转换为cv::Mat
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        cv_image = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(pc_msg, *cloud);

    pc_process.setCloud(cloud);
    pc_process.transform(R, t);
    pc_process.scaleTo(1000.0f);
    pc_process.PassThroughFilter("z", 1000.0, FLT_MAX);

    ImageDraw image_draw(1, 1, 1, 1, cameraMatrix, distCoeffs);
    std::vector<cv::Point2f> imagePoints;
    image_draw.projectPointsToImage(*pc_process.getProcessedPointcloud(), imagePoints);
    image_draw.drawPointsOnImageIntensity(*pc_process.getProcessedPointcloud(), imagePoints, cv_image);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(img_msg.header, "bgr8", cv_image).toImageMsg();
    lidar_cam_fusion_image.publish(msg);

}

bool culmu = true;
void publishSyncMessages(image_transport::Publisher& img_pub, ros::Publisher& cloud_pub, image_transport::Publisher& lidar_cam_fusion_image) {
    
    if (culmu && (image_cache.size() < 20 || cloud_cache.size() < 20)) return;
    culmu = false;
    while (!image_cache.empty() && !cloud_cache.empty()) {
        // Manually find the minimum time stamp among the first elements of the caches
        ros::Time min_time = std::min({image_cache.begin()->first, cloud_cache.begin()->first});
        ROS_WARN("Diff timestamp: %f", (image_cache.begin()->first - cloud_cache.begin()->first).toSec());
        // Check if all messages are within the time window
            
            auto max_time = std::max({image_cache.rbegin()->first.toSec(), cloud_cache.rbegin()->first.toSec()});
            if (std::abs((image_cache.begin()->first - cloud_cache.begin()->first).toSec()) <= 0.05) {
                img_pub.publish(image_cache.begin()->second);
                cloud_pub.publish(cloud_cache.begin()->second);
                fusionProcessPub(image_cache.begin()->second, cloud_cache.begin()->second, lidar_cam_fusion_image);
                // Remove published messages from cache
                image_cache.erase(image_cache.begin());
                cloud_cache.erase(cloud_cache.begin());
                return;
            }
            else {
                if (image_cache.begin()->first == min_time) {
                    image_cache.erase(min_time);
                }
                if (cloud_cache.begin()->first == min_time) {
                    cloud_cache.erase(min_time);
                }
            }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sync_republisher");
    ros::NodeHandle nh;


    std::string frame_id;

    std::string topic_pc_sub;
    std::string topic_pc_proc_sub;
    std::string topic_pc_pub;

    std::string topic_img_sub;
    std::string topic_img_pub;

	std::string topic_pc_corners_sub;
    std::string topic_img_corners_sub;
    std::string topic_corners_pub;

    std::string topic_command_sub;
    std::string topic_command_pub;

    nh.param("frame_id", frame_id, std::string("rslidar"));

	nh.param("pointcloud_process_pc_sub_topic", topic_pc_sub, std::string("/rslidar_points"));

	nh.param("image_process_img_sub_topic", topic_img_sub, std::string("/hikcamera/image_0/compressed"));
    nh.param("calibration_img_pub_topic", topic_img_pub, std::string("/lcr_cal/image/proc"));

    nh.param("pointcloud_process_cor_pub_topic", topic_pc_corners_sub, std::string("/pointcloud_process/corners"));
    nh.param("image_process_cor_pub_topic", topic_img_corners_sub, std::string("/image_process/corners"));

    nh.param("calibration_command_sub_topic", topic_command_sub, std::string("/lcr_cal/command_controller"));
    nh.param("calibration_command_pub_topic", topic_command_pub, std::string("/lcr_cal/command_cal_node"));


    std::string packagePath;
    if (!nh.getParam("package_path", packagePath)) {
        ROS_ERROR("Failed to get 'package_path' from the parameter server.");
        return 1;
    }
    std::cout << "package_path: " << packagePath << std::endl;
    int chdir_flags = chdir(packagePath.c_str());
    if (chdir_flags != 0) {
        perror("Error changing directory");  
        return 1;  
    }
    
    cv::String yaml_path;
    nh.param("yaml_path", yaml_path, cv::String("~/"));

    cv::String intrinsicsPath = yaml_path + "camera-intrinsics.yaml";
    boost::filesystem::path p = boost::filesystem::current_path();  
    std::cout << "Current working directory: " << p << std::endl;
    std::cout << intrinsicsPath << std::endl;
    cv::FileStorage fs(intrinsicsPath, cv::FileStorage::READ);
    int image_width{0}, image_height{0};
    fs["imageWidth"] >> image_width;
    fs["imageHeight"] >> image_height;

    cv::Size image_size = cv::Size(image_width, image_height);

    
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    fs["newCameraMatrixAlpha0"] >> newCameraMatrix;
    fs["newDistCoeffsAlpha0"] >> newDistCoeffes;
    fs.release();
    std::cout << cameraMatrix << std::endl;
    std::cout << distCoeffs << std::endl;
    std::cout << image_size << std::endl;

    std::string cornerset_csv_path;
    std::string error_anaylysis_csv_path;
    std::string extrinsics_path;
    nh.param("pointset_save_path", cornerset_csv_path, std::string("src/lcr_cal/data/point_set.csv"));
    nh.param("error_analysis_save_path", error_anaylysis_csv_path, std::string("src/lcr_cal/data/border_error_anaylysis.csv"));
    nh.param("extrinsics_save_path", extrinsics_path, std::string("src/lcr_cal/config/extrinsics.yaml"));

    YamlOperator yaml_operator(extrinsics_path);

    yaml_operator.readExtrinsicsFromYaml(R, t);


    // ros::Publisher pub_image = nh.advertise<sensor_msgs::Image>("synced_image", 1);
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_image = it.advertise("synced_compressed_image", 1);
    image_transport::Publisher lidar_cam_fusion_image = it.advertise("lidar_cam_fusion_image", 1);
    ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("synced_cloud", 1);

    // ros::Subscriber sub_image = nh.subscribe("/hikcamera/image_0", 1, imageCallback);
    ros::Subscriber sub_compressed_image = nh.subscribe(topic_img_sub, 1, compressedImageCallback);
    ros::Subscriber sub_cloud = nh.subscribe(topic_pc_sub, 1, cloudCallback);

    ros::Rate loop_rate(10); // 调整为所需的频率
    while (ros::ok()) {
        ros::spinOnce();
        publishSyncMessages(pub_image, pub_cloud, lidar_cam_fusion_image);
        loop_rate.sleep();
    }

    return 0;
}