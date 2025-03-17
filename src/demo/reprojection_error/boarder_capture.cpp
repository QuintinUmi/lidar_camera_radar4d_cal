#define PCL_NO_PRECOMPILE
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>


#include <Eigen/Dense>
#include <Eigen/SVD>

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
#include <memory>


#include <yaml-cpp/yaml.h>
#include "tools/file_operator.h"

#include <geometry_msgs/Point32.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>  
#include <aruco/aruco.h>  
#include "opencv2/aruco/charuco.hpp"  

#include "image_transport/image_transport.h"

#include "image_opr/image_process.h"
#include "image_opr/image_draw.h"
#include "image_opr/aruco_manager.h"

#include "pointcloud2_opr/point_cloud_process.h"

// #include "tools/conversion_bridge.h"
#include "tools/dynamic_reconfigure.h"
#include "tools/rviz_draw.h"
#include "tools/timestamp_cov.h"

#include <opencv2/opencv.hpp>

#include <Eigen/Core>


#include <opencv2/core.hpp>
#include <vector>
#include <cmath>
#include <limits>

#include <pcl/visualization/cloud_viewer.h> 

#include "pointcloud2_opr/point_cloud_process.h"

#include "tools/dynamic_reconfigure.h"
#include "tools/rviz_draw.h"
// #include "tools/ros_topic_manager.h"
#include "tools/conversion_bridge.h"
#include "tools/CommandHandler.h"

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

// namespace velodyne_ros
namespace velodyne_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        // float time;
        uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}  
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (float, intensity, intensity)
        // (float, time, time)
        (uint16_t, ring, ring)
)

// namespace pandar_ros
namespace pandar_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        double timestamp;
        uint16_t  ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(pandar_ros::Point,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (float, intensity, intensity)
        (double, timestamp, timestamp)
        (std::uint16_t, ring, ring)
)

double pointToLineDistance(const cv::Point2f& point, const cv::Point2f& line_start, const cv::Point2f& line_end) {
    // 计算直线方向向量
    cv::Point2f d = line_end - line_start;

    // 计算点到直线的距离公式
    double numerator = std::fabs(d.x * (line_start.y - point.y) - d.y * (line_start.x - point.x));
    double denominator = std::sqrt(d.x * d.x + d.y * d.y);

    return numerator / denominator;
}

double computeReprojectionError(
    const std::vector<cv::Point2f>& imagePoints,          // Lidar 点云投影到图像上的边缘点
    const std::vector<std::vector<cv::Point2f>>& p2ds,    // ArUco 检测出的边框边缘点（交点集合）
    double outlier_threshold = 10.0) {                    // 离群点阈值（默认值为 10 像素）
    
    double total_error = 0.0;
    int num_points = 0;
    int outlier_count = 0;

    // 遍历每个投影点
    for (const auto& projected_point : imagePoints) {
        double min_distance = std::numeric_limits<double>::max();

        // 遍历每条边
        for (size_t i = 0; i < p2ds.size(); ++i) {
            const auto& line_start = p2ds[i][0]; // 每条边的起点
            const auto& line_end = p2ds[i][1];   // 每条边的终点

            // 计算点到当前边的距离
            double distance = pointToLineDistance(projected_point, line_start, line_end);

            // 更新最小距离
            if (distance < min_distance) {
                min_distance = distance;
            }
        }

        // 判断是否为离群点
        if (min_distance <= outlier_threshold) {
            // 累加误差
            total_error += min_distance;
            ++num_points;
        } else {
            // 统计离群点
            ++outlier_count;
        }
    }

    // 输出离群点信息（可选）
    std::cout << "Outlier count: " << outlier_count << " / " << imagePoints.size() << std::endl;

    // 返回平均误差（如果没有有效点，则返回 -1 表示无效）
    if (num_points > 0) {
        return total_error / num_points;
    } else {
        std::cerr << "No valid points after outlier filtering!" << std::endl;
        return -1.0; // 表示无效结果
    }
}

using PointType = velodyne_ros::Point;
using PointCloudType = pcl::PointCloud<PointType>;

// 根据 ring 字段对点云分组
std::map<uint16_t, PointCloudType::Ptr> groupByRing(const PointCloudType::Ptr& input_cloud) {
    std::map<uint16_t, PointCloudType::Ptr> ring_groups;

    for (const auto& point : input_cloud->points) {
        uint16_t ring_id = point.ring; // 获取点的 ring 值
        if (ring_groups.find(ring_id) == ring_groups.end()) {
            ring_groups[ring_id] = PointCloudType::Ptr(new PointCloudType);
        }
        ring_groups[ring_id]->points.push_back(point);
    }

    return ring_groups;
}

std::vector<pcl::PointXYZI> extractEdgePoints(const std::map<uint16_t, PointCloudType::Ptr>& ring_groups) {
    std::vector<pcl::PointXYZI> edge_points;

    for (const auto& ring_pair : ring_groups) {
        const auto& cloud = ring_pair.second; // 当前线束的点云

        if (cloud->points.empty()) {
            continue;
        }

        // 按 x 值排序（可以根据需求选择 x、y 或 z 轴）
        auto compare_x = [](const PointType& p1, const PointType& p2) {
            return p1.x < p2.x;
        };
        auto min_max_x = std::minmax_element(cloud->points.begin(), cloud->points.end(), compare_x);

        // 转换起始点和末尾点为 pcl::PointXYZI 类型
        pcl::PointXYZI start_point;
        start_point.x = min_max_x.first->x;
        start_point.y = min_max_x.first->y;
        start_point.z = min_max_x.first->z;
        start_point.intensity = min_max_x.first->intensity; // 如果 PointType 没有 intensity，请适配

        pcl::PointXYZI end_point;
        end_point.x = min_max_x.second->x;
        end_point.y = min_max_x.second->y;
        end_point.z = min_max_x.second->z;
        end_point.intensity = min_max_x.second->intensity; // 如果 PointType 没有 intensity，请适配

        // 添加到返回的 vector 中
        edge_points.push_back(start_point);
        edge_points.push_back(end_point);
    }

    return edge_points;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr createEdgePointCloud(const std::vector<pcl::PointXYZI>& edge_points) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr edge_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto& point : edge_points) {
        edge_cloud->points.push_back(point);
    }
    return edge_cloud;
}

#define PI 3.14159265358979324


using namespace std;
using namespace lidar_camera_cal;
using namespace lidar_camera_cal::image_opr;

int fps(int deltaTime) 
{
    int fps = static_cast<int>(1.f / deltaTime * 1000); 
    return fps;
}

using namespace lidar_camera_cal;
using namespace lidar_camera_cal::pointcloud2_opr;


#define SHOW_DEBUG_MESSAGE false
namespace lidar_camera_cal {

    template <typename PointT>
    struct PointCloudPacket {
        pcl::PointCloud<PointT> cloud;
        std::string frame_id;
        int seq;
        uint64_t timestamp;
        bool is_valid;

        operator bool() const {
            return is_valid && !cloud.empty();
        }

        PointCloudPacket() : frame_id(""), seq(0), timestamp(0), is_valid(false) {}
        PointCloudPacket(pcl::PointCloud<PointT> cloud, const std::string& frame_id, int seq, uint64_t timestamp)
            : cloud(std::move(cloud)), frame_id(frame_id), seq(seq), timestamp(timestamp), is_valid(true) {}
    };

    template <typename PointT>
    class PointCloudSubscriber {
    public:
        PointCloudSubscriber(ros::NodeHandle& nh, size_t queue_size)
            : nh_(nh), queue_size_(queue_size) {}

        void addTopic(const std::string& topic) {
            std::lock_guard<std::mutex> lock(mutex_);
            ros::Subscriber sub = nh_.subscribe<sensor_msgs::PointCloud2>(
                topic, 1,
                boost::bind(&PointCloudSubscriber::pointCloudCallback, this, _1, topic)
            );
            subscribers_.emplace_back(sub);
            if (SHOW_DEBUG_MESSAGE) ROS_INFO("Subscribed to PointCloud2 topic: %s", topic.c_str());
        }

        void addTopics(const std::vector<std::string>& topics) {
            for (const auto& topic : topics) {
                addTopic(topic);
            }
        }

        PointCloudPacket<PointT> getPointCloud(const std::string& topic) {
            std::lock_guard<std::mutex> lock(mutex_);
            auto it = cloud_queues_.find(topic);
            if (it != cloud_queues_.end() && !it->second.empty()) {
                PointCloudPacket<PointT> packet = std::move(it->second.front());
                it->second.pop();
                return packet;
            }
            return PointCloudPacket<PointT>();  // 返回无效的点云包
        }

    private:
        void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg, const std::string& topic) {
            pcl::PointCloud<PointT> cloud;
            pcl::fromROSMsg(*msg, cloud);  // 将 ROS 点云消息转换为 PCL 点云格式
            std::lock_guard<std::mutex> lock(mutex_);
            auto& queue = cloud_queues_[topic];
            if (queue.size() >= queue_size_) {
                queue.pop();
            }
            queue.emplace(std::move(cloud), msg->header.frame_id, msg->header.seq, rosTimeToTimestamp(msg->header.stamp));
        }

        ros::NodeHandle nh_;
        std::vector<ros::Subscriber> subscribers_;
        std::map<std::string, std::queue<PointCloudPacket<PointT>>> cloud_queues_;
        std::mutex mutex_;
        size_t queue_size_;
    };

    template <typename PointT>
    class PointCloudPublisher {
    public:
        PointCloudPublisher(ros::NodeHandle& nh) : nh_(nh) {}

        void addTopic(const std::string& topic, size_t queue_size) {
            pub_[topic] = nh_.advertise<sensor_msgs::PointCloud2>(topic, queue_size);
            if (SHOW_DEBUG_MESSAGE) ROS_INFO("PointCloud2 publisher added for topic: %s", topic.c_str());
        }

        void addTopics(const std::vector<std::string>& topics, size_t queue_size) {
            for (const std::string& topic : topics) {
                addTopic(topic, queue_size);
            }
        }

        void publish(const std::string& topic, const PointCloudPacket<PointT>& cloud_packet) {
            auto it = pub_.find(topic);
            if (it != pub_.end() && cloud_packet && cloud_packet.is_valid) {
                sensor_msgs::PointCloud2 msg;
                pcl::toROSMsg(cloud_packet.cloud, msg);
                msg.header.stamp = timestampToRosTime(cloud_packet.timestamp);
                msg.header.frame_id = cloud_packet.frame_id;
                it->second.publish(msg);
                if (SHOW_DEBUG_MESSAGE) ROS_INFO("Published PointCloud2 to topic: %s", topic.c_str());
            } else {
                ROS_WARN("No publisher available for topic: %s", topic.c_str());
            }
        }

    private:
        ros::NodeHandle nh_;
        std::map<std::string, ros::Publisher> pub_;
    };

    struct ImagePacket {
        std::shared_ptr<cv::Mat> image;
        std::string frame_id;
        int seq;
        uint64_t timestamp;
        bool is_valid;

        operator bool() const {
            return is_valid && image && !image->empty();
        }

        ImagePacket() : image(std::make_shared<cv::Mat>()), frame_id(""), seq(0), timestamp(0), is_valid(false) {}
        ImagePacket(std::shared_ptr<cv::Mat> image, const std::string& frame_id, int seq, uint64_t timestamp)
            : image(std::move(image)), frame_id(frame_id), seq(seq), timestamp(timestamp), is_valid(true) {}
    };

    class ImageSubscriber {
    public:
        ImageSubscriber(ros::NodeHandle& nh, size_t queue_size)
            : nh_(nh), queue_size_(queue_size) {}

        void addTopic(const std::string& topic) {
            std::lock_guard<std::mutex> lock(mutex_);
            if (topic.find("compressed") != std::string::npos) {
                ros::Subscriber sub = nh_.subscribe<sensor_msgs::CompressedImage>(
                    topic, 1,
                    boost::bind(&ImageSubscriber::compressedImageCallback, this, _1, topic)
                );
                subscribers_.emplace_back(sub);
            } else {
                ros::Subscriber sub = nh_.subscribe<sensor_msgs::Image>(
                    topic, 1,
                    boost::bind(&ImageSubscriber::imageCallback, this, _1, topic)
                );
                subscribers_.emplace_back(sub);
            }
            if (SHOW_DEBUG_MESSAGE) ROS_INFO("Subscribed to %s image topic: %s", (topic.find("compressed") != std::string::npos ? "compressed" : "regular"), topic.c_str());
        }

        void addTopics(const std::vector<std::string>& topics) {
            for (const auto& topic : topics) {
                addTopic(topic);
            }
        }

        ImagePacket getImage(const std::string& topic) {
            std::lock_guard<std::mutex> lock(mutex_);
            auto it = image_queues_.find(topic);
            if (it != image_queues_.end() && !it->second.empty()) {
                ImagePacket packet = std::move(it->second.front());
                it->second.pop();
                return packet;
            }
            return ImagePacket();  // 返回无效的图像包
        }

    private:
        template<typename T>
        void processImage(const T& msg, const std::string& encoding, const std::string& topic) {
            try {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, encoding);
                std::lock_guard<std::mutex> lock(mutex_);
                auto& queue = image_queues_[topic];
                if (queue.size() >= queue_size_) {
                    queue.pop();
                }
                queue.emplace(std::make_shared<cv::Mat>(cv_ptr->image), cv_ptr->header.frame_id, cv_ptr->header.seq, rosTimeToTimestamp(cv_ptr->header.stamp));
            } catch (const cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }
        }

        void imageCallback(const sensor_msgs::ImageConstPtr& msg, const std::string& topic) {
            processImage(msg, sensor_msgs::image_encodings::BGR8, topic);
        }

        void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr& msg, const std::string& topic) {
            processImage(msg, sensor_msgs::image_encodings::BGR8, topic);
        }

        ros::NodeHandle nh_;
        std::vector<ros::Subscriber> subscribers_;
        std::map<std::string, std::queue<ImagePacket>> image_queues_;
        std::mutex mutex_;
        size_t queue_size_;
    };

    class ImagePublisher {
    public:
        ImagePublisher(ros::NodeHandle& nh) : it_(nh) {}

        void addTopic(const std::string& topic, size_t queue_size) {
            pub_[topic] = std::make_shared<image_transport::Publisher>(it_.advertise(topic, queue_size));
            if (SHOW_DEBUG_MESSAGE) ROS_INFO("Image transport publisher added for topic: %s", topic.c_str());
        }

        void addTopics(const std::vector<std::string>& topics, size_t queue_size) {
            for (const std::string& topic : topics) {
                addTopic(topic, queue_size);
            }
        }

        void publish(const std::string& topic, const ImagePacket& image_packet, const std::string& encoding = "bgr8") {
            auto it = pub_.find(topic);
            if (it != pub_.end() && image_packet && image_packet.is_valid) {
                cv_bridge::CvImage cv_image;
                cv_image.header.stamp = timestampToRosTime(image_packet.timestamp);
                cv_image.header.frame_id = image_packet.frame_id;
                cv_image.encoding = encoding;
                cv_image.image = *image_packet.image;
                sensor_msgs::ImagePtr msg = cv_image.toImageMsg();
                it->second->publish(msg);
                if (SHOW_DEBUG_MESSAGE) ROS_INFO("Published image to topic: %s", topic.c_str());
            } else {
                ROS_WARN("No publisher available for topic: %s", topic.c_str());
            }
        }

    private:
        image_transport::ImageTransport it_;
        std::map<std::string, std::shared_ptr<image_transport::Publisher>> pub_;
    };
}


// 读取历史重投影误差数据
std::vector<double> loadReprojectionErrorData(const std::string& filename) {
    std::vector<double> errors;

    std::ifstream file(filename);
    if (file.is_open()) {
        double value;
        while (file >> value) {
            errors.push_back(value);
        }
        file.close();
    } else {
        std::cout << "No existing data found. Starting fresh..." << std::endl;
    }

    return errors;
}

// 保存新的重投影误差数据到文件
void saveReprojectionErrorData(const std::string& filename, const std::vector<double>& errors) {
    std::ofstream file(filename);
    if (file.is_open()) {
        for (const auto& error : errors) {
            file << error << std::endl;
        }
        file.close();
    } else {
        std::cerr << "Error: Could not open file for writing." << std::endl;
    }
}

// 计算平均误差
double computeAverageReprojectionError(const std::vector<double>& errors) {
    if (errors.empty()) {
        return 0.0;
    }

    double sum = std::accumulate(errors.begin(), errors.end(), 0.0);
    return sum / errors.size();
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "image_process_node");
    ros::NodeHandle rosHandle;

    bool undistort;
    int undistort_mode;
    std::string frame_id;
    std::string topic_img_sub;
    std::string topic_img_pub;
    std::string topic_cor_pub;
    std::string topic_trans_pub;
    rosHandle.param("undistort", undistort, false);
    rosHandle.param("undistort_mode", undistort_mode, 0);
    rosHandle.param("frame_id", frame_id, std::string("velodyne"));
	rosHandle.param("image_process_img_sub_topic", topic_img_sub, std::string("/hikcamera/image_0/compressed"));
    rosHandle.param("image_process_img_pub_topic", topic_img_pub, std::string("/image_process/proc"));
    rosHandle.param("image_process_cor_pub_topic", topic_cor_pub, std::string("/image_process/corners"));
    rosHandle.param("image_process_trans_pub_topic", topic_trans_pub, std::string("/image_process/trans"));

	std::string topic_pc_sub;
    std::string topic_pc_pub;
	rosHandle.param("pointcloud_process_pc_sub_topic", topic_pc_sub, std::string("/rslidar_points"));
    rosHandle.param("pointcloud_process_pc_pub_topic", topic_pc_pub, std::string("/pointcloud_process/proc"));


    std::string package_path;
    if (!rosHandle.getParam("package_path", package_path)) {
        ROS_ERROR("Failed to get 'package_path' from the parameter server.");
        return 1;
    }
    std::cout << "package_path: " << package_path << std::endl;
    int chdir_flags = chdir(package_path.c_str());
    if (chdir_flags != 0) {
        perror("Error changing directory");  
        return 1;  
    }

    
    cv::String yaml_path;

    rosHandle.param("yaml_path", yaml_path, cv::String("~/"));

    int dictionaryName;
    vector<int> ids;
    bool use_center_aruco;
    int center_id;
    int markerSize;
    vector<float> arucoRealLength;

    rosHandle.param("dictionary_name", dictionaryName, 10);
    rosHandle.param("selected_ids", ids, vector<int>{});
    rosHandle.param("use_center_aruco", use_center_aruco, false);
    rosHandle.param("center_id", center_id, 12);
    rosHandle.param("aruco_marker_size", markerSize, 500);
    rosHandle.param("aruco_real_length", arucoRealLength, vector<float>{1.0});

    float caliboard_width;
    float caliboard_height;

    rosHandle.param("caliboard_width", caliboard_width, static_cast<float>(12.0));
    rosHandle.param("caliboard_height", caliboard_height, static_cast<float>(12.0));

    cv::String intrinsicsPath = yaml_path + "camera-intrinsics.yaml";
    boost::filesystem::path p = boost::filesystem::current_path();  
    std::cout << "Current working directory: " << p << std::endl;
    std::cout << intrinsicsPath << std::endl;
    cv::FileStorage fs(intrinsicsPath, cv::FileStorage::READ);
    int image_width{0}, image_height{0};
    fs["imageWidth"] >> image_width;
    fs["imageHeight"] >> image_height;

    cv::Size image_size = cv::Size(image_width, image_height);

    std::string matrix_alpha = "newCameraMatrixAlpha";
    std::string distcoeffs_alpha = "newDistCoeffsAlpha";
    if (undistort_mode == 0) {
        matrix_alpha += "0";
        distcoeffs_alpha += "0";
    } else {
        matrix_alpha += "1";
        distcoeffs_alpha += "1";
    }
    cv::Mat cameraMatrix, distCoeffs, newCameraMatrix, newDistCoeffes;
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    fs[matrix_alpha] >> newCameraMatrix;
    fs[distcoeffs_alpha] >> newDistCoeffes;
    fs.release();
    std::cout << cameraMatrix << std::endl;
    std::cout << distCoeffs << std::endl;
    std::cout << newCameraMatrix << std::endl;
    std::cout << newDistCoeffes << std::endl;
    std::cout << image_size << std::endl;

    cv::Mat map1, map2;
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), newCameraMatrix, image_size, CV_32FC2, map1, map2);


    cv::aruco::DICT_6X6_1000;
    ArucoManager arucos;
    if (undistort) {
        arucos = ArucoManager(dictionaryName, ids, arucoRealLength, newCameraMatrix, newDistCoeffes);
    } else {
        arucos = ArucoManager(dictionaryName, ids, arucoRealLength, cameraMatrix, distCoeffs);
    }
    arucos.setDetectionParameters(1);
    arucos.create();

    ImageDraw image_draw;
    if (undistort) {
        image_draw = ImageDraw(arucoRealLength[0], 1.0, 1.0, 1.0, newCameraMatrix, newDistCoeffes);
    } else {
        image_draw = ImageDraw(arucoRealLength[0], 1.0, 1.0, 1.0, cameraMatrix, distCoeffs);
    }

    
    ImageSubscriber image_sub(rosHandle, 5);
    ImagePublisher image_pub(rosHandle);
    image_sub.addTopic(topic_img_sub);
    image_pub.addTopic(topic_img_pub, 5);

    // CornersPublisher corners_pub(rosHandle);
    // corners_pub.addTopic(topic_cor_pub, 5);

    // TransformPublisher transform_pub(rosHandle);
    // transform_pub.addTopic(topic_trans_pub, 2);

	rosHandle.param("caliboard_width", caliboard_width, 800.0f);
	rosHandle.param("caliboard_height", caliboard_height, 600.0f);
	caliboard_width /= 1000;
	caliboard_height /= 1000;


	PointCloudSubscriber<velodyne_ros::Point> pc_sub(rosHandle, 10);
	// PointCloudPublisher<velodyne_ros::Point> pc_pub(rosHandle);
	PointCloudPublisher<pcl::PointXYZI> pc_pub(rosHandle);
	pc_sub.addTopic(topic_pc_sub);
	pc_pub.addTopic(topic_pc_pub, 10);
	pointcloud2_opr::PointCloud2Proc<velodyne_ros::Point> pc_process(true); // Remove Origin Point Published by livox_ros_driver2
	PointcloudFilterReconfigure filterRecfg;
    RQTConfig rqtCfg;

	// CornersPublisher cor_pub(rosHandle);
	// cor_pub.addTopic(topic_cor_pub, 10);

    std::string extrinsics_path;
    rosHandle.param("extrinsics_save_path", extrinsics_path, std::string("src/lidar_camera_cal/config/extrinsics.yaml"));

    YamlOperator yaml_operator(extrinsics_path);

    Eigen::Matrix3f R = Eigen::Matrix3f::Identity(); 
    Eigen::Vector3f t(0.0f, 0.0f, 0.0f); 

    yaml_operator.readExtrinsicsFromYaml(R, t);

    RvizDraw rviz_draw("pointcloud_process_node/rviz_draw", frame_id);


    std::string topic_command_sub;
    std::string topic_command_pub;
    rosHandle.param("calibration_command_sub_topic", topic_command_sub, std::string("/lidar_camera_cal/command_controller"));
    rosHandle.param("calibration_command_pub_topic", topic_command_pub, std::string("/lidar_camera_cal/command_cal_node"));
    CommandHandler command_handler(rosHandle, topic_command_sub, topic_command_pub);


    ros::Rate rate(50);


    while(ros::ok())
    {
        ros::spinOnce();

        auto rcv_pc_packet = pc_sub.getPointCloud(topic_pc_sub);
		if(!rcv_pc_packet) {
			// ROS_INFO("Waiting For Point Cloud Subscribe\n");
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			continue;
		}

        auto rcv_pc = pcl::make_shared<pcl::PointCloud<velodyne_ros::Point>> (rcv_pc_packet.cloud);

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

		if(filterRecfg.isUpdated())
		{
			pcl::PointCloud<PointType>::Ptr box_corners(new pcl::PointCloud<PointType>);
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

        rviz_draw.publish();

        auto rcv_image_packet = image_sub.getImage(topic_img_sub);

        if(!rcv_image_packet) {
            ROS_INFO("Waiting For Image Subscribe\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        rcv_image_packet.frame_id = frame_id;
        // ROS_WARN("Img Rcv t: %ld", rcv_image_packet.timestamp);

        cv::Mat proc_image;
        if (undistort) {
            cv::remap(*rcv_image_packet.image, proc_image, map1, map2, cv::INTER_LINEAR);
        } else {
            proc_image = *rcv_image_packet.image;
        }     

        std::vector<cv::Vec3d> rvecs; 
        std::vector<cv::Vec3d> tvecs;
        pcl::Indices detectedIds;
        arucos.extCalibMultipulArucos(proc_image, rvecs, tvecs, detectedIds);
        if(detectedIds.size() == 0) {
            ROS_INFO("Not Marker Detected \n");
            image_pub.publish(topic_img_pub, rcv_image_packet);
            continue;
        }

        cv::Vec3d rvec;
        cv::Vec3d tvec;
        ImageProc::estimateAveragePose(rvecs, tvecs, rvec, tvec);
        
        std::vector<cv::Point3f> corners_plane;
        std::vector<cv::Point3f> corners_3d;
        cv::Point3f center;
        if(use_center_aruco) {
            
            int center_index = -1;
            for (int i = 0; i < detectedIds.size(); i ++) {
                if (detectedIds[i] == center_id) {
                    center_index = i;
                }
            }
            if (center_index == -1) {

                image_pub.publish(topic_img_pub, rcv_image_packet);
                ROS_INFO("Not Detected Center Marker!\n");
                continue;
            }
            
            center.x = tvecs[center_index][0];
            center.y = tvecs[center_index][1];
            center.z = tvecs[center_index][2];

            
            corners_plane.emplace_back(-caliboard_width/2 * 1000, +caliboard_height/2 * 1000, 0);
            corners_plane.emplace_back(+caliboard_width/2 * 1000, +caliboard_height/2 * 1000, 0);
            corners_plane.emplace_back(+caliboard_width/2 * 1000, -caliboard_height/2 * 1000, 0);
            corners_plane.emplace_back(-caliboard_width/2 * 1000, -caliboard_height/2 * 1000, 0);

            ImageProc::transform3dPoints(corners_plane, corners_3d, rvec, tvec);

            image_draw.drawOrthoCoordinate2d(proc_image, ConversionBridge::rvecs3dToMat_d(rvecs), ConversionBridge::rvecs3dToMat_d(tvecs));
            image_draw.drawLine2d(proc_image, corners_plane[0], corners_plane[1], cv::Mat(rvec), cv::Mat(tvecs[center_index]), cv::Scalar(0, 0, 255));
            image_draw.drawLine2d(proc_image, corners_plane[1], corners_plane[2], cv::Mat(rvec), cv::Mat(tvecs[center_index]), cv::Scalar(0, 0, 255));
            image_draw.drawLine2d(proc_image, corners_plane[2], corners_plane[3], cv::Mat(rvec), cv::Mat(tvecs[center_index]), cv::Scalar(0, 0, 255));
            image_draw.drawLine2d(proc_image, corners_plane[3], corners_plane[0], cv::Mat(rvec), cv::Mat(tvecs[center_index]), cv::Scalar(0, 0, 255));

        }
        else
        {
            
            center.x = tvec[0];
            center.y = tvec[1];
            center.z = tvec[2];

            
            corners_plane.emplace_back(-caliboard_width/2 * 1000, +caliboard_height/2 * 1000, 0);
            corners_plane.emplace_back(+caliboard_width/2 * 1000, +caliboard_height/2 * 1000, 0);
            corners_plane.emplace_back(+caliboard_width/2 * 1000, -caliboard_height/2 * 1000, 0);
            corners_plane.emplace_back(-caliboard_width/2 * 1000, -caliboard_height/2 * 1000, 0);

            ImageProc::transform3dPoints(corners_plane, corners_3d, rvec, tvec);

            image_draw.drawOrthoCoordinate2d(proc_image, ConversionBridge::rvecs3dToMat_d(rvecs), ConversionBridge::rvecs3dToMat_d(tvecs));
            image_draw.drawLine2d(proc_image, corners_plane[0], corners_plane[1], cv::Mat(rvec), cv::Mat(tvec), cv::Scalar(0, 0, 255));
            image_draw.drawLine2d(proc_image, corners_plane[1], corners_plane[2], cv::Mat(rvec), cv::Mat(tvec), cv::Scalar(0, 0, 255));
            image_draw.drawLine2d(proc_image, corners_plane[2], corners_plane[3], cv::Mat(rvec), cv::Mat(tvec), cv::Scalar(0, 0, 255));
            image_draw.drawLine2d(proc_image, corners_plane[3], corners_plane[0], cv::Mat(rvec), cv::Mat(tvec), cv::Scalar(0, 0, 255));
            
        }
        // std::cout << rvec << tvec << corners_plane << std::endl;

        if(pc_process.normalClusterExtraction().size() == 0)
        {
            continue;
        }
        if(pc_process.extractNearestClusterCloud().indices.size() == 0)
        {
            continue;
        }

        auto ring_groups = groupByRing(pc_process.getProcessedPointcloud());
        auto edge_points = extractEdgePoints(ring_groups);
        auto edge_cloud = createEdgePointCloud(edge_points);
        pc_pub.publish(topic_pc_pub, PointCloudPacket<pcl::PointXYZI>(*edge_cloud, frame_id, 0, rosTimeToTimestamp(ros::Time::now())));
        pointcloud2_opr::PointCloud2Proc<pcl::PointXYZI> pc_reproject(true);
        pc_reproject.setCloud(edge_cloud);
        pc_reproject.transform(R, t);
        pc_reproject.scaleTo(1000.0f);
        pc_reproject.PassThroughFilter("z", 0.0, FLT_MAX);

        std::vector<cv::Point2f> imagePoints;
        image_draw.projectPointsToImage(*pc_reproject.getProcessedPointcloud(), imagePoints);
        static float min_intensity = 0;
        static float max_intensity = 70;
        for (size_t i = 0; i < imagePoints.size(); i++) 
        {
            cv::Scalar color = ImageDraw::intensityToRainbowColor(pc_reproject.getProcessedPointcloud()->points[i].intensity, min_intensity, max_intensity);
            cv::circle(proc_image, imagePoints[i], 8, color, -1);
        }

        rcv_image_packet.image = std::make_shared<cv::Mat>(proc_image);
        image_pub.publish(topic_img_pub, rcv_image_packet);

        vector<vector<cv::Point2f>> p2ds;

        if(use_center_aruco) {
            
            int center_index = -1;
            for (int i = 0; i < detectedIds.size(); i ++) {
                if (detectedIds[i] == center_id) {
                    center_index = i;
                }
            }
            
            center.x = tvecs[center_index][0];
            center.y = tvecs[center_index][1];
            center.z = tvecs[center_index][2];

            corners_plane.emplace_back(-caliboard_width/2, +caliboard_height/2, 0);
            corners_plane.emplace_back(+caliboard_width/2, +caliboard_height/2, 0);
            corners_plane.emplace_back(+caliboard_width/2, -caliboard_height/2, 0);
            corners_plane.emplace_back(-caliboard_width/2, -caliboard_height/2, 0);

            std::vector<cv::Point3f> corners_vec1 = {corners_plane[0], corners_plane[1]};
            std::vector<cv::Point3f> corners_vec2 = {corners_plane[1], corners_plane[2]};
            std::vector<cv::Point3f> corners_vec3 = {corners_plane[2], corners_plane[3]};
            std::vector<cv::Point3f> corners_vec4 = {corners_plane[3], corners_plane[0]};

            vector<cv::Point2f> p2d;
            cv::projectPoints(corners_vec1, rvec, tvec, cameraMatrix, distCoeffs, p2d);
            p2ds.emplace_back(p2d);
            cv::projectPoints(corners_vec2, rvec, tvec, cameraMatrix, distCoeffs, p2d);
            p2ds.emplace_back(p2d);
            cv::projectPoints(corners_vec3, rvec, tvec, cameraMatrix, distCoeffs, p2d);
            p2ds.emplace_back(p2d);
            cv::projectPoints(corners_vec4, rvec, tvec, cameraMatrix, distCoeffs, p2d);
            p2ds.emplace_back(p2d);

            ImageProc::transform3dPoints(corners_plane, corners_3d, rvec, tvec);

        }
        else
        {
            
            center.x = tvec[0];
            center.y = tvec[1];
            center.z = tvec[2];
   
            corners_plane.emplace_back(-caliboard_width/2, +caliboard_height/2, 0);
            corners_plane.emplace_back(+caliboard_width/2, +caliboard_height/2, 0);
            corners_plane.emplace_back(+caliboard_width/2, -caliboard_height/2, 0);
            corners_plane.emplace_back(-caliboard_width/2, -caliboard_height/2, 0);

            std::vector<cv::Point3f> corners_vec1 = {corners_plane[0], corners_plane[1]};
            std::vector<cv::Point3f> corners_vec2 = {corners_plane[1], corners_plane[2]};
            std::vector<cv::Point3f> corners_vec3 = {corners_plane[2], corners_plane[3]};
            std::vector<cv::Point3f> corners_vec4 = {corners_plane[3], corners_plane[0]};

            vector<cv::Point2f> p2d;
            cv::projectPoints(corners_vec1, rvec, tvec, cameraMatrix, distCoeffs, p2d);
            p2ds.emplace_back(p2d);
            cv::projectPoints(corners_vec2, rvec, tvec, cameraMatrix, distCoeffs, p2d);
            p2ds.emplace_back(p2d);
            cv::projectPoints(corners_vec3, rvec, tvec, cameraMatrix, distCoeffs, p2d);
            p2ds.emplace_back(p2d);
            cv::projectPoints(corners_vec4, rvec, tvec, cameraMatrix, distCoeffs, p2d);
            p2ds.emplace_back(p2d);

            ImageProc::transform3dPoints(corners_plane, corners_3d, rvec, tvec);
            
        }

        std::string command_received = command_handler.getCommand();

        if (command_received == "capture_border") {
            double reprojection_error = computeReprojectionError(imagePoints, p2ds);
            // std::cout << "Reprojection Error: " << reprojection_error << std::endl;

            std::string filename = "data/reprojection_error_anaylysis";

            // 读取历史数据
            std::vector<double> errors = loadReprojectionErrorData(filename + std::string(".txt"));

            // 打印历史数据
            std::cout << "Loaded historical reprojection errors: ";
            for (const auto& error : errors) {
                std::cout << error << " ";
            }
            std::cout << std::endl;

            if (reprojection_error > 0) {
                // 将新的误差追加到历史数据中
                errors.push_back(reprojection_error);

                // 保存更新后的数据
                saveReprojectionErrorData(filename + std::string(".txt"), errors);

                cv::imwrite(filename + std::string("_") + std::to_string(errors.size()) + std::string(".png"), proc_image);
            }

            // 计算新的平均误差
            double average_error = computeAverageReprojectionError(errors);
            std::cout << "New reprojection error: " << reprojection_error << std::endl;
            std::cout << "Updated average reprojection error: " << average_error << std::endl;

            command_handler.sendCommand("capture_border_complete");
            command_handler.resetReceivedStatus();
        }
            
        rate.sleep();

    }

    return 0;
}