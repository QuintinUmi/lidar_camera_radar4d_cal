#ifndef _ROS_TOPIC_MANAGER_H_
#define _ROS_TOPIC_MANAGER_H_

#include <ros/ros.h>
#include <mutex>
#include <queue>
#include <string>
#include <vector>
#include <boost/bind.hpp> 

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#include "tools/timestamp_cov.h"

#define SHOW_DEBUG_MESSAGE false

using namespace message_filters;

namespace lcr_cal {

    struct ImagePacket {
        std::shared_ptr<cv::Mat> image;
        std::string frame_id;
        int seq;
        uint64_t timestamp;
        bool is_valid;

        operator bool() const {
            return is_valid && !image->empty();
        }

        ImagePacket() : is_valid(false) {}
        ImagePacket(std::shared_ptr<cv::Mat> image, std::string frame_id, int seq, uint64_t timestamp)
            : image(image), frame_id(frame_id), seq(seq), timestamp(timestamp), is_valid(true) {}
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
                std::lock_guard<std::mutex> lock(mutex_);
                try {
                    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, encoding);
                    if (image_queues_[topic].size() >= queue_size_) {
                        image_queues_[topic].pop();
                    }
                    if (image_queues_[topic].size() > queue_size_) image_queues_[topic].pop();
                    image_queues_[topic].emplace(std::make_shared<cv::Mat>(cv_ptr->image), cv_ptr->header.frame_id, cv_ptr->header.seq, rosTimeToTimestamp(cv_ptr->header.stamp));
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
                if (it != pub_.end() && !image_packet.image->empty()) {
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





namespace lcr_cal {

    struct PointCloudPacket {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        std::string frame_id;
        int seq;
        uint64_t timestamp;
        bool is_valid;

        operator bool() const {
            return is_valid && (cloud.size() != 0);
        }

        PointCloudPacket() : is_valid(false) {}
        PointCloudPacket(pcl::PointCloud<pcl::PointXYZI> cloud, std::string frame_id, int seq, uint64_t timestamp)
            : cloud(cloud), frame_id(frame_id), seq(seq), timestamp(timestamp), is_valid(true) {}
    };

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

        PointCloudPacket getPointCloud(const std::string& topic) {
            std::lock_guard<std::mutex> lock(mutex_);
            auto it = cloud_queues_.find(topic);
            if (it != cloud_queues_.end() && !it->second.empty()) {
                PointCloudPacket packet = std::move(it->second.front());
                it->second.pop();
                return packet;
            }
            return PointCloudPacket();  // 返回无效的点云包
        }

    private:
        void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg, const std::string& topic) {
            pcl::PointCloud<pcl::PointXYZI> cloud;
            pcl::fromROSMsg(*msg, cloud);  // 将 ROS 点云消息转换为 PCL 点云格式
            std::lock_guard<std::mutex> lock(mutex_);
            if (cloud_queues_[topic].size() >= queue_size_) {
                cloud_queues_[topic].pop();
            }
            if (cloud_queues_[topic].size() > queue_size_) cloud_queues_[topic].pop();
            cloud_queues_[topic].emplace(cloud, msg->header.frame_id, msg->header.seq, rosTimeToTimestamp(msg->header.stamp));
        }

        ros::NodeHandle nh_;
        std::vector<ros::Subscriber> subscribers_;
        std::map<std::string, std::queue<PointCloudPacket>> cloud_queues_;
        std::mutex mutex_;
        size_t queue_size_;
    };


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

        void publish(const std::string& topic, const PointCloudPacket& cloud_packet) {
            auto it = pub_.find(topic);
            if (it != pub_.end() && !cloud_packet.cloud.empty()) {
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
}



namespace lcr_cal 
{

    struct Radar4DPoint {
        float x;
        float y;
        float z;
        float velocity;
        float snr;
        float power;
        uint16_t valid_flg;
        uint16_t motion_state;
    
        Radar4DPoint() : x(0), y(0), z(0), velocity(0), snr(0), power(0), valid_flg(0), motion_state(0) {}
    };
    
    struct Radar4DPacket {
        std::vector<Radar4DPoint> points;
        std::string frame_id;
        int seq;
        uint64_t timestamp;
        bool is_valid;
    
        // 布尔运算符，用于判断点云包的有效性
        operator bool() const {
            return is_valid && !points.empty();
        }
    
        // 默认构造函数，生成无效点云包
        Radar4DPacket() : is_valid(false) {}
    
        // 带参数的构造函数，用于创建有效点云包
        Radar4DPacket(std::vector<Radar4DPoint> points, std::string frame_id, int seq, uint64_t timestamp)
            : points(std::move(points)), frame_id(std::move(frame_id)), seq(seq), timestamp(timestamp), is_valid(true) {}
    };
    
    class Radar4DSubscriber {
    public:
        Radar4DSubscriber(ros::NodeHandle& nh, size_t queue_size)
            : nh_(nh), queue_size_(queue_size) {}
    
        void addTopic(const std::string& topic) {
            std::lock_guard<std::mutex> lock(mutex_);
            ros::Subscriber sub = nh_.subscribe<sensor_msgs::PointCloud2>(
                topic, 1,
                boost::bind(&Radar4DSubscriber::pointCloudCallback, this, _1, topic)
            );
            subscribers_.emplace_back(sub);
            ROS_INFO("Subscribed to radar4d topic: %s", topic.c_str());
        }
    
        void addTopics(const std::vector<std::string>& topics) {
            for (const auto& topic : topics) {
                addTopic(topic);
            }
        }
    
        Radar4DPacket getRadar4DPacket(const std::string& topic) {
            std::lock_guard<std::mutex> lock(mutex_);
            auto it = radar_queues_.find(topic);
            if (it != radar_queues_.end() && !it->second.empty()) {
                Radar4DPacket packet = std::move(it->second.front());
                it->second.pop();
                return packet;
            }
            return Radar4DPacket(); // 返回无效点云包
        }
    
    private:
        void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg, const std::string& topic) {
            std::lock_guard<std::mutex> lock(mutex_);
    
            // 解析点云字段
            std::vector<Radar4DPoint> radar_points;
            const uint32_t point_step = msg->point_step; // 每个点的字节数
            const uint32_t data_size = msg->data.size();
    
            for (size_t i = 0; i < data_size; i += point_step) {
                Radar4DPoint point;
                const uint8_t* point_data = &msg->data[i];
    
                // 解析字段
                point.x = *reinterpret_cast<const float*>(point_data + 0);
                point.y = *reinterpret_cast<const float*>(point_data + 4);
                point.z = *reinterpret_cast<const float*>(point_data + 8);
                point.velocity = *reinterpret_cast<const float*>(point_data + 16);
                point.snr = *reinterpret_cast<const float*>(point_data + 20);
                point.power = *reinterpret_cast<const float*>(point_data + 24);
                point.valid_flg = *reinterpret_cast<const uint16_t*>(point_data + 28);
                point.motion_state = *reinterpret_cast<const uint16_t*>(point_data + 30);
    
                // 保存所有点
                radar_points.push_back(point);
            }
    
            // 如果队列超过容量，移除最早的数据
            if (radar_queues_[topic].size() >= queue_size_) {
                radar_queues_[topic].pop();
            }
    
            radar_queues_[topic].emplace(radar_points, msg->header.frame_id, msg->header.seq, rosTimeToTimestamp(msg->header.stamp));
        }
    
        ros::NodeHandle nh_;
        std::vector<ros::Subscriber> subscribers_;
        std::map<std::string, std::queue<Radar4DPacket>> radar_queues_;
        std::mutex mutex_;
        size_t queue_size_;
    };
    
    class Radar4DPublisher {
    public:
        Radar4DPublisher(ros::NodeHandle& nh) : nh_(nh) {}
    
        void addTopic(const std::string& topic, size_t queue_size) {
            pubs_[topic] = nh_.advertise<sensor_msgs::PointCloud2>(topic, queue_size);
            ROS_INFO("Radar4D publisher added for topic: %s", topic.c_str());
        }

        void addTopics(const std::vector<std::string>& topics, size_t queue_size) {
            for (const std::string& topic : topics) {
                addTopic(topic, queue_size);
            }
        }
    
        void publish(const std::string& topic, const Radar4DPacket& radar_packet) {
            auto it = pubs_.find(topic);
            if (it != pubs_.end() && radar_packet) { // 使用布尔运算符判断点云包有效性
                sensor_msgs::PointCloud2 msg;
                msg.header.stamp = ros::Time().fromNSec(radar_packet.timestamp);
                msg.header.frame_id = radar_packet.frame_id;
                msg.height = 1;
                msg.width = radar_packet.points.size();
                msg.is_dense = true;
                msg.is_bigendian = false;
                msg.point_step = 32; // 每个点 32 字节
                msg.row_step = msg.point_step * msg.width;
    
                // 定义字段
                msg.fields.resize(8);
                msg.fields[0].name = "x";
                msg.fields[0].offset = 0;
                msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
                msg.fields[0].count = 1;
    
                msg.fields[1].name = "y";
                msg.fields[1].offset = 4;
                msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
                msg.fields[1].count = 1;
    
                msg.fields[2].name = "z";
                msg.fields[2].offset = 8;
                msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
                msg.fields[2].count = 1;
    
                msg.fields[3].name = "velocity";
                msg.fields[3].offset = 16;
                msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
                msg.fields[3].count = 1;
    
                msg.fields[4].name = "snr";
                msg.fields[4].offset = 20;
                msg.fields[4].datatype = sensor_msgs::PointField::FLOAT32;
                msg.fields[4].count = 1;
    
                msg.fields[5].name = "power";
                msg.fields[5].offset = 24;
                msg.fields[5].datatype = sensor_msgs::PointField::FLOAT32;
                msg.fields[5].count = 1;
    
                msg.fields[6].name = "valid_flg";
                msg.fields[6].offset = 28;
                msg.fields[6].datatype = sensor_msgs::PointField::UINT16;
                msg.fields[6].count = 1;
    
                msg.fields[7].name = "motion_state";
                msg.fields[7].offset = 30;
                msg.fields[7].datatype = sensor_msgs::PointField::UINT16;
                msg.fields[7].count = 1;
    
                // 填充点云数据
                msg.data.resize(msg.row_step);
                for (size_t i = 0; i < radar_packet.points.size(); ++i) {
                    const Radar4DPoint& point = radar_packet.points[i];
                    uint8_t* ptr = &msg.data[i * msg.point_step];
                    memcpy(ptr + 0, &point.x, sizeof(float));
                    memcpy(ptr + 4, &point.y, sizeof(float));
                    memcpy(ptr + 8, &point.z, sizeof(float));
                    memcpy(ptr + 16, &point.velocity, sizeof(float));
                    memcpy(ptr + 20, &point.snr, sizeof(float));
                    memcpy(ptr + 24, &point.power, sizeof(float));
                    memcpy(ptr + 28, &point.valid_flg, sizeof(uint16_t));
                    memcpy(ptr + 30, &point.motion_state, sizeof(uint16_t));
                }
    
                it->second.publish(msg);
                ROS_INFO("Published Radar4D data to topic: %s", topic.c_str());
            } else {
                ROS_WARN("No publisher available for topic: %s", topic.c_str());
            }
        }
    
    private:
        ros::NodeHandle nh_;
        std::map<std::string, ros::Publisher> pubs_;
    };
}




namespace lcr_cal {

    struct PointsPacket {
        geometry_msgs::PolygonStamped points;
        std::string frame_id;
        int seq;
        uint64_t timestamp;
        bool is_valid;

        operator bool() const {
            return is_valid;
        }

        PointsPacket() : is_valid(false) {}
        PointsPacket(geometry_msgs::PolygonStamped points, std::string frame_id, int seq, uint64_t timestamp)
            : points(points), frame_id(frame_id), seq(seq), timestamp(timestamp), is_valid(true) {}
    };

    class PointsetSubscriber {
    public:
        PointsetSubscriber(ros::NodeHandle& nh, size_t queue_size)
            : nh_(nh), queue_size_(queue_size) {}

        void addTopic(const std::string& topic) {
            std::lock_guard<std::mutex> lock(mutex_);
            ros::Subscriber sub = nh_.subscribe<geometry_msgs::PolygonStamped>(
                topic, 1, 
                boost::bind(&PointsetSubscriber::pointsCallback, this, _1, topic)
            );
            subscribers_.emplace_back(sub);
            if (SHOW_DEBUG_MESSAGE) ROS_INFO("Subscribed to points topic: %s", topic.c_str());
        }

        void addTopics(const std::vector<std::string>& topics) {
            for (const auto& topic : topics) {
                addTopic(topic);
            }
        }

        PointsPacket getPoints(const std::string& topic) {
            std::lock_guard<std::mutex> lock(mutex_);
            auto it = points_queues_.find(topic);
            if (it != points_queues_.end() && !it->second.empty()) {
                PointsPacket packet = std::move(it->second.front());
                it->second.pop();
                return packet;
            }
            return PointsPacket();  
        }

    private:
        void pointsCallback(const geometry_msgs::PolygonStamped::ConstPtr& msg, const std::string& topic) {
            std::lock_guard<std::mutex> lock(mutex_);
            if (points_queues_[topic].size() >= queue_size_) {
                points_queues_[topic].pop();
            }
            if (points_queues_[topic].size() > queue_size_) points_queues_[topic].pop();
            points_queues_[topic].emplace(*msg, msg->header.frame_id, msg->header.seq, rosTimeToTimestamp(msg->header.stamp));
        }

        ros::NodeHandle nh_;
        std::vector<ros::Subscriber> subscribers_;
        std::map<std::string, std::queue<PointsPacket>> points_queues_;
        std::mutex mutex_;
        size_t queue_size_;
    };


    class PointsetPublisher {
    public:
        PointsetPublisher(ros::NodeHandle& nh) : nh_(nh) {}

        void addTopic(const std::string& topic, size_t queue_size) {
            pub_[topic] = nh_.advertise<geometry_msgs::PolygonStamped>(topic, queue_size);
            if (SHOW_DEBUG_MESSAGE) ROS_INFO("PointsetPublisher added for topic: %s", topic.c_str());
        }

        void addTopics(const std::vector<std::string>& topics, size_t queue_size) {
            for (const std::string& topic : topics) {
                addTopic(topic, queue_size);
            }
        }

        void publish(const std::string& topic, const PointsPacket& points_packet) {
            auto it = pub_.find(topic); 
            if (it != pub_.end() && points_packet) {
                geometry_msgs::PolygonStamped points;
                points = points_packet.points;
                points.header.stamp = timestampToRosTime(points_packet.timestamp);
                points.header.frame_id = points_packet.frame_id;  // 设置适当的 frame_id
                it->second.publish(points);
                if (SHOW_DEBUG_MESSAGE) ROS_INFO("Published points to topic: %s", topic.c_str());
            } else {
                ROS_WARN("No publisher available for topic: %s", topic.c_str());
            }
        }

    private:
        ros::NodeHandle nh_;
        std::map<std::string, ros::Publisher> pub_;
    };
}



namespace lcr_cal {

    struct TransformPacket {
        geometry_msgs::PoseStamped pose;  // 使用 PoseStamped 传输
        bool is_valid;                    // 数据有效标志

        operator bool() const {
            return is_valid;
        }

        TransformPacket() : is_valid(false) {}
        TransformPacket(geometry_msgs::PoseStamped pose)
            : pose(pose), is_valid(true) {}
    };

    class TransformUtils {
    public:
        // 静态函数：将 rvec 和 tvec 转换为 TransformPacket
        static TransformPacket createTransformPacket(const cv::Vec3d& rvec, const cv::Vec3d& tvec, const std::string& frame_id) {
            // 将 rvec 转换为四元数
            cv::Mat rotation_matrix;
            cv::Rodrigues(rvec, rotation_matrix);
            cv::Matx33d R(rotation_matrix);

            double qw = sqrt(1.0 + R(0, 0) + R(1, 1) + R(2, 2)) / 2.0;
            double qx = (R(2, 1) - R(1, 2)) / (4.0 * qw);
            double qy = (R(0, 2) - R(2, 0)) / (4.0 * qw);
            double qz = (R(1, 0) - R(0, 1)) / (4.0 * qw);

            // 填充 PoseStamped 消息
            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.stamp = ros::Time::now();
            pose_msg.header.frame_id = frame_id;
            pose_msg.pose.position.x = tvec[0];
            pose_msg.pose.position.y = tvec[1];
            pose_msg.pose.position.z = tvec[2];
            pose_msg.pose.orientation.x = qx;
            pose_msg.pose.orientation.y = qy;
            pose_msg.pose.orientation.z = qz;
            pose_msg.pose.orientation.w = qw;

            // 返回 TransformPacket
            return TransformPacket(pose_msg);
        }

        // 静态函数：从 TransformPacket 提取 rvec 和 tvec
        static void extractRvecTvec(const TransformPacket& packet, cv::Vec3d& rvec, cv::Vec3d& tvec) {
            // 提取平移向量
            tvec[0] = packet.pose.pose.position.x;
            tvec[1] = packet.pose.pose.position.y;
            tvec[2] = packet.pose.pose.position.z;

            // 提取四元数
            double qx = packet.pose.pose.orientation.x;
            double qy = packet.pose.pose.orientation.y;
            double qz = packet.pose.pose.orientation.z;
            double qw = packet.pose.pose.orientation.w;

            // 将四元数转换为旋转矩阵
            cv::Matx33d R(
                1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy),
                2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx),
                2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)
            );

            // 将旋转矩阵转换为旋转向量
            cv::Rodrigues(R, rvec);
        }
    };

    class TransformSubscriber {
    public:
        TransformSubscriber(ros::NodeHandle& nh, size_t queue_size)
            : nh_(nh), queue_size_(queue_size) {}

        void addTopic(const std::string& topic) {
            std::lock_guard<std::mutex> lock(mutex_);
            ros::Subscriber sub = nh_.subscribe<geometry_msgs::PoseStamped>(
                topic, 1,
                boost::bind(&TransformSubscriber::TransformCallback, this, _1, topic)
            );
            subscribers_.emplace_back(sub);
            ROS_INFO("Subscribed to Transform topic: %s", topic.c_str());
        }

        void addTopics(const std::vector<std::string>& topics) {
            for (const auto& topic : topics) {
                addTopic(topic);
            }
        }

        TransformPacket getTransform(const std::string& topic) {
            std::lock_guard<std::mutex> lock(mutex_);
            auto it = transform_queues_.find(topic);
            if (it != transform_queues_.end() && !it->second.empty()) {
                TransformPacket packet = std::move(it->second.front());
                it->second.pop();
                return packet;
            }
            return TransformPacket();
        }

        // 提取 rvec 和 tvec
        bool getRvecTvec(const std::string& topic, cv::Vec3d& rvec, cv::Vec3d& tvec) {
            TransformPacket packet = getTransform(topic);
            if (packet) {
                TransformUtils::extractRvecTvec(packet, rvec, tvec);
                return true;
            }
            return false;
        }

    private:
        void TransformCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, const std::string& topic) {
            std::lock_guard<std::mutex> lock(mutex_);
            if (transform_queues_[topic].size() >= queue_size_) {
                transform_queues_[topic].pop();
            }
            transform_queues_[topic].emplace(*msg);
        }

        ros::NodeHandle nh_;
        std::vector<ros::Subscriber> subscribers_;
        std::map<std::string, std::queue<TransformPacket>> transform_queues_;
        std::mutex mutex_;
        size_t queue_size_;
    };

    class TransformPublisher {
    public:
        TransformPublisher(ros::NodeHandle& nh) : nh_(nh) {}

        void addTopic(const std::string& topic, size_t queue_size) {
            pub_[topic] = nh_.advertise<geometry_msgs::PoseStamped>(topic, queue_size);
            ROS_INFO("TransformPublisher added for topic: %s", topic.c_str());
        }

        void addTopics(const std::vector<std::string>& topics, size_t queue_size) {
            for (const std::string& topic : topics) {
                addTopic(topic, queue_size);
            }
        }

        void publish(const std::string& topic, const TransformPacket& transform_packet) {
            auto it = pub_.find(topic);
            if (it != pub_.end() && transform_packet) {
                geometry_msgs::PoseStamped pose_msg = transform_packet.pose;
                pose_msg.header.stamp = ros::Time::now();  // 设置时间戳
                it->second.publish(pose_msg);
                // ROS_INFO("Published Transform to topic: %s", topic.c_str());
            } else {
                ROS_WARN("No publisher available for topic: %s", topic.c_str());
            }
        }

    private:
        ros::NodeHandle nh_;
        std::map<std::string, ros::Publisher> pub_;
    };
}


#endif