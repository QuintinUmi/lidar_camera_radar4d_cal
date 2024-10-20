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

#include "tools/timestamp_cov.h"

#define SHOW_DEBUG_MESSAGE false

using namespace message_filters;

namespace lidar_camera_cal {

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





namespace lidar_camera_cal {

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


namespace lidar_camera_cal {

    struct CornersPacket {
        geometry_msgs::PolygonStamped corners;
        std::string frame_id;
        int seq;
        uint64_t timestamp;
        bool is_valid;

        operator bool() const {
            return is_valid;
        }

        CornersPacket() : is_valid(false) {}
        CornersPacket(geometry_msgs::PolygonStamped corners, std::string frame_id, int seq, uint64_t timestamp)
            : corners(corners), frame_id(frame_id), seq(seq), timestamp(timestamp), is_valid(true) {}
    };

    class CornersSubscriber {
    public:
        CornersSubscriber(ros::NodeHandle& nh, size_t queue_size)
            : nh_(nh), queue_size_(queue_size) {}

        void addTopic(const std::string& topic) {
            std::lock_guard<std::mutex> lock(mutex_);
            ros::Subscriber sub = nh_.subscribe<geometry_msgs::PolygonStamped>(
                topic, 1, 
                boost::bind(&CornersSubscriber::CornersCallback, this, _1, topic)
            );
            subscribers_.emplace_back(sub);
            if (SHOW_DEBUG_MESSAGE) ROS_INFO("Subscribed to Corners topic: %s", topic.c_str());
        }

        void addTopics(const std::vector<std::string>& topics) {
            for (const auto& topic : topics) {
                addTopic(topic);
            }
        }

        CornersPacket getCorners(const std::string& topic) {
            std::lock_guard<std::mutex> lock(mutex_);
            auto it = corners_queues_.find(topic);
            if (it != corners_queues_.end() && !it->second.empty()) {
                CornersPacket packet = std::move(it->second.front());
                it->second.pop();
                return packet;
            }
            return CornersPacket();  
        }

    private:
        void CornersCallback(const geometry_msgs::PolygonStamped::ConstPtr& msg, const std::string& topic) {
            std::lock_guard<std::mutex> lock(mutex_);
            if (corners_queues_[topic].size() >= queue_size_) {
                corners_queues_[topic].pop();
            }
            if (corners_queues_[topic].size() > queue_size_) corners_queues_[topic].pop();
            corners_queues_[topic].emplace(*msg, msg->header.frame_id, msg->header.seq, rosTimeToTimestamp(msg->header.stamp));
        }

        ros::NodeHandle nh_;
        std::vector<ros::Subscriber> subscribers_;
        std::map<std::string, std::queue<CornersPacket>> corners_queues_;
        std::mutex mutex_;
        size_t queue_size_;
    };


    class CornersPublisher {
    public:
        CornersPublisher(ros::NodeHandle& nh) : nh_(nh) {}

        void addTopic(const std::string& topic, size_t queue_size) {
            pub_[topic] = nh_.advertise<geometry_msgs::PolygonStamped>(topic, queue_size);
            if (SHOW_DEBUG_MESSAGE) ROS_INFO("CornersPublisher added for topic: %s", topic.c_str());
        }

        void addTopics(const std::vector<std::string>& topics, size_t queue_size) {
            for (const std::string& topic : topics) {
                addTopic(topic, queue_size);
            }
        }

        void publish(const std::string& topic, const CornersPacket& corners_packet) {
            auto it = pub_.find(topic); 
            if (it != pub_.end() && corners_packet) {
                geometry_msgs::PolygonStamped corners;
                corners = corners_packet.corners;
                corners.header.stamp = timestampToRosTime(corners_packet.timestamp);
                corners.header.frame_id = corners_packet.frame_id;  // 设置适当的 frame_id
                it->second.publish(corners);
                if (SHOW_DEBUG_MESSAGE) ROS_INFO("Published Corners to topic: %s", topic.c_str());
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