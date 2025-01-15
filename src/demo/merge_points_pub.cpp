#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>             // PCL 和 ROS 点云转换
#include <pcl_conversions/pcl_conversions.h> // ROS 和 PCL 点云转换
#include <pcl/point_cloud.h>                // PCL 点云类型
#include <pcl/point_types.h>                // PCL 点类型
#include <pcl/common/transforms.h>          // PCL 点云变换

class PointCloudFusion {
public:
    PointCloudFusion()
        : nh_("~"), tf_listener_(tf_buffer_) {
        // 订阅两个 LiDAR 点云话题
        velodyne_sub_ = nh_.subscribe("/velodyne_points", 1, &PointCloudFusion::velodyneCallback, this);
        hesai_sub_ = nh_.subscribe("/lidar_points", 1, &PointCloudFusion::hesaiCallback, this);

        // 发布融合后的点云
        merged_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/merged_points", 1);

        velodyne_received_ = false;
        hesai_received_ = false;
    }

    void velodyneCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        // ROS_INFO("Received /velodyne_points");
        velodyne_cloud_ = *msg;  // 保存 Velodyne 点云
        velodyne_received_ = true;
    }

    void hesaiCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        // ROS_INFO("Received /lidar_points");
        try {
            // 获取 TF 变换
            geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform(
                "velodyne", msg->header.frame_id, msg->header.stamp, ros::Duration(1.0));

            // 将 TF 变换转换为 Eigen::Matrix4f
            Eigen::Matrix4f transform_matrix = transformToEigen(transform_stamped);

            // 将 Hesai 点云转换为 PCL 格式
            pcl::PointCloud<pcl::PointXYZI>::Ptr hesai_pcl(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*msg, *hesai_pcl);

            // 应用变换到 Hesai 点云
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_pcl(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(*hesai_pcl, *transformed_pcl, transform_matrix);

            // 转换回 ROS 格式
            pcl::toROSMsg(*transformed_pcl, hesai_cloud_);
            hesai_cloud_.header.frame_id = "velodyne";
            hesai_cloud_.header.stamp = msg->header.stamp;

            hesai_received_ = true;
        } catch (tf2::TransformException& ex) {
            ROS_WARN("TF transform failed: %s", ex.what());
        }
    }

    void fusePointClouds() {
        if (!velodyne_received_ || !hesai_received_) {
            return;  // 如果任意一个点云没有接收到，跳过处理
        }

        // ROS_INFO("Fusing point clouds...");

        // 将 Velodyne 点云转换为 PCL 格式
        pcl::PointCloud<pcl::PointXYZI>::Ptr velodyne_pcl(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(velodyne_cloud_, *velodyne_pcl);

        // 将 Hesai 点云转换为 PCL 格式
        pcl::PointCloud<pcl::PointXYZI>::Ptr hesai_pcl(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(hesai_cloud_, *hesai_pcl);

        // 合并两个点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr merged_pcl(new pcl::PointCloud<pcl::PointXYZI>());
        *merged_pcl = *velodyne_pcl + *hesai_pcl;

        // 转换为 ROS 消息
        sensor_msgs::PointCloud2 merged_cloud;
        pcl::toROSMsg(*merged_pcl, merged_cloud);

        // 设置消息头
        merged_cloud.header.stamp = hesai_cloud_.header.stamp;
        merged_cloud.header.frame_id = "velodyne";

        // 发布融合后的点云
        merged_pub_.publish(merged_cloud);

        // ROS_INFO("Published /merged_points");
    }

    void spin() {
        ros::Rate rate(10);  // 10 Hz
        while (ros::ok()) {
            fusePointClouds();
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber velodyne_sub_;
    ros::Subscriber hesai_sub_;
    ros::Publisher merged_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    sensor_msgs::PointCloud2 velodyne_cloud_;
    sensor_msgs::PointCloud2 hesai_cloud_;

    bool velodyne_received_;
    bool hesai_received_;

    Eigen::Matrix4f transformToEigen(const geometry_msgs::TransformStamped& transform_stamped) {
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

        // 平移部分
        transform(0, 3) = transform_stamped.transform.translation.x;
        transform(1, 3) = transform_stamped.transform.translation.y;
        transform(2, 3) = transform_stamped.transform.translation.z;

        // 旋转部分
        Eigen::Quaternionf q(
            transform_stamped.transform.rotation.w,
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z);
        Eigen::Matrix3f rotation = q.toRotationMatrix();

        transform.block<3, 3>(0, 0) = rotation;  // 将旋转矩阵赋值给变换矩阵

        return transform;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_fusion_node");

    PointCloudFusion fusion_node;
    fusion_node.spin();

    return 0;
}