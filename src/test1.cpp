#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>

// 自定义点类型的定义
struct PointXYZIRT
{
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// 在PCL库中注册新的点类型
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
    (double, timestamp, timestamp)
)

double ensureNanoseconds(double timestamp) {
    // Determine the magnitude of the timestamp to guess if it's in seconds
    double magnitude = timestamp > 0 ? std::floor(std::log10(timestamp)) : 0;

    std::cout << magnitude << std::endl;
    // If the magnitude is less than 9, assume the timestamp is in seconds (common threshold)
    if (magnitude < 10) {
        return timestamp * 1e9;  // Convert seconds to nanoseconds
    }

    // If the magnitude is 9 or greater, assume it's already in nanoseconds
    return timestamp;
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // 创建自定义点云类型的智能指针
    pcl::PointCloud<PointXYZIRT>::Ptr cloud(new pcl::PointCloud<PointXYZIRT>);

    // 从ROS消息转换为PCL点云
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // 输出第一个点的时间戳（假设所有点具有相同的时间戳）
    if (!cloud->points.empty()) {
        ROS_INFO("Received point cloud with timestamp: %ld", ensureNanoseconds(cloud->points[0].timestamp));
    } else {
        ROS_INFO("Received an empty point cloud.");
    }

    for (auto point : cloud->points) {
        std::cout << "ring:" << point.ring << std::endl;
    }
    // 可以在这里添加更多处理逻辑
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/rslidar_points", 10, cloudCallback);

    ros::spin();

    return 0;
}