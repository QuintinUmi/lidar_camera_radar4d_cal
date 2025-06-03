#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <boost/filesystem.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>  
#include <aruco/aruco.h>  
#include "opencv2/aruco/charuco.hpp"  

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include <pcl/visualization/cloud_viewer.h> 

#include <yaml-cpp/yaml.h>

#include "image_transport/image_transport.h"

#include "calibration_tool.h"

#include "image_opr/image_process.h"
#include "image_opr/image_draw.h"

#include "pointcloud2_opr/point_cloud_process.h"


#include "pointcloud2_opr/point_cloud_process.h"

#include "tools/dynamic_reconfigure.h"
#include "tools/rviz_draw.h"
#include "tools/ros_topic_manager.h"
#include "tools/conversion_bridge.h"
#include "tools/file_operator.h"
#include "tools/CommandHandler.h"


#define PI 3.14159265358979324


using namespace std;
using namespace lcr_cal;
using namespace lcr_cal::pointcloud2_opr;
using namespace lcr_cal::image_opr;


int fps(int deltaTime) 
{
    int fps = static_cast<int>(1.f / deltaTime * 1000); 
    return fps;
}

double computeReprojectionError(
    const std::vector<cv::Point2f>& projected_points,  // 投影的激光雷达点（像素坐标）
    const std::vector<cv::Point2f>& pixel_corners      // 标定板角点（像素坐标）
) {
    double total_error = 0.0;  // 总误差初始化为 0

    // 遍历每个投影点
    for (size_t i = 0; i < projected_points.size(); ++i) {
        const cv::Point2f& projected_point = projected_points[i];  // 当前投影点

        // 初始化最近点和最小距离
        double min_distance = std::numeric_limits<double>::max();
        cv::Point2f nearest_corner;

        // 遍历所有标定板角点，找到最近的点
        for (const auto& corner : pixel_corners) {
            cv::Point2f image_corner(corner.x, corner.y);  // 转换为 2D 点
            double distance = cv::norm(projected_point - image_corner);  // 计算距离

            if (distance < min_distance) {
                min_distance = distance;  // 更新最小距离
                nearest_corner = image_corner;  // 更新最近点
            }
        }

        // 记录误差
        total_error += min_distance;

        // 输出每个点的详细误差信息
        std::cout << "Projected Point [" << i << "]: " << projected_point
                  << ", Nearest Image Corner: " << nearest_corner
                  << ", Error: " << min_distance << std::endl;
    }

    // 平均误差
    double mean_error = total_error / projected_points.size();

    // 返回平均误差
    return mean_error;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr convertToPointCloud(const std::vector<Radar4DPoint>& radar_points) {
    // 创建一个 PointCloud XYZI 的智能指针
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    // 遍历 Radar4DPoint 的 vector，将每个点转换为 PointXYZI
    for (const auto& point : radar_points) {
        pcl::PointXYZI pcl_point;
        pcl_point.x = point.x;
        pcl_point.y = point.y;
        pcl_point.z = point.z;
        pcl_point.intensity = point.snr;  // 将 snr 转换为强度 I

        cloud->points.push_back(pcl_point); // 添加到点云中
    }

    // 设置点云的宽度、高度和密集标志
    cloud->width = cloud->points.size();
    cloud->height = 1; // 表示点云是一维的（无组织）
    cloud->is_dense = true;

    return cloud; // 返回转换后的点云
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "calibration_node");
    ros::NodeHandle rosHandle;

    bool undistort;
    int undistort_mode;

    std::string frame_id;
    std::string frame_id_radar;

    std::string topic_pc_sub;
    std::string topic_pc_proc_sub;
    std::string topic_pc_pub;

    std::string topic_rd_sub;

    std::string topic_img_corners_sub;
    std::string topic_img_center_sub;
    std::string topic_pc_corners_sub;
	std::string topic_pc_center_sub;
	std::string topic_rd_center_sub;
    std::string topic_corners_pub;

    std::string topic_trans_sub;

    std::string topic_command_sub;
    std::string topic_command_pub;

    rosHandle.param("undistort", undistort, false);
    rosHandle.param("undistort_mode", undistort_mode, 0);

    rosHandle.param("frame_id", frame_id, std::string("velodyne"));
    rosHandle.param("frame_id_radar", frame_id_radar, std::string("car"));

	rosHandle.param("pointcloud_process_pc_sub_topic", topic_pc_sub, std::string("/rslidar_points"));

	rosHandle.param("radar4d_process_pc_sub_topic", topic_rd_sub, std::string("/Target_Radar_1_extreme"));

    rosHandle.param("image_process_cor_pub_topic", topic_img_corners_sub, std::string("/image_process/corners"));
    rosHandle.param("image_process_cen_pub_topic", topic_img_center_sub, std::string("/image_process/center"));
    rosHandle.param("pointcloud_process_cor_pub_topic", topic_pc_corners_sub, std::string("/pointcloud_process/corners"));
    rosHandle.param("pointcloud_process_cen_pub_topic", topic_pc_center_sub, std::string("/pointcloud_process/center"));
    rosHandle.param("radar4d_process_cen_pub_topic", topic_rd_center_sub, std::string("/radar4d_process/center"));

    rosHandle.param("image_process_trans_pub_topic", topic_trans_sub, std::string("/image_process/trans"));

    rosHandle.param("calibration_command_sub_topic", topic_command_sub, std::string("/lcr_cal/command_controller"));
    rosHandle.param("calibration_command_pub_topic", topic_command_pub, std::string("/lcr_cal/command_cal_node"));


    float caliboard_width;
    float caliboard_height;
    rosHandle.param("caliboard_width", caliboard_width, (float)12.0);
    rosHandle.param("caliboard_height", caliboard_height, (float)12.0);

    std::vector<cv::Point3f> image_corners;
    image_corners.emplace_back(-caliboard_width/2, +caliboard_height/2, 0);
    image_corners.emplace_back(+caliboard_width/2, +caliboard_height/2, 0);
    image_corners.emplace_back(+caliboard_width/2, -caliboard_height/2, 0);
    image_corners.emplace_back(-caliboard_width/2, -caliboard_height/2, 0);


    std::string packagePath;
    if (!rosHandle.getParam("package_path", packagePath)) {
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
    rosHandle.param("yaml_path", yaml_path, cv::String("~/"));

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


    std::string pointset_csv_path;
    std::string error_anaylysis_csv_path;
    std::string extrinsics_path;
    rosHandle.param("rl_pointset_save_path", pointset_csv_path, std::string("data/radar_lidar/point_set.csv"));
    rosHandle.param("error_analysis_save_path", error_anaylysis_csv_path, std::string("data/border_error_anaylysis.csv"));
    rosHandle.param("rl_extrinsics_save_path", extrinsics_path, std::string("result/radar_lidar/extrinsics.yaml"));
    rosHandle.setParam("pointset_save_path", pointset_csv_path);

    PointCloudSubscriber pc_sub(rosHandle, 5);
    PointCloudPublisher pc_pub(rosHandle);
    pc_sub.addTopic(topic_pc_sub);
    pc_pub.addTopic("/show_radar_trans_cloud", 5);

    Radar4DSubscriber rd_sub(rosHandle, 5);
    Radar4DPublisher rd_pub(rosHandle);
    rd_sub.addTopic(topic_rd_sub);

    PointsetSubscriber pts_sub(rosHandle, 5);
    PointsetPublisher pts_pub(rosHandle);
    pts_sub.addTopic(topic_img_corners_sub);
    pts_sub.addTopic(topic_img_center_sub);
    pts_sub.addTopic(topic_pc_corners_sub);
    pts_sub.addTopic(topic_pc_center_sub);
    pts_sub.addTopic(topic_rd_center_sub);

    TransformSubscriber transform_sub(rosHandle, 5);
    transform_sub.addTopic(topic_trans_sub);


    PointCloud2Proc<pcl::PointXYZI> pc_process(true);

    ImageDraw image_draw;
    if (undistort) {
        image_draw = ImageDraw(1.0, 1.0, 1.0, 1.0, newCameraMatrix, newDistCoeffes);
    } else {
        image_draw = ImageDraw(1.0, 1.0, 1.0, 1.0, cameraMatrix, distCoeffs);
    }

    RQTConfig rqtCfg;
    // PointcloudFilterReconfigure filterRecfg(rosHandle);

    PointSetCsvOperator pointset_csv_operator(pointset_csv_path);
    BorderSetCsvOperator boarderset_csv_operator(error_anaylysis_csv_path);
    // boarderset_csv_operator.writePointsToCSVOverwrite(pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>(new pcl::PointCloud<pcl::PointXYZI>), std::vector<std::vector<geometry_msgs::Point32>>());

    YamlOperator extrinsics_yaml_opr(extrinsics_path);

    CommandHandler command_handler(rosHandle, topic_command_sub, topic_command_pub);

    
    ros::Rate rate(30);


    pcl::PointCloud<pcl::PointXYZI> originalCloud; 
    
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity(); 
    Eigen::Vector3f t(0.0f, 0.0f, 0.0f); 

    extrinsics_yaml_opr.readExtrinsicsFromYaml(R, t);


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

        auto rcv_rd_packet = rd_sub.getRadar4DPacket(topic_rd_sub);
        if(!rcv_rd_packet) {
            // ROS_INFO("Waiting For Image Subscribe\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        rcv_rd_packet.frame_id = frame_id_radar;
        auto rcv_rd = std::make_shared<std::vector<Radar4DPoint>>(rcv_rd_packet.points);
        auto rcv_rd_xyzi = convertToPointCloud(rcv_rd_packet.points);
        pc_process.setCloud(rcv_rd_xyzi);
        
        pc_process.transform(R, t);

        // pcl::PointCloud<pcl::PointXYZI> fusion_cloud = *rcv_pc + *pc_process.getProcessedPointcloud();
        pc_pub.publish("/show_radar_trans_cloud", PointCloudPacket(pcl::PointCloud<pcl::PointXYZI>(*pc_process.getProcessedPointcloud()), frame_id, 0, rosTimeToTimestamp(ros::Time::now())));
        // cv::imshow("Projected Points", image);
        // int key = cv::waitKey(1); 
        int key = 0; 

        std::string command_received = command_handler.getCommand();

        if(command_received == "capture" || key == 13)
        {
            PointsPacket rcv_img_corners_packet = pts_sub.getPoints(topic_img_corners_sub);
            PointsPacket rcv_img_center_packet = pts_sub.getPoints(topic_img_center_sub);
            PointsPacket rcv_pc_corners_packet = pts_sub.getPoints(topic_pc_corners_sub);
            PointsPacket rcv_pc_center_packet = pts_sub.getPoints(topic_pc_center_sub);
            PointsPacket rcv_rd_center_packet = pts_sub.getPoints(topic_rd_center_sub);

            std::vector<geometry_msgs::Point32> img_corners_rcv = rcv_img_corners_packet.points.polygon.points;
            std::vector<geometry_msgs::Point32> img_center_rcv = rcv_img_center_packet.points.polygon.points;
            std::vector<geometry_msgs::Point32> pc_corners_rcv = rcv_pc_corners_packet.points.polygon.points;
            std::vector<geometry_msgs::Point32> pc_center_rcv = rcv_pc_center_packet.points.polygon.points;
            std::vector<geometry_msgs::Point32> rd_center_rcv = rcv_rd_center_packet.points.polygon.points;
            
            // pointset_csv_operator.writePointsToCSVAppend(pc_corners_rcv, img_corners_rcv);
            pointset_csv_operator.writePointsToCSVAppend(rd_center_rcv, pc_center_rcv);

            std::vector<geometry_msgs::Point32> rd_centers_raw;
            std::vector<geometry_msgs::Point32> pc_centers_raw;

            pointset_csv_operator.readPointsFromCSV(rd_centers_raw, pc_centers_raw);

            if(rd_centers_raw.size() < 3 || pc_centers_raw.size() < 3)
            {
                ROS_INFO("Centers Not Enough. At Least 3 Groups.\n");
                command_handler.sendCommand("capture_complete");
                command_handler.resetReceivedStatus();
                continue;
            }

            Eigen::Vector3f center_rd = CalTool::computeCentroid(rd_centers_raw);
            Eigen::Vector3f center_pc = CalTool::computeCentroid(pc_centers_raw);

            Eigen::MatrixXf rd_center_refer;
            Eigen::MatrixXf pc_center_refer;

            CalTool::alignPointsToCentroid(rd_centers_raw, center_rd, rd_center_refer);
            CalTool::alignPointsToCentroid(pc_centers_raw, center_pc, pc_center_refer);

            R = CalTool::findRotationByICP(rd_center_refer, pc_center_refer);
            t = CalTool::findTranslation(center_rd, center_pc, R);

            extrinsics_yaml_opr.writeExtrinsicsToYaml(R, t);

            geometry_msgs::PolygonStamped corners_cal;
            for(auto& pc_corner: pc_corners_rcv)
            {
                Eigen::Vector3f corner_trans(pc_corner.x, pc_corner.y, pc_corner.z);
                corner_trans = R * corner_trans + t;
                pc_corner.x = corner_trans.x();
                pc_corner.y = corner_trans.y();
                pc_corner.z = corner_trans.z();
                corners_cal.polygon.points.emplace_back(pc_corner);
            }

            pts_pub.publish(topic_corners_pub, PointsPacket(corners_cal, frame_id, 0, rosTimeToTimestamp(ros::Time::now())));

            std::cout << R << std::endl << t << std::endl;

            command_handler.sendCommand("capture_complete");
            command_handler.resetReceivedStatus();
        }
        if(command_received == "undo" || command_received == "delete_once" || key == 8)
        {
            PointsPacket rcv_img_corners_packet = pts_sub.getPoints(topic_img_corners_sub);
            PointsPacket rcv_img_center_packet = pts_sub.getPoints(topic_img_center_sub);
            PointsPacket rcv_pc_corners_packet = pts_sub.getPoints(topic_pc_corners_sub);
            PointsPacket rcv_pc_center_packet = pts_sub.getPoints(topic_pc_center_sub);
            PointsPacket rcv_rd_center_packet = pts_sub.getPoints(topic_rd_center_sub);

            std::vector<geometry_msgs::Point32> img_corners_rcv = rcv_img_corners_packet.points.polygon.points;
            std::vector<geometry_msgs::Point32> img_center_rcv = rcv_img_center_packet.points.polygon.points;
            std::vector<geometry_msgs::Point32> pc_corners_rcv = rcv_pc_corners_packet.points.polygon.points;
            std::vector<geometry_msgs::Point32> pc_center_rcv = rcv_pc_center_packet.points.polygon.points;
            std::vector<geometry_msgs::Point32> rd_center_rcv = rcv_rd_center_packet.points.polygon.points;

            std::vector<geometry_msgs::Point32> rd_centers_raw;
            std::vector<geometry_msgs::Point32> pc_centers_raw;

            pointset_csv_operator.readPointsFromCSV(rd_centers_raw, pc_centers_raw);

            if(rd_centers_raw.size() < 3 || pc_centers_raw.size() < 3)
            {
                ROS_INFO("Centers Not Enough. At Least 3 Groups.\n");
                command_handler.sendCommand("delete_once_complete");
                command_handler.resetReceivedStatus();
                continue;
            }

            Eigen::Vector3f center_rd = CalTool::computeCentroid(rd_centers_raw);
            Eigen::Vector3f center_pc = CalTool::computeCentroid(pc_centers_raw);

            Eigen::MatrixXf rd_center_refer;
            Eigen::MatrixXf pc_center_refer;

            CalTool::alignPointsToCentroid(rd_centers_raw, center_rd, rd_center_refer);
            CalTool::alignPointsToCentroid(pc_centers_raw, center_pc, pc_center_refer);

            R = CalTool::findRotationByICP(rd_center_refer, pc_center_refer);
            t = CalTool::findTranslation(center_rd, center_pc, R);

            extrinsics_yaml_opr.writeExtrinsicsToYaml(R, t);

            geometry_msgs::PolygonStamped corners_cal;
            for(auto& pc_corner: pc_corners_rcv)
            {
                Eigen::Vector3f corner_trans(pc_corner.x, pc_corner.y, pc_corner.z);
                corner_trans = R * corner_trans + t;
                pc_corner.x = corner_trans.x();
                pc_corner.y = corner_trans.y();
                pc_corner.z = corner_trans.z();
                corners_cal.polygon.points.emplace_back(pc_corner);
            }

            pts_pub.publish(topic_corners_pub, PointsPacket(corners_cal, frame_id, 0, rosTimeToTimestamp(ros::Time::now())));

            std::cout << R << std::endl << t << std::endl;

            command_handler.sendCommand("delete_once_complete");
            command_handler.resetReceivedStatus();
        }

        if (command_received == "capture_border") {
            // Step 1: 获取 rvec 和 tvec
            cv::Vec3d rvec;
            cv::Vec3d tvec;
            PointsPacket rcv_pc_corners_packet = pts_sub.getPoints(topic_pc_corners_sub);
            transform_sub.getRvecTvec(topic_trans_sub, rvec, tvec);

            auto lidar_corners = rcv_pc_corners_packet.points.polygon.points;

            cv::Mat tvec_meters = cv::Mat(tvec);  
            cv::Mat rvec_mat = cv::Mat(rvec);             

            std::vector<cv::Point3f> lidar_corners_cv;
            for (const auto& corner : lidar_corners) {
                lidar_corners_cv.emplace_back(corner.x, corner.y, corner.z);
            }

            Eigen::Affine3f transform = Eigen::Affine3f::Identity();
            transform.linear() = R;
            transform.translation() = t;
            Eigen::Matrix4f transform_matrix = transform.matrix();

            std::cout << transform_matrix << std::endl;

            if (lidar_corners_cv.empty()) {
                command_handler.sendCommand("capture_border_complete");
                command_handler.resetReceivedStatus();
                continue;
            }

            std::vector<cv::Point3f> trans_lidar_corners_cv;
            for (const auto& point : lidar_corners_cv) {
                Eigen::Vector4f lidar_point_homogeneous(point.x, point.y, point.z, 1.0f);

                Eigen::Vector4f transformed_point_homogeneous = transform_matrix * lidar_point_homogeneous;

                float x_mm = transformed_point_homogeneous.x();
                float y_mm = transformed_point_homogeneous.y();
                float z_mm = transformed_point_homogeneous.z();

                trans_lidar_corners_cv.emplace_back(x_mm, y_mm, z_mm);
            }
            
            std::vector<cv::Point2f> projected_lidar_points;
            cv::Vec3d rvec0(0, 0, 0); // 零旋转
            cv::Vec3d tvec0(0, 0, 0); // 零平移
            cv::projectPoints(trans_lidar_corners_cv, rvec0, tvec0, cameraMatrix, distCoeffs, projected_lidar_points);

            std::vector<cv::Point2f> pixel_corners;
            cv::projectPoints(image_corners, rvec, tvec, cameraMatrix, distCoeffs, pixel_corners);
            double mean_error = computeReprojectionError(projected_lidar_points, pixel_corners);
            std::cout << "Mean Reprojection Error: " << mean_error << std::endl;

            command_handler.sendCommand("capture_border_complete");
            command_handler.resetReceivedStatus();
        }

        if(key == 27) break;

        rate.sleep();

    }

    return 0;
}

