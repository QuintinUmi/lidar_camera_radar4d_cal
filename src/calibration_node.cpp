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

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <yaml-cpp/yaml.h>

#include "image_transport/image_transport.h"

#include "calibration_tool.h"

#include "image_opr/image_process.h"
#include "image_opr/image_draw.h"

#include "pointcloud2_opr/point_cloud_process.h"

#include "tools/dynamic_reconfigure.h"
#include "tools/rviz_draw.h"
#include "tools/ros_topic_manager.h"
#include "tools/conversion_bridge.h"
#include "tools/file_operator.h"
#include "tools/CommandHandler.h"


#define PI 3.14159265358979324


using namespace std;
using namespace lidar_camera_cal;
using namespace lidar_camera_cal::pointcloud2_opr;
using namespace lidar_camera_cal::image_opr;


int fps(int deltaTime) 
{
    int fps = static_cast<int>(1.f / deltaTime * 1000); 
    return fps;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "calibration_node");
    ros::NodeHandle rosHandle;

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

    rosHandle.param("frame_id", frame_id, std::string("rslidar"));

	rosHandle.param("pointcloud_process_pc_sub_topic", topic_pc_sub, std::string("/rslidar_points"));

	rosHandle.param("image_process_img_sub_topic", topic_img_sub, std::string("/hikcamera/image_0/compressed"));
    rosHandle.param("calibration_img_pub_topic", topic_img_pub, std::string("/lidar_camera_cal/image/proc"));

    rosHandle.param("pointcloud_process_cor_pub_topic", topic_pc_corners_sub, std::string("/pointcloud_process/corners"));
    rosHandle.param("image_process_cor_pub_topic", topic_img_corners_sub, std::string("/image_process/corners"));

    rosHandle.param("calibration_command_sub_topic", topic_command_sub, std::string("/lidar_camera_cal/command_controller"));
    rosHandle.param("calibration_command_pub_topic", topic_command_pub, std::string("/lidar_camera_cal/command_cal_node"));


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

    cv::Mat cameraMatrix, distCoeffs, newCameraMatrix, newDistCoeffes;
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
    rosHandle.param("pointset_save_path", cornerset_csv_path, std::string("src/lidar_camera_cal/data/point_set.csv"));
    rosHandle.param("error_analysis_save_path", error_anaylysis_csv_path, std::string("src/lidar_camera_cal/data/border_error_anaylysis.csv"));
    rosHandle.param("extrinsics_save_path", extrinsics_path, std::string("src/lidar_camera_cal/config/extrinsics.yaml"));


    PointCloudSubscriber pc_sub(rosHandle, 5);
    PointCloudPublisher pc_pub(rosHandle);
    pc_sub.addTopic(topic_pc_sub);

    ImageSubscriber img_sub(rosHandle, 5);
    ImagePublisher img_pub(rosHandle);
    img_sub.addTopic(topic_img_sub);
    img_pub.addTopic(topic_img_pub, 5);
    img_pub.addTopic("/show_fusion_cloud", 5);

    CornersSubscriber cor_sub(rosHandle, 5);
    CornersPublisher cor_pub(rosHandle);
    cor_sub.addTopic(topic_pc_corners_sub);
    cor_sub.addTopic(topic_img_corners_sub);


    PointCloud2Proc<pcl::PointXYZI> pc_process(true);

    ImageDraw image_draw(1, 1, 1, 1, cameraMatrix, distCoeffs);

    RQTConfig rqtCfg;
    // PointcloudFilterReconfigure filterRecfg(rosHandle);

    CornerSetCsvOperator cornerset_csv_operator(cornerset_csv_path);
    BorderSetCsvOperator boarderset_csv_operator(error_anaylysis_csv_path);
    // boarderset_csv_operator.writePointsToCSVOverwrite(pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>(new pcl::PointCloud<pcl::PointXYZI>), std::vector<std::vector<geometry_msgs::Point32>>());

    YamlOperator yaml_operator(extrinsics_path);

    CommandHandler command_handler(rosHandle, topic_command_sub, topic_command_pub);

    
    ros::Rate rate(30);


    pcl::PointCloud<pcl::PointXYZI> originalCloud; 
    
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity(); 
    Eigen::Vector3f t(0.0f, 0.0f, 0.0f); 

    yaml_operator.readExtrinsicsFromYaml(R, t);

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

        auto rcv_image_packet = img_sub.getImage(topic_img_sub);
        if(!rcv_image_packet) {
            // ROS_INFO("Waiting For Image Subscribe\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        rcv_image_packet.frame_id = frame_id;
        cv::Mat image = *rcv_image_packet.image;


        pc_process.transform(R, t);
        pc_process.scaleTo(1000.0f);
        pc_process.PassThroughFilter("z", 0.0, FLT_MAX);

        std::vector<cv::Point2f> imagePoints;
        image_draw.projectPointsToImage(*pc_process.getProcessedPointcloud(), imagePoints);
        image_draw.drawPointsOnImageIntensity(*pc_process.getProcessedPointcloud(), imagePoints, image);

        img_pub.publish("/show_fusion_cloud", ImagePacket(std::make_shared<cv::Mat>(image), frame_id, 0, rosTimeToTimestamp(ros::Time::now())));
        // cv::imshow("Projected Points", image);
        // int key = cv::waitKey(1); 
        int key = 0; 

        std::string command_received = command_handler.getCommand();

        if(command_received == "capture" || key == 13)
        {
            CornersPacket rcv_img_corners_packet = cor_sub.getCorners(topic_img_corners_sub);
            CornersPacket rcv_pc_corners_packet = cor_sub.getCorners(topic_pc_corners_sub);

            std::vector<geometry_msgs::Point32> img_corners_rcv = rcv_img_corners_packet.corners.polygon.points;
            std::vector<geometry_msgs::Point32> pc_corners_rcv = rcv_pc_corners_packet.corners.polygon.points;

            cornerset_csv_operator.writePointsToCSVAppend(pc_corners_rcv, img_corners_rcv);

            std::vector<geometry_msgs::Point32> pc_corners_raw;
            std::vector<geometry_msgs::Point32> img_corners_raw;

            cornerset_csv_operator.readPointsFromCSV(pc_corners_raw, img_corners_raw);

            if(pc_corners_raw.empty() || img_corners_raw.empty())
            {
                ROS_INFO("No Corners Found\n");
                continue;
            }

            Eigen::Vector3f center_pc = CalTool::computeCentroid(pc_corners_raw);
            Eigen::Vector3f center_img = CalTool::computeCentroid(img_corners_raw);

            Eigen::MatrixXf pc_center_refer;
            Eigen::MatrixXf img_center_refer;

            CalTool::alignPointsToCentroid(pc_corners_raw, center_pc, pc_center_refer);
            CalTool::alignPointsToCentroid(img_corners_raw, center_img, img_center_refer);

            R = CalTool::findRotationByICP(pc_center_refer, img_center_refer);
            t = CalTool::findTranslation(center_pc, center_img, R);

            yaml_operator.writeExtrinsicsToYaml(R, t);

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

            cor_pub.publish(topic_corners_pub, CornersPacket(corners_cal, frame_id, 0, rosTimeToTimestamp(ros::Time::now())));

            std::cout << R << std::endl << t << std::endl;

            command_handler.sendCommand("capture_complete");
            command_handler.resetReceivedStatus();
        }
        if(command_received == "undo" || command_received == "delete_once" || key == 8)
        {
            CornersPacket rcv_img_corners_packet = cor_sub.getCorners(topic_img_corners_sub);
            CornersPacket rcv_pc_corners_packet = cor_sub.getCorners(topic_pc_corners_sub);

            std::vector<geometry_msgs::Point32> img_corners_rcv = rcv_img_corners_packet.corners.polygon.points;
            std::vector<geometry_msgs::Point32> pc_corners_rcv = rcv_pc_corners_packet.corners.polygon.points;

            std::vector<geometry_msgs::Point32> pc_corners_raw;
            std::vector<geometry_msgs::Point32> img_corners_raw;

            cornerset_csv_operator.readPointsFromCSV(pc_corners_raw, img_corners_raw);

            if(pc_corners_raw.empty() || img_corners_raw.empty())
            {
                ROS_INFO("No Corners Found\n");
                continue;
            }

            Eigen::Vector3f center_pc = CalTool::computeCentroid(pc_corners_raw);
            Eigen::Vector3f center_img = CalTool::computeCentroid(img_corners_raw);

            Eigen::MatrixXf pc_center_refer;
            Eigen::MatrixXf img_center_refer;

            CalTool::alignPointsToCentroid(pc_corners_raw, center_pc, pc_center_refer);
            CalTool::alignPointsToCentroid(img_corners_raw, center_img, img_center_refer);

            R = CalTool::findRotationByICP(pc_center_refer, img_center_refer);
            t = CalTool::findTranslation(center_pc, center_img, R);

            yaml_operator.writeExtrinsicsToYaml(R, t);

            geometry_msgs::PolygonStamped corners_cal;
            for(auto pc_corner: pc_corners_rcv)
            {
                Eigen::Vector3f corner_trans(pc_corner.x, pc_corner.y, pc_corner.z);
                corner_trans = R * corner_trans + t;
                pc_corner.x = corner_trans.x();
                pc_corner.y = corner_trans.y();
                pc_corner.z = corner_trans.z();
                corners_cal.polygon.points.emplace_back(pc_corner);
            }

            cor_pub.publish(topic_corners_pub, CornersPacket(corners_cal, frame_id, 0, rosTimeToTimestamp(ros::Time::now())));

            std::cout << R << std::endl << t << std::endl;

            command_handler.sendCommand("delete_once_complete");
            command_handler.resetReceivedStatus();
        }

        // if(command_received == "capture_border")
        // {
            
        //     std::vector<geometry_msgs::Point32> img_corners_rcv = img_corners_SUB_PUB.getCornersPoints32();
        //     std::vector<geometry_msgs::Point32> pc_corners_rcv = pc_corners_SUB_PUB.getCornersPoints32();
        //     pc_process.setCloud(pc_SUB_PUB.getPointcloudXYZI());
        //     pc_process.boxFilter(Eigen::Vector3f(center_x, center_y, center_z), length_x, length_y, length_z, rotate_x, rotate_y, rotate_z);
        //     pc_process.normalClusterExtraction();
        //     pc_process.extractNearestClusterCloud();
        //     pc_process.planeSegmentation();

        //     // test caliboard_pc
        //     PointCloud2Proc<pcl::PointXYZI> caliboard_pc_proc(true);
        //     caliboard_pc_proc.setCloud(pc_process.getProcessedPointcloud());
        //     caliboard_pc_proc.transform(R, t);
        //     caliboard_pc_proc.scaleTo(1000.0f);
        //     caliboard_pc_proc.PassThroughFilter("z", 0, 4000);

        //     std::vector<cv::Point2f> caliboard_pc;
        //     auto img_caliboard_pc = img_SUB_PUB.getImage();
        //     image_draw.projectPointsToImage(*caliboard_pc_proc.getProcessedPointcloud(), caliboard_pc);
        //     image_draw.drawPointsOnImageZ(*caliboard_pc_proc.getProcessedPointcloud(), caliboard_pc, img_caliboard_pc);
        //     cv::imshow("caliboard_pc", img_caliboard_pc);
        //     // test caliboard_pc //


        //     pcl::PointCloud<pcl::PointXYZI>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        //     pcl::copyPointCloud(*pc_process.calculateConcaveHull(pc_process.getProcessedPointcloud(), concave_hull_alpha), *hull_cloud);
        //     CalTool::removeBoundingBoxOutliers<pcl::PointXYZI>(hull_cloud, pc_corners_rcv);
        //     pc_process.setCloud(hull_cloud);

        //     boarderset_csv_operator.writePointsToCSVAppend(hull_cloud, img_corners_rcv);


        //     // pc_process.extractConvexHull();
        //     pc_process.transform(R, t);
        //     pc_process.scaleTo(1000.0f);
        //     pc_process.PassThroughFilter("z", 0, 4000);

        //     std::vector<cv::Point2f> imagePoints;
        //     auto img_hull = img_SUB_PUB.getImage();
        //     image_draw.projectPointsToImage(*pc_process.getProcessedPointcloud(), imagePoints);
        //     image_draw.drawPointsOnImageZ(*pc_process.getProcessedPointcloud(), imagePoints, img_hull);


        //     cv::imshow("concave_hull_cloud", img_hull);

            

        //     pcl::PointCloud<pcl::PointXYZI>::Ptr border_clouds(new pcl::PointCloud<pcl::PointXYZI>);
        //     std::vector<std::vector<geometry_msgs::Point32>> image_corner_sets;
        //     boarderset_csv_operator.readPointsFromCSV(border_clouds, image_corner_sets);
        //     // double reprojection_error = CalTool::computeReprojectionErrors(hull_cloud, img_corners_raw, R, t, newCameraMatrix, newDisCoffes);
        //     double pixel_mean_error, pixel_stddev_error;
        //     CalTool::computeReprojectionErrorsInPixels<pcl::PointXYZI>(border_clouds, image_corner_sets, R, t, newCameraMatrix, newDisCoffes, pixel_mean_error, pixel_stddev_error);
            

        //     // std::cout << "Reprojection Errors = " << reprojection_error << " mm" << std::endl;
        //     std::cout << "Reprojection Mean Errors In Pixels = " << pixel_mean_error << " pixels" << std::endl;
        //     std::cout << "Reprojection Standard Deviation Errors In Pixels = " << pixel_stddev_error << " pixels" << std::endl;


        //     command_handler.sendCommand("capture_border_complete");
        //     command_handler.resetReceivedStatus();
        // }




        if(key == 27) break;
        
        

        rate.sleep();

    }

    return 0;
}

