#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <boost/filesystem.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>  
#include <aruco/aruco.h>  
#include "opencv2/aruco/charuco.hpp"  

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <yaml-cpp/yaml.h>

#include <geometry_msgs/Point32.h>

#include "image_transport/image_transport.h"

#include "image_opr/image_process.h"
#include "image_opr/image_draw.h"
#include "image_opr/aruco_manager.h"

#include "tools/conversion_bridge.h"
#include "tools/dynamic_reconfigure.h"
#include "tools/rviz_draw.h"
#include "tools/ros_topic_manager.h"
#include "tools/timestamp_cov.h"

#include "calibration_tool.h"

#define PI 3.14159265358979324


using namespace std;
using namespace lidar_camera_cal;
using namespace lidar_camera_cal::image_opr;

int fps(int deltaTime) 
{
    int fps = static_cast<int>(1.f / deltaTime * 1000); 
    return fps;
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
    rosHandle.param("undistort", undistort, false);
    rosHandle.param("undistort_mode", undistort_mode, 0);
    rosHandle.param("frame_id", frame_id, std::string("rslidar"));
	rosHandle.param("image_process_img_sub_topic", topic_img_sub, std::string("/hikcamera/image_0/compressed"));
    rosHandle.param("image_process_img_pub_topic", topic_img_pub, std::string("/image_process/proc"));
    rosHandle.param("image_process_cor_pub_topic", topic_cor_pub, std::string("/image_process/corners"));


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

    rosHandle.param("caliboard_width", caliboard_width, (float)12.0);
    rosHandle.param("caliboard_height", caliboard_height, (float)12.0);

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
    arucos.setDetectionParameters(3);
    arucos.create();

    ImageDraw image_draw;
    if (undistort) {
        image_draw = ImageDraw(arucoRealLength[0], 1.0, 1.0, 1.0, newCameraMatrix, newDistCoeffes);
    } else {
        image_draw = ImageDraw(arucoRealLength[0], 1.0, 1.0, 1.0, cameraMatrix, distCoeffs);
    }

    RvizDraw rviz_draw("image_process_node/rviz_draw", frame_id);
    
    ImageSubscriber image_sub(rosHandle, 5);
    ImagePublisher image_pub(rosHandle);
    image_sub.addTopic(topic_img_sub);
    image_pub.addTopic(topic_img_pub, 5);

    CornersPublisher corners_pub(rosHandle);
    corners_pub.addTopic(topic_cor_pub, 5);
    

    ros::Rate rate(50);


    while(ros::ok())
    {
        ros::spinOnce();

        auto rcv_image_packet = image_sub.getImage(topic_img_sub);

        if(!rcv_image_packet) {
            // ROS_INFO("Waiting For Image Subscribe\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        rcv_image_packet.frame_id = frame_id;

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

                rviz_draw.deleteObject("line1");
                rviz_draw.deleteObject("line2");
                rviz_draw.deleteObject("line3");
                rviz_draw.deleteObject("line4");
                rviz_draw.publish();
                image_pub.publish(topic_img_pub, rcv_image_packet);
                ROS_INFO("Not Detected Center Marker!\n");
                continue;
            }
            
            center.x = tvecs[center_index][0];
            center.y = tvecs[center_index][1];
            center.z = tvecs[center_index][2];

            
            corners_plane.emplace_back(-caliboard_width/2, +caliboard_height/2, 0);
            corners_plane.emplace_back(+caliboard_width/2, +caliboard_height/2, 0);
            corners_plane.emplace_back(+caliboard_width/2, -caliboard_height/2, 0);
            corners_plane.emplace_back(-caliboard_width/2, -caliboard_height/2, 0);

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

            
            corners_plane.emplace_back(-caliboard_width/2, +caliboard_height/2, 0);
            corners_plane.emplace_back(+caliboard_width/2, +caliboard_height/2, 0);
            corners_plane.emplace_back(+caliboard_width/2, -caliboard_height/2, 0);
            corners_plane.emplace_back(-caliboard_width/2, -caliboard_height/2, 0);

            ImageProc::transform3dPoints(corners_plane, corners_3d, rvec, tvec);

            image_draw.drawOrthoCoordinate2d(proc_image, ConversionBridge::rvecs3dToMat_d(rvecs), ConversionBridge::rvecs3dToMat_d(tvecs));
            image_draw.drawLine2d(proc_image, corners_plane[0], corners_plane[1], cv::Mat(rvec), cv::Mat(tvec), cv::Scalar(0, 0, 255));
            image_draw.drawLine2d(proc_image, corners_plane[1], corners_plane[2], cv::Mat(rvec), cv::Mat(tvec), cv::Scalar(0, 0, 255));
            image_draw.drawLine2d(proc_image, corners_plane[2], corners_plane[3], cv::Mat(rvec), cv::Mat(tvec), cv::Scalar(0, 0, 255));
            image_draw.drawLine2d(proc_image, corners_plane[3], corners_plane[0], cv::Mat(rvec), cv::Mat(tvec), cv::Scalar(0, 0, 255));
            
        }


        rviz_draw.addLine("line1", corners_3d[0].x /1000, corners_3d[0].y /1000, corners_3d[0].z /1000, corners_3d[1].x /1000, corners_3d[1].y /1000, corners_3d[1].z /1000, 0.01, 1.0, 0.0, 0.0);
        rviz_draw.addLine("line2", corners_3d[1].x /1000, corners_3d[1].y /1000, corners_3d[1].z /1000, corners_3d[2].x /1000, corners_3d[2].y /1000, corners_3d[2].z /1000, 0.01, 1.0, 0.0, 0.0);
        rviz_draw.addLine("line3", corners_3d[2].x /1000, corners_3d[2].y /1000, corners_3d[2].z /1000, corners_3d[3].x /1000, corners_3d[3].y /1000, corners_3d[3].z /1000, 0.01, 1.0, 0.0, 0.0);
        rviz_draw.addLine("line4", corners_3d[3].x /1000, corners_3d[3].y /1000, corners_3d[3].z /1000, corners_3d[0].x /1000, corners_3d[0].y /1000, corners_3d[0].z /1000, 0.01, 1.0, 0.0, 0.0);
        

        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);

        cv::Mat z_axis_vector = rotation_matrix.col(2);
        Eigen::Vector3f plane_normal;
        plane_normal << -z_axis_vector.at<double>(0), -z_axis_vector.at<double>(1), -z_axis_vector.at<double>(2);

        CalTool::sortPointByNormalImgFrame(corners_3d, plane_normal);

        geometry_msgs::PolygonStamped ros_corners;
		for (const auto& corner : corners_3d) 
		{
			geometry_msgs::Point32 ros_point;
			ros_point.x = corner.x / 1000;
			ros_point.y = corner.y / 1000;
			ros_point.z = corner.z / 1000;
			ros_corners.polygon.points.push_back(ros_point);
    	}

        corners_pub.publish(topic_cor_pub, CornersPacket(ros_corners, frame_id, int(0), rosTimeToTimestamp(ros::Time::now())));

        rviz_draw.addText("corner_1", CBridge::rosPoint32ToPoint(ros_corners.polygon.points.at(0)), "1", 0.3, 1.0, 0.0, 0.0);
        rviz_draw.addText("corner_2", CBridge::rosPoint32ToPoint(ros_corners.polygon.points.at(1)), "2", 0.3, 1.0, 0.0, 0.0);
        rviz_draw.addText("corner_3", CBridge::rosPoint32ToPoint(ros_corners.polygon.points.at(2)), "3", 0.3, 1.0, 0.0, 0.0);
        rviz_draw.addText("corner_4", CBridge::rosPoint32ToPoint(ros_corners.polygon.points.at(3)), "4", 0.3, 1.0, 0.0, 0.0);

        Eigen::Vector3f plane_normal_(plane_normal.x(), plane_normal.y(), plane_normal.z());
        plane_normal_ = plane_normal_.normalized();

        Eigen::Vector3f center_point(0.0, 0.0, 0.0);
        for(const auto& corner:ros_corners.polygon.points)
        {
            center_point[0] += corner.x;
            center_point[1] += corner.y;
            center_point[2] += corner.z;
        }
        center_point /= ros_corners.polygon.points.size();

        float dotProduct = plane_normal_.dot(center_point);

        if (dotProduct > 0) 
        {
            plane_normal_ = -plane_normal_;
        }
        rviz_draw.addArrow("plane_normals", 
                            center_point.x(),
                            center_point.y(),
                            center_point.z(),
                            plane_normal_, 0.03, 0.06, 0.06, 1.0, 0.0, 0.0);

        rviz_draw.publish();

        image_pub.publish(topic_img_pub, rcv_image_packet);

        rate.sleep();

    }

    return 0;
}

