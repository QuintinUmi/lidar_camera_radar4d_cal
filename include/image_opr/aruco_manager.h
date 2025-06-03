#ifndef _ARUCO_MANAGER_H_
#define _ARUCO_MANAGER_H_

#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <unistd.h>
#include <stdlib.h>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"  
// #include "apriltag/apriltag.h"    
#include "opencv2/aruco/charuco.hpp"  

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/SVD>
  
#include <yaml-cpp/yaml.h>

#include "image_transport/image_transport.h"

#include "tools/conversion_bridge.h"

using namespace std;


namespace lcr_cal
{

    namespace image_opr
    {
        #define SUCCESS_PROCESS true
        #define FAILD_PROCESS false

        class ArucoManager{

            public:

                ArucoManager();
                ArucoManager(int dictionaryName, vector<int> selected_ids, vector<float> marker_real_length = vector<float>{1.0}, cv::Mat cameraMatrix = cv::Mat(), cv::Mat dist_coeffes = cv::Mat());
                ArucoManager(cv::Ptr<cv::aruco::Dictionary> marker_dictionary, vector<int> selected_ids, vector<float> marker_real_length = vector<float>{1.0}, cv::Mat cameraMatrix = cv::Mat(), cv::Mat dist_coeffes = cv::Mat());
                ~ArucoManager();

                void setDetectionParameters(int cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG, int adaptiveThreshWinSizeMin = 3, int adaptiveThreshWinSizeMax = 23,
                                            int adaptiveThreshWinSizeStep = 10, double adaptiveThreshConstant = 7, 
                                            double minMarkerPerimeterRate = 0.03, double maxMarkerPerimeterRate = 0.8);
                void create();
                void arucoMapInit();
                // void release();

                float getMarkerRealLength(int markerId);
                vector<int> getSelectedIds();
                int getSelectedIds(int index);

                    
                void setAruco(int dictionaryName, vector<int> selected_ids, vector<float> marker_real_length = vector<float>{1.0});
                void setAruco(cv::Ptr<cv::aruco::Dictionary> markerDictionary, vector<int> selected_ids, vector<float> marker_real_length = vector<float>{1.0});
                void selArucoIds(vector<int> selected_ids);
                void setArucoRealLength(vector<float> marker_real_length);
                void setCameraIntrinsics(cv::Mat cameraMatrix, cv::Mat dist_coeffes);

                vector<cv::Mat> generateArucoMarker(int markerSize);
                vector<cv::Mat> generateArucoMarker(int dictionaryName, vector<int> selected_ids, int markerSize);
                void generateArucoInner(int markerSize);
                void generateArucoInner(int dictionaryName, vector<int> selected_ids, int markerSize);

                void detectAruco(cv::Mat &inputImage, cv::OutputArrayOfArrays markerCorners, vector<int> markerIds);

                void extCalibSingleArucos(cv::Mat &inputImage, int targetId, vector<cv::Mat> &rvecs, vector<cv::Mat> &tvecs);
                void extCalibSingleArucos(cv::Mat &inputImage, int targetId, vector<cv::Vec3d> &rvecs, vector<cv::Vec3d> &tvecs);
                
                void extCalibMultipulArucos(cv::Mat &inputImage, vector<cv::Mat> &rvecs, vector<cv::Mat> &tvecs, vector<int>& detectedIds);
                void extCalibMultipulArucos(cv::Mat &inputImage, vector<cv::Vec3d> &rvecs, vector<cv::Vec3d> &tvecs, vector<int>& detectedIds);

                void estimateAveragePose(const vector<cv::Vec3d> &rvecs, const vector<cv::Vec3d> &tvecs, vector<int>& detectedIds, cv::Vec3d& averageRvec, cv::Vec3d& averageTvec);

                void aruco_marker_save(cv::String imageSavePath, cv::String imageFormat, vector<cv::Mat> arucoMarkerImages, int dictionaryName, bool showImage);

            private:
                
                std::map<int, int> _aruco_map;
                vector<int> _target_ids; 

                cv::Ptr<cv::aruco::Dictionary> _marker_dictionary;
                vector<int> _selected_ids;
                cv::Ptr<cv::aruco::DetectorParameters> _dParameters;
                vector<cv::Mat> _marker_image;

                vector<float> _marker_real_length;

                cv::Mat _camera_matrix;
                cv::Mat _dist_coeffes;

        };
    }
}




#endif