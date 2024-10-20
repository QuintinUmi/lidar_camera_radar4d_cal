#include "image_opr/aruco_manager.h"

using namespace std;
using namespace lidar_camera_cal::image_opr;

ArucoManager::ArucoManager() {
    _dParameters = cv::aruco::DetectorParameters::create();
}
ArucoManager::ArucoManager(int dictionaryName, vector<int> selected_ids, vector<float> marker_real_length, cv::Mat camera_matrix, cv::Mat dist_coeffes) {   
    cv::aruco::DICT_6X6_1000;
    _marker_dictionary = cv::aruco::getPredefinedDictionary(dictionaryName);
    _selected_ids = selected_ids;
    _dParameters = cv::aruco::DetectorParameters::create();
    for(int i = 0; i < marker_real_length.size(); i++)
    {
        _marker_real_length.emplace_back(marker_real_length[i]);
    }
    setCameraIntrinsics(camera_matrix, dist_coeffes);
}
ArucoManager::ArucoManager(cv::Ptr<cv::aruco::Dictionary> marker_dictionary, vector<int> selected_ids, vector<float> marker_real_length, cv::Mat camera_matrix, cv::Mat dist_coeffes) {
    _marker_dictionary = marker_dictionary;
    _selected_ids = selected_ids;
    _dParameters = cv::aruco::DetectorParameters::create();
    for(int i = 0; i < marker_real_length.size(); i++)
    {
        _marker_real_length.emplace_back(marker_real_length[i]);
    }
    setCameraIntrinsics(camera_matrix, dist_coeffes);
}
ArucoManager::~ArucoManager()
{
    // if(aruco_hash != NULL){
    //     int* tmp = aruco_hash;
    //     aruco_hash = NULL;
    //     delete tmp;
    // }
}

void ArucoManager::setDetectionParameters(int cornerRefinementMethod, int adaptiveThreshWinSizeMin, int adaptiveThreshWinSizeMax,
                                    int adaptiveThreshWinSizeStep, int adaptiveThreshConstant, 
                                    int minMarkerPerimeterRate, int maxMarkerPerimeterRate)
{
    if (!_dParameters) {
        std::cerr << "Detection parameters are not initialized." << std::endl;
        return;
    }
    
    _dParameters->cornerRefinementMethod = cornerRefinementMethod;

    _dParameters->adaptiveThreshWinSizeMin = adaptiveThreshWinSizeMin;
    _dParameters->adaptiveThreshWinSizeMax = adaptiveThreshWinSizeMax;
    _dParameters->adaptiveThreshWinSizeStep = adaptiveThreshWinSizeStep;
    _dParameters->adaptiveThreshConstant = adaptiveThreshConstant;

    _dParameters->minMarkerPerimeterRate = minMarkerPerimeterRate;
    _dParameters->maxMarkerPerimeterRate = maxMarkerPerimeterRate;
}

void ArucoManager::create()
{
    arucoMapInit();
}

void ArucoManager::arucoMapInit()
{
    for(int i = 0; i < _selected_ids.size(); i++)
    {
        _aruco_map[_selected_ids[i]] = i;
    }
    
    printf("Aruco Map Init Success!\n");
}

// void ArucoManager::release()
// {
//     if(aruco_hash != NULL){
//         int* tmp = aruco_hash;
//         aruco_hash = NULL;
//         delete tmp;
//     }
// }


float ArucoManager::getMarkerRealLength(int markerId)
{
    if(_aruco_map.find(markerId) == _aruco_map.end())
    {
        return 1.0;
    }
    return _marker_real_length[_aruco_map.find(markerId)->second];
}
vector<int> ArucoManager::getSelectedIds()
{
    return _selected_ids;
}
int ArucoManager::getSelectedIds(int index)
{
    return _selected_ids.at(index);
}


void ArucoManager::setAruco(int dictionaryName, vector<int> selected_ids, vector<float> marker_real_length)
{
    _marker_dictionary = cv::aruco::getPredefinedDictionary(dictionaryName);
    _selected_ids = selected_ids;
    _dParameters = cv::aruco::DetectorParameters::create();
    for(int i = 0; i < marker_real_length.size(); i++)
    {
        _marker_real_length.emplace_back(marker_real_length[i]);
    }
}
void ArucoManager::setAruco(cv::Ptr<cv::aruco::Dictionary> marker_dictionary, vector<int> selected_ids, vector<float> marker_real_length)
{
    _marker_dictionary = marker_dictionary;
    _selected_ids = selected_ids;
    _dParameters = cv::aruco::DetectorParameters::create();
    for(int i = 0; i < marker_real_length.size(); i++)
    {
        _marker_real_length.emplace_back(marker_real_length[i]);
    }
}
void ArucoManager::selArucoIds(vector<int> selected_ids)
{
    _selected_ids.clear();
    _selected_ids = selected_ids;
    arucoMapInit();
}
void ArucoManager::setArucoRealLength(vector<float> marker_real_length)
{
    for(int i = 0; i < marker_real_length.size(); i++)
    {
        _marker_real_length.emplace_back(marker_real_length[i]);
    }
}
void ArucoManager::setCameraIntrinsics(cv::Mat camera_matrix, cv::Mat dist_coeffes)
{
    _camera_matrix = camera_matrix;
    _dist_coeffes = dist_coeffes;
}


vector<cv::Mat> ArucoManager::generateArucoMarker(int markerSize)
{
    vector<cv::Mat> imgArucoMarker;
    for(int i = 0; i < _selected_ids.size(); i++)
    { 
        cv::Mat temp;
        cv::aruco::drawMarker(_marker_dictionary, _selected_ids[i], markerSize, temp);
        imgArucoMarker.emplace_back(temp);
    }
    
    return imgArucoMarker;
}
vector<cv::Mat> ArucoManager::generateArucoMarker(int dictionaryName, vector<int> selected_ids, int markerSize)
{
    cv::Ptr<cv::aruco::Dictionary> marker_dictionary = cv::aruco::getPredefinedDictionary(dictionaryName);
    vector<cv::Mat> imgArucoMarker;
    for(int i = 0; i < selected_ids.size(); i++)
    { 
        cv::Mat temp;
        cv::aruco::drawMarker(marker_dictionary, selected_ids[i], markerSize, temp);
        imgArucoMarker.emplace_back(temp);
    }
    
    return imgArucoMarker;
}
void ArucoManager::generateArucoInner(int markerSize)
{
    for(int i = 0; i < _selected_ids.size(); i++)
    { 
        cv::Mat temp;
        cv::aruco::drawMarker(_marker_dictionary, _selected_ids[i], markerSize, temp);
        _marker_image.emplace_back(temp);
    }
}
void ArucoManager::generateArucoInner(int dictionaryName, vector<int> selected_ids, int markerSize)
{
    _marker_dictionary = cv::aruco::getPredefinedDictionary(dictionaryName);
    for(int i = 0; i < _selected_ids.size(); i++)
    { 
        cv::Mat temp;
        cv::aruco::drawMarker(_marker_dictionary, _selected_ids[i], markerSize, temp);
        _marker_image.emplace_back(temp);
    }
}


void ArucoManager::detectAruco(cv::Mat &inputImage, cv::OutputArrayOfArrays markerCorners, vector<int> markerIds)
{
    // cv::aruco::DetectorParameters testParameters;
    // testParameters.minDistanceToBorder = 0;
    // testParameters.adaptiveThreshWinSizeMax = 1500;
    // cv::Mat arucoTest;
    // cv::aruco::drawMarker(_marker_dictionary, 4, 500, arucoTest);
    vector<vector<cv::Point2f>> rejectPoints;
    cv::aruco::detectMarkers(inputImage, cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250), markerCorners, markerIds);
    // if (markerIds.size() > 0)
    //     cv::aruco::drawDetectedMarkers(inputImage, markerCorners, markerIds);
    
    // std::cout << rejectPoints.empty() << std::endl;
}

void ArucoManager::extCalibSingleArucos(cv::Mat &inputImage, int targetId, 
                                    vector<cv::Mat> &rvecs, vector<cv::Mat> &tvecs)
{
    vector<vector<cv::Point2f>> markerCorners, selectedCorners;
    vector<int> markerIds;
    int indexId;
    vector<cv::Vec3d> rvecs3d, tvecs3d;
    cv::Mat vecData;
    
    rvecs.clear();
    tvecs.clear();
    cv::aruco::detectMarkers(inputImage, _marker_dictionary, markerCorners, markerIds, _dParameters);
    if(markerIds.empty())
    {
        // printf("No marker detected!\n");
        return;
    }
    for(indexId = 0; indexId < markerIds.size(); indexId++){
        // printf("Marker %d check!\n", markerIds[indexId]);
        if(markerIds[indexId] == targetId)
            selectedCorners.emplace_back(markerCorners[indexId]);
    }
    if(!selectedCorners.empty())
    {
        // std::cout << _camera_matrix << std::endl;
        std::cout << markerIds[indexId] << std::endl;
        cv::aruco::estimatePoseSingleMarkers(selectedCorners, _marker_real_length[_aruco_map.find(targetId)->second], _camera_matrix, _dist_coeffes, rvecs3d, tvecs3d);
        int vecSize = selectedCorners.size();
        for(int j = 0; j < vecSize; j++)
        {
            rvecs.emplace_back((cv::Mat_<float>(3, 1) << rvecs3d[j][0], rvecs3d[j][1], rvecs3d[j][2]));
            tvecs.emplace_back((cv::Mat_<float>(3, 1) << tvecs3d[j][0], tvecs3d[j][1], tvecs3d[j][2]));
        }
    }
    else
    {
        rvecs = vector<cv::Mat>{};
        tvecs = vector<cv::Mat>{};
    }
    
}

void ArucoManager::extCalibSingleArucos(cv::Mat &inputImage, int targetId, 
                                    vector<cv::Vec3d> &rvecs, vector<cv::Vec3d> &tvecs)
{
    vector<vector<cv::Point2f>> markerCorners, selectedCorners;
    vector<int> markerIds;
    int indexId;
    vector<cv::Vec3d> rvecs3d, tvecs3d;
    cv::Mat vecData;
    
    rvecs.clear();
    tvecs.clear();
    cv::aruco::detectMarkers(inputImage, _marker_dictionary, markerCorners, markerIds, _dParameters);
    if(markerIds.empty())
    {
        // printf("No marker detected!\n");
        return;
    }
    for(indexId = 0; indexId < markerIds.size(); indexId++){
        // printf("Marker %d check!\n", markerIds[indexId]);
        if(markerIds[indexId] == targetId)
            selectedCorners.emplace_back(markerCorners[indexId]);
    }
    if(!selectedCorners.empty())
    {
        // std::cout << _camera_matrix << std::endl;
        std::cout << markerIds[indexId] << std::endl;
        cv::aruco::estimatePoseSingleMarkers(selectedCorners, _marker_real_length[_aruco_map.find(targetId)->second], _camera_matrix, _dist_coeffes, rvecs3d, tvecs3d);
        int vecSize = selectedCorners.size();
        for(int j = 0; j < vecSize; j++)
        {
            rvecs.emplace_back(rvecs3d[j]);
            tvecs.emplace_back(rvecs3d[j]);
        }
    }
    else
    {
        rvecs = vector<cv::Vec3d>{};
        tvecs = vector<cv::Vec3d>{};
    }
    
}


void ArucoManager::extCalibMultipulArucos(cv::Mat &inputImage, vector<cv::Mat> &rvecs, vector<cv::Mat> &tvecs, vector<int>& detectedIds)
{
    vector<vector<cv::Point2f>> markerCorners;
    vector<vector<vector<cv::Point2f>>> selectedCorners;
    vector<int> markerIds;
    int indexId;
    cv::Mat vecData;
    
    rvecs.clear();
    tvecs.clear();
    detectedIds.clear();
    cv::aruco::detectMarkers(inputImage, _marker_dictionary, markerCorners, markerIds, _dParameters);
    if(markerIds.empty())
    {
        printf("No marker detected!\n");
        rvecs = vector<cv::Mat>{};
        tvecs = vector<cv::Mat>{};
        return;
    }
    for(indexId = 0; indexId < markerIds.size(); indexId++){
        printf("Marker %d find!\n", markerIds[indexId]);
        if(_aruco_map.find(markerIds[indexId]) != _aruco_map.end())
        {
            printf("Marker %d is selected!\n", markerIds[indexId]);
            std::vector<std::vector<cv::Point2f>> temp_markerCorners;
            temp_markerCorners.emplace_back(markerCorners[indexId]);
            selectedCorners.emplace_back(temp_markerCorners);
            detectedIds.emplace_back(markerIds[indexId]);           
        }
        
    }
    if(!detectedIds.empty())
    {

        vector<cv::Vec3d> rvecs3d, tvecs3d;
        for(int detectedIds_index = 0; detectedIds_index < detectedIds.size(); detectedIds_index++)
        {
            cv::aruco::estimatePoseSingleMarkers(selectedCorners[detectedIds_index], _marker_real_length[_aruco_map.find(markerIds[detectedIds_index])->second], _camera_matrix, _dist_coeffes, rvecs3d, tvecs3d);
            
            int vecSize = selectedCorners[detectedIds_index].size();
            for(int j = 0; j < vecSize; j++)
            {
                rvecs.emplace_back((cv::Mat_<float>(3, 1) << rvecs3d[j][0], rvecs3d[j][1], rvecs3d[j][2]));
                tvecs.emplace_back((cv::Mat_<float>(3, 1) << tvecs3d[j][0], tvecs3d[j][1], tvecs3d[j][2]));
            }
        }
        
    }
    else
    {
        std::cout << "Detection Empty!\n" << std::endl;
        rvecs = vector<cv::Mat>{};
        tvecs = vector<cv::Mat>{};
    }
    
}

void ArucoManager::extCalibMultipulArucos(cv::Mat &inputImage, vector<cv::Vec3d> &rvecs, vector<cv::Vec3d> &tvecs, vector<int>& detectedIds)
{
    vector<vector<cv::Point2f>> markerCorners;
    vector<vector<vector<cv::Point2f>>> selectedCorners;
    vector<int> markerIds;
    int indexId;
    cv::Mat vecData;
    
    rvecs.clear();
    tvecs.clear();
    detectedIds.clear();
    cv::aruco::detectMarkers(inputImage, _marker_dictionary, markerCorners, markerIds, _dParameters);
    if(markerIds.empty())
    {
        printf("No marker detected!\n");
        rvecs = vector<cv::Vec3d>{};
        tvecs = vector<cv::Vec3d>{};
        return;
    }
    for(indexId = 0; indexId < markerIds.size(); indexId++){
        // printf("Marker %d find!\n", markerIds[indexId]);
        if(_aruco_map.find(markerIds[indexId]) != _aruco_map.end())
        {
            // printf("Marker %d is selected!\n", markerIds[indexId]);
            std::vector<std::vector<cv::Point2f>> temp_markerCorners;
            temp_markerCorners.emplace_back(markerCorners[indexId]);
            selectedCorners.emplace_back(temp_markerCorners);
            detectedIds.emplace_back(markerIds[indexId]);           
        }
        
    }
    if(!detectedIds.empty())
    {

        vector<cv::Vec3d> rvecs3d, tvecs3d;
        for(int detectedIds_index = 0; detectedIds_index < detectedIds.size(); detectedIds_index++)
        {
            cv::aruco::estimatePoseSingleMarkers(selectedCorners[detectedIds_index], _marker_real_length[_aruco_map.find(markerIds[detectedIds_index])->second], _camera_matrix, _dist_coeffes, rvecs3d, tvecs3d);
            
            int vecSize = selectedCorners[detectedIds_index].size();
            for(int j = 0; j < vecSize; j++)
            {
                rvecs.emplace_back(rvecs3d[j]);
                tvecs.emplace_back(tvecs3d[j]);
            }
        }
        
    }
    else
    {
        std::cout << "Detection Empty!\n" << std::endl;
        rvecs = vector<cv::Vec3d>{};
        tvecs = vector<cv::Vec3d>{};
    }
    
}

void ArucoManager::estimateAveragePose(const vector<cv::Vec3d> &rvecs, const vector<cv::Vec3d> &tvecs, vector<int>& detectedIds, cv::Vec3d& averageRvec, cv::Vec3d& averageTvec)
{
    int count = 0;
    cv::Vec3d sumTvecs(0, 0, 0);
    for(const auto& tvec : tvecs) 
    {
        sumTvecs += tvec;
        count ++;
    }
    averageTvec = cv::Vec3d(sumTvecs[0] / count, sumTvecs[1] / count, sumTvecs[2] / count);


    std::vector<Eigen::Quaterniond> quaternions = ConversionBridge::rvecs3dToQuaternions(rvecs);

    Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
    for (const auto& q : quaternions) {
        Eigen::Vector4d q_vec(q.w(), q.x(), q.y(), q.z());
        M += q_vec * q_vec.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigen_decomposition(M);
    auto eigen_vectors = eigen_decomposition.eigenvectors();
    auto eigen_values = eigen_decomposition.eigenvalues();

    int max_index = 0;
    double max_eigen_values = eigen_values(0);
    for(int i = 1; i <= 4; i++)
    {
        if(eigen_values(i) > max_eigen_values) 
        {
            max_eigen_values = eigen_values(i);
            max_index = i;
        }
    }

    Eigen::Vector4d max_eigen_vector = eigen_vectors.col(max_index);

    averageRvec = ConversionBridge::quaternionToRvec3d(Eigen::Quaterniond(max_eigen_vector(0), max_eigen_vector(1), max_eigen_vector(2), max_eigen_vector(3)).normalized());

}


void ArucoManager::aruco_marker_save(cv::String imageSavePath, cv::String imageFormat, vector<cv::Mat> arucoMarkerImages, 
                                int dictionaryName, bool showImage)
{
    cv::Mat processImg(arucoMarkerImages[0].size(), CV_8U, cv::Scalar(255));
    cv::Ptr<cv::aruco::Dictionary> _marker_dictionary = cv::aruco::getPredefinedDictionary(dictionaryName);
    cv::String saveFileName;
    vector<vector<cv::Point2f>> markerCorners;
    vector<int> markerIds;
    // ImageDraw d3d;
    vector<int> markerIdss;
    markerIdss.emplace_back(5);
    // arucoMarkerImages = generateArucoMarker(dictionaryName, markerIdss, 500);
    for(int i = 0; i < arucoMarkerImages.size(); i++)
    { 
        processImg = cv::Mat(arucoMarkerImages[i].size(), CV_8U, cv::Scalar(255));
        
        // d3d.center_image_scale(arucoMarkerImages[i], processImg, 0.5, 0.5, 1, 0, cv::Scalar(255));


        float srcH = arucoMarkerImages[i].size().height;
        float srcW = arucoMarkerImages[i].size().width;
        float cX = srcW / 2;
        float cY = srcH / 2;

        float scaleX = 0.5;
        float scaleY = 0.5;
        int flags = 1;
        int borderMode = 0;
        cv::Scalar borderValue(255);

        cv::Point2f srcP[] = {  cv::Point2f(srcW, 0), 
                                cv::Point2f(srcW ,srcH), 
                                cv::Point2f(0 ,srcH)
                            };

        cv::Point2f dstP[] = {  cv::Point2f((srcW - cX) * scaleX + cX, (0 - cY) * scaleY + cY), 
                                cv::Point2f((srcW - cX) * scaleX + cX, (srcH - cY) * scaleY + cY), 
                                cv::Point2f((0 - cX) * scaleX + cX, (srcH - cY) * scaleY + cY)
                            };

        cv::Mat warpM = cv::getAffineTransform(srcP, dstP);
        cv::warpAffine(arucoMarkerImages[i], processImg, warpM, arucoMarkerImages[i].size(), flags, borderMode, borderValue);


        cv::aruco::detectMarkers(processImg, _marker_dictionary, markerCorners, markerIds);
        saveFileName = imageSavePath + cv::String("id_") + std::to_string(markerIds[0]) + 
                                        cv::String("--dictName_") + std::to_string(dictionaryName) + 
                                        cv::String(".") + imageFormat;
        // saveFileName = to_string(i) + cv::String(".png");
        cv::imwrite(saveFileName, arucoMarkerImages[i]);
        if(showImage){
            cv::imshow(cv::String("id_") + std::to_string(markerIds[0]) + 
                        cv::String("--dictName_") + std::to_string(dictionaryName) + 
                        cv::String(".") + imageFormat, 
                        arucoMarkerImages[i]);
            cv::waitKey(0);
        }
    }
}