#include "image_opr/image_draw.h"

#define PI 3.14159265358979324

using namespace std;
using namespace lidar_camera_cal::image_opr;


// ImageDraw::ImageDraw(){
//     scaleX = 1.0;
//     scaleY = 1.0;
//     scaleZ = 1.0;
// }
ImageDraw::ImageDraw(float unitLength ,float scaleX, float scaleY, float scaleZ, cv::Mat cameraMatrix, cv::Mat distCoeffs)
{
    unitLength = unitLength;
    scaleX = scaleX;
    scaleY = scaleY;
    scaleZ = scaleZ;
    setCameraMatrix = cameraMatrix;
    setDisCoffes = distCoeffs;

}
ImageDraw::ImageDraw(cv::Mat cameraMatrix, cv::Mat distCoeffs)
{
    setCameraMatrix = cameraMatrix;
    setDisCoffes = distCoeffs;
}

void ImageDraw::setUnitLen(float unitLength){
    unitLength;
}
void ImageDraw::setScale(float scaleX, float scaleY, float scaleZ)
{
    scaleX = scaleX;
    scaleY = scaleY;
    scaleZ = scaleZ;
}


void ImageDraw::writeIn(cv::Point3f &dst, float x, float y, float z)
{
    dst.x = x * scaleX;
    dst.y = y * scaleY;
    dst.z = z * scaleZ;
}
void ImageDraw::writeIn(vector<cv::Point3f> &dst, float x, float y, float z)
{
    dst.emplace_back(x * scaleX, y * scaleY, z * scaleZ);
}
void ImageDraw::writeIn(vector<cv::Point3f> &dst, cv::Point3f point)
{
    point = cv::Point3f(point.x * scaleX, point.y * scaleX, point.z * scaleX);
    dst.emplace_back(point);
}

cv::Mat ImageDraw::getCameraMatrix()
{
    return setCameraMatrix;
}
cv::Mat ImageDraw::getDisCoffes()
{
    return setDisCoffes;
}

vector<vector<cv::Point3f>> ImageDraw::drawOrthoCoordinate3d(cv::Point3f centerPoint, float density)
{
    vector<vector<cv::Point3f>> coordinate(4);
    float cx = centerPoint.x;
    float cy = centerPoint.y;
    float cz = centerPoint.z;
    writeIn(coordinate[0], cx, cy, cz);
    int pointNum;
    pointNum = unitLength / density;
    for(int i = 1; i <= pointNum; i++)
        writeIn(coordinate[1], cx + i * density, cy, cz);
    for(int i = 1; i <= pointNum; i++)
        writeIn(coordinate[2], cx, cy + i * density, cz);
    for(int i = 1; i <= pointNum; i++)
        writeIn(coordinate[3], cx, cy, cz + i * density);

    return coordinate;
}
vector<vector<cv::Point3f>> ImageDraw::drawOrthoCoordinate3d(float cx, float cy, float cz, float density)
{
    vector<vector<cv::Point3f>> coordinate(4);
    writeIn(coordinate[0], cx, cy, cz);
    int pointNum;
    pointNum = unitLength / density;
    for(int i = 1; i <= pointNum; i++)
        writeIn(coordinate[1], cx + i * density, cy, cz);
    for(int i = 1; i <= pointNum; i++)
        writeIn(coordinate[2], cx, cy + i * density, cz);
    for(int i = 1; i <= pointNum; i++)
        writeIn(coordinate[3], cx, cy, cz + i * density);

    return coordinate;
}
void ImageDraw::drawOrthoCoordinate2d(cv::Mat &imgInputOutput, cv::Mat rvec, cv::Mat tvec,
                                        float scale, float cx, float cy, float cz, cv::Mat cameraMatrix, cv::Mat distCoeffs)
{
    if(cameraMatrix.empty() || distCoeffs.empty())
    {
        if(setCameraMatrix.empty() || setDisCoffes.empty())
        {
            return;
        }
        cameraMatrix = setCameraMatrix;
        distCoeffs = setDisCoffes;
    }
    vector<cv::Point3f> p3d;
    writeIn(p3d, cx, cy, cz);
    writeIn(p3d, cx + unitLength * scale, cy, cz);
    writeIn(p3d, cx, cy + unitLength * scale, cz);
    writeIn(p3d, cx, cy, cz + unitLength * scale);

    vector<cv::Point2f> p2d;
    cv::projectPoints(p3d, rvec, tvec, cameraMatrix, distCoeffs, p2d);
    
    cv::line(imgInputOutput, p2d[0], p2d[1], cv::Scalar(0, 0, 255), 3);
    cv::line(imgInputOutput, p2d[0], p2d[2], cv::Scalar(0, 255, 0), 3);
    cv::line(imgInputOutput, p2d[0], p2d[3], cv::Scalar(255, 0, 0), 3);

}

void ImageDraw::drawOrthoCoordinate2d(cv::Mat &imgInputOutput, vector<cv::Mat> rvecs, vector<cv::Mat> tvecs, 
                                        float scale, float cx, float cy, float cz, cv::Mat cameraMatrix, cv::Mat distCoeffs)
{
    if(cameraMatrix.empty() || distCoeffs.empty())
    {
        if(setCameraMatrix.empty() || setDisCoffes.empty())
        {
            return;
        }
        cameraMatrix = setCameraMatrix;
        distCoeffs = setDisCoffes;
    }
    vector<cv::Point3f> p3d;
    writeIn(p3d, cx, cy, cz);
    writeIn(p3d, cx + unitLength * scale, cy, cz);
    writeIn(p3d, cx, cy + unitLength * scale, cz);
    writeIn(p3d, cx, cy, cz + unitLength * scale);

    for(int i = 0; i < rvecs.size() && i < tvecs.size(); i++)
    {
        vector<cv::Point2f> p2d;
        cv::projectPoints(p3d, rvecs.at(i), tvecs.at(i), cameraMatrix, distCoeffs, p2d);
        
        cv::line(imgInputOutput, p2d[0], p2d[1], cv::Scalar(0, 0, 255), 3);
        cv::line(imgInputOutput, p2d[0], p2d[2], cv::Scalar(0, 255, 0), 3);
        cv::line(imgInputOutput, p2d[0], p2d[3], cv::Scalar(255, 0, 0), 3);
        p2d.clear();
    }
}


void ImageDraw::transform3dPoints(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, cv::Mat rvec, cv::Mat tvec)
{
    if(rvec.empty()){
        rvec = (cv::Mat_<float>(3, 1) << 0, 0, 0);
    }
    if(tvec.empty()){
        tvec = (cv::Mat_<float>(3, 1) << 0, 0, 0);
    }
    cv::Mat R;
    cv::Mat srcMatrix, dstMatrix;
    cv::Rodrigues(rvec, R);
    size_t t = srcWorldPoints.size();
    for(int i = 0; i < t; i++)
    {
        srcMatrix = cv::Mat(srcWorldPoints[i]);
        dstMatrix = R * srcMatrix + tvec;
        if(srcWorldPoints == newWorldPoints){
            newWorldPoints[i] = cv::Point3f(dstMatrix);
        }
        else{
            newWorldPoints.emplace_back(dstMatrix);
        }
    }
    
}
void ImageDraw::transform3dPoints(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, 
                                        float rx, float ry, float rz, float tx, float ty, float tz)
{
    cv::Mat rvec = (cv::Mat_<float>(3, 1) << rx, ry, rz), tvec = (cv::Mat_<float>(3, 1) << tx, ty, tz);
    cv::Mat R;
    cv::Mat srcMatrix, dstMatrix;
    cv::Rodrigues(rvec, R);
    size_t t = srcWorldPoints.size();
    for(int i = 0; i < t; i++)
    {
        srcMatrix = cv::Mat(srcWorldPoints[i]);
        dstMatrix = R * srcMatrix + tvec;
        if(srcWorldPoints == newWorldPoints){
            newWorldPoints[i] = cv::Point3f(dstMatrix);
        }
        else{
            newWorldPoints.emplace_back(dstMatrix);
        }
    }
}


void ImageDraw::mirror3dPoints(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, cv::Point3f surfaceNorVec)
{
    float len = pow(surfaceNorVec.x, 2) + pow(surfaceNorVec.y, 2) + pow(surfaceNorVec.z, 2);
    float uX = surfaceNorVec.x / len;
    float uY = surfaceNorVec.y / len;
    float uZ = surfaceNorVec.z / len;

    cv::Mat q = (cv::Mat_<float>(3, 3) <<   1-2*pow(uX, 2), -2*uX*uY, -2*uX*uZ,
                                            -2*uX*uY, 1-2*pow(uY, 2), -2*uY*uZ,
                                            -2*uX*uZ, -2*uY*uZ, 1-2*pow(uZ, 2));

    cv::Mat srcMatrix, dstMatrix;
    size_t t = srcWorldPoints.size();
    for(int i = 0; i < t; i++)
    {
        srcMatrix = cv::Mat(srcWorldPoints[i]);
        dstMatrix = q * srcMatrix;
        if(srcWorldPoints == newWorldPoints){
            newWorldPoints[i] = cv::Point3f(dstMatrix);
        }
        else{
            newWorldPoints.emplace_back(dstMatrix);
        }
            
    }
}
void ImageDraw::mirror3dPoints(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, float surNorX, float surNorY, float surNorZ)
{
    cv::Mat surfaceNorVec;
    float len = pow(surNorX, 2) + pow(surNorY, 2) + pow(surNorZ, 2);
    float uX = surNorX / len;
    float uY = surNorY / len;
    float uZ = surNorZ / len;

    cv::Mat q = (cv::Mat_<float>(3, 3) <<   1-2*pow(uX, 2), -2*uX*uY, -2*uX*uZ,
                                            -2*uX*uY, 1-2*pow(uY, 2), -2*uY*uZ,
                                            -2*uX*uZ, -2*uY*uZ, 1-2*pow(uZ, 2));

    cv::Mat srcMatrix, dstMatrix;
    size_t t = srcWorldPoints.size();
    for(int i = 0; i < t; i++)
    {
        srcMatrix = cv::Mat(srcWorldPoints[i]);
        dstMatrix = q * srcMatrix;
        if(srcWorldPoints == newWorldPoints){
            newWorldPoints[i] = cv::Point3f(dstMatrix);
        }
        else{
            newWorldPoints.emplace_back(dstMatrix);
        }
    }
}




void ImageDraw::setParamImagePerspective3d(cv::Mat cameraMatrix, cv::Mat distCoeffs, cv::Point3f imgOriPoint, cv::Size imgSizeIn3d, 
                                                    cv::Mat offsetRvec, cv::Mat offsetTvec)
{
    setCameraMatrix = cameraMatrix;
    setDisCoffes = distCoeffs;                          
    setImgOriPoint = imgOriPoint;
    setImgSizeIn3d = imgSizeIn3d;
    setOffsetRvec = offsetRvec;
    setOffsetTvec = offsetTvec;
}
void ImageDraw::pasteImagePerspective3d(cv::Mat &srcImageToPaste, cv::Mat &dstImagePasteOn, bool remove_background_color, bool center_image_axis, cv::Mat rvec, cv::Mat tvec)
{
    cv::Mat cameraMatrix = setCameraMatrix;
    cv::Mat distCoeffs = setDisCoffes;                            
    cv::Point3f imgOriPoint = setImgOriPoint;
    cv::Size imgSizeIn3d = setImgSizeIn3d;
    cv::Mat offsetRvec = setOffsetRvec;
    cv::Mat offsetTvec = setOffsetTvec;

    if(center_image_axis)
    {
        imgOriPoint.x = imgOriPoint.x - imgSizeIn3d.width / 2;
        imgOriPoint.y = imgOriPoint.y - imgSizeIn3d.height / 2;
    }
    
    vector<cv::Point3f> srcImagePoints3D;
    writeIn(srcImagePoints3D, imgOriPoint.x, imgOriPoint.y, imgOriPoint.z);
    writeIn(srcImagePoints3D, imgOriPoint.x + imgSizeIn3d.width, imgOriPoint.y, imgOriPoint.z);
    writeIn(srcImagePoints3D, imgOriPoint.x, imgOriPoint.y + imgSizeIn3d.height, imgOriPoint.z);
    writeIn(srcImagePoints3D, imgOriPoint.x + imgSizeIn3d.width, imgOriPoint.y + imgSizeIn3d.height, imgOriPoint.z);
    
    cv::Mat srcImage;
    cv::resize(srcImageToPaste, srcImage, cv::Size(srcImageToPaste.size().width, srcImageToPaste.size().width * (imgSizeIn3d.height / imgSizeIn3d.width)));
    // printf("test------------------------------------\n");
    if(!(offsetRvec.empty() && offsetTvec.empty()))
    {
        transform3dPoints(srcImagePoints3D, srcImagePoints3D, offsetRvec, offsetTvec);
    }

    vector<cv::Point2f> dstImagePoints2D;
    cv::projectPoints(srcImagePoints3D, rvec, tvec, cameraMatrix, distCoeffs, dstImagePoints2D);

    vector<cv::Point2f> srcImagePoints2D = {cv::Point2f(0, 0), cv::Point2f(0, srcImage.cols),
                                            cv::Point2f(srcImage.rows, 0), cv::Point2f(srcImage.rows, srcImage.cols)};

    cv::Mat warpM = cv::getPerspectiveTransform(srcImagePoints2D, dstImagePoints2D);
    cv::Mat _dstImage;
    cv::warpPerspective(srcImage, _dstImage, warpM, dstImagePasteOn.size());
    
    if(remove_background_color)
    {
        // cv::fillConvexPoly(dstImage, dstImagePoints2D, cv::Scalar(0, 0, 0));
        cv::Point mask[] = {    dstImagePoints2D[0],
                                dstImagePoints2D[1],
                                dstImagePoints2D[3],
                                dstImagePoints2D[2]};
        // std::cout << dstImage.empty() << std::endl;
        cv::fillConvexPoly(dstImagePasteOn, mask, 4, cv::Scalar(0, 0, 0));
    }
    
    dstImagePasteOn = dstImagePasteOn + _dstImage;
}
void ImageDraw::pasteImagePerspective3d(cv::Mat &srcImageToPaste, cv::Mat &dstImagePasteOn, bool remove_background_color, bool center_image_axis, vector<cv::Mat> rvecs, vector<cv::Mat> tvecs)
{
    if(rvecs.size() != tvecs.size())
    {
        printf("Sizes of rvecs and tvecs are not equal!\n");
        return;
    }
    // if(rvecs.empty() || tvecs.empty())
    // {
    //     printf("No rvecs or tvecs data. Perspective Failed!"\n);
    //     return;
    // }
    cv::Mat cameraMatrix = setCameraMatrix;
    cv::Mat distCoeffs = setDisCoffes;                            
    cv::Point3f imgOriPoint = setImgOriPoint;
    cv::Size imgSizeIn3d = setImgSizeIn3d;
    cv::Mat offsetRvec = setOffsetRvec;
    cv::Mat offsetTvec = setOffsetTvec;

    if(center_image_axis)
    {
        imgOriPoint.x = imgOriPoint.x - imgSizeIn3d.width / 2;
        imgOriPoint.y = imgOriPoint.y - imgSizeIn3d.height / 2;
    }
    
    vector<cv::Point3f> srcImagePoints3D;
    writeIn(srcImagePoints3D, imgOriPoint.x, imgOriPoint.y, imgOriPoint.z);
    writeIn(srcImagePoints3D, imgOriPoint.x + imgSizeIn3d.width, imgOriPoint.y, imgOriPoint.z);
    writeIn(srcImagePoints3D, imgOriPoint.x, imgOriPoint.y + imgSizeIn3d.height, imgOriPoint.z);
    writeIn(srcImagePoints3D, imgOriPoint.x + imgSizeIn3d.width, imgOriPoint.y + imgSizeIn3d.height, imgOriPoint.z);
    

    // std::cout << srcImageToPaste.size() << std::endl;
    cv::Mat srcImage;
    cv::resize(srcImageToPaste, srcImage, cv::Size(srcImageToPaste.size().width, srcImageToPaste.size().width * (imgSizeIn3d.height / imgSizeIn3d.width)));
    // printf("test------------------------------------\n");
    if(!(offsetRvec.empty() && offsetTvec.empty()))
    {
        transform3dPoints(srcImagePoints3D, srcImagePoints3D, offsetRvec, offsetTvec);
    }

    vector<cv::Point2f> dstImagePoints2D;
    vector<cv::Point2f> srcImagePoints2D = {cv::Point2f(0, 0), cv::Point2f(0, srcImage.cols),
                                            cv::Point2f(srcImage.rows, 0), cv::Point2f(srcImage.rows, srcImage.cols)};
    cv::Mat warpM, _dstImage;
    for(int i = 0; i < rvecs.size(); i++)
    {
        cv::projectPoints(srcImagePoints3D, rvecs[i], tvecs[i], cameraMatrix, distCoeffs, dstImagePoints2D);

        warpM = cv::getPerspectiveTransform(srcImagePoints2D, dstImagePoints2D);
        cv::Mat _dstImage;
        cv::warpPerspective(srcImage, _dstImage, warpM, dstImagePasteOn.size());
        
        if(remove_background_color)
        {
            // cv::fillConvexPoly(dstImage, dstImagePoints2D, cv::Scalar(0, 0, 0));
            cv::Point mask[] = {    dstImagePoints2D[0],
                                    dstImagePoints2D[1],
                                    dstImagePoints2D[3],
                                    dstImagePoints2D[2]};
            // std::cout << dstImage.empty() << std::endl;
            cv::fillConvexPoly(dstImagePasteOn, mask, 4, cv::Scalar(0, 0, 0));
        }
        
        dstImagePasteOn = dstImagePasteOn + _dstImage;
    }
    
}
void ImageDraw::pasteImagePerspective3d(cv::Mat &srcImage, cv::Mat &dstImage, bool remove_background_color, bool center_image_axis, 
                                    cv::Mat cameraMatrix, cv::Mat distCoeffs, cv::Mat rvec, cv::Mat tvec,
                                    cv::Point3f imgOriPoint, cv::Size imgSizeIn3d, cv::Mat offsetRvec, cv::Mat offsetTvec)
{
    if(center_image_axis)
    {
        imgOriPoint.x = imgOriPoint.x - imgSizeIn3d.width / 2;
        imgOriPoint.y = imgOriPoint.y - imgSizeIn3d.height / 2;
    }

    vector<cv::Point3f> srcImagePoints3D;
    writeIn(srcImagePoints3D, imgOriPoint.x, imgOriPoint.y, imgOriPoint.z);
    writeIn(srcImagePoints3D, imgOriPoint.x + imgSizeIn3d.width, imgOriPoint.y, imgOriPoint.z);
    writeIn(srcImagePoints3D, imgOriPoint.x, imgOriPoint.y + imgSizeIn3d.height, imgOriPoint.z);
    writeIn(srcImagePoints3D, imgOriPoint.x + imgSizeIn3d.width, imgOriPoint.y + imgSizeIn3d.height, imgOriPoint.z);

    cv::resize(srcImage, srcImage, cv::Size(srcImage.size().width, srcImage.size().width * (imgSizeIn3d.height / imgSizeIn3d.width)));
    
    if(!(offsetRvec.empty() && offsetTvec.empty()))
    {
        transform3dPoints(srcImagePoints3D, srcImagePoints3D, offsetRvec, offsetTvec);
    }

    vector<cv::Point2f> dstImagePoints2D;
    cv::projectPoints(srcImagePoints3D, rvec, tvec, cameraMatrix, distCoeffs, dstImagePoints2D);

    vector<cv::Point2f> srcImagePoints2D = {cv::Point2f(0, 0), cv::Point2f(0, srcImage.cols),
                                            cv::Point2f(srcImage.rows, 0), cv::Point2f(srcImage.rows, srcImage.cols)};

    cv::Mat warpM = cv::getPerspectiveTransform(srcImagePoints2D, dstImagePoints2D);
    cv::Mat _dstImage;
    cv::warpPerspective(srcImage, _dstImage, warpM, dstImage.size());
    
    if(remove_background_color)
    {
        // cv::fillConvexPoly(dstImage, dstImagePoints2D, cv::Scalar(0, 0, 0));
        cv::Point mask[] = {    dstImagePoints2D[0],
                                dstImagePoints2D[1],
                                dstImagePoints2D[3],
                                dstImagePoints2D[2]};
        // std::cout << dstImage.empty() << std::endl;
        cv::fillConvexPoly(dstImage, mask, 4, cv::Scalar(0, 0, 0));
    }
    
    dstImage = dstImage + _dstImage;
    
}   

void ImageDraw::centerImageScale(cv::Mat &srcImage, cv::Mat &dstImage)
{
    float srcH = srcImage.size().height, srcW = srcImage.size().width;
    float cX = srcW / 2;
    float cY = srcH / 2;
    float scaleX = scaleX;
    float scaleY = scaleY;

    cv::Point2f srcP[] = {  cv::Point2f(srcW, 0), 
                            cv::Point2f(srcW ,srcH), 
                            cv::Point2f(0 ,srcH)
                         };

    cv::Point2f dstP[] = {  cv::Point2f((srcW - cX) * scaleX + cX, (0 - cY) * scaleY + cY), 
                            cv::Point2f((srcW - cX) * scaleX + cX, (srcH - cY) * scaleY + cY), 
                            cv::Point2f((0 - cX) * scaleX + cX, (srcH - cY) * scaleY + cY)
                         };

    cv::Mat warpM = cv::getAffineTransform(srcP, dstP);
    cv::warpAffine(srcImage, dstImage, warpM, srcImage.size());
}
void ImageDraw::centerImageScale(cv::Mat &srcImage, cv::Mat &dstImage, float scaleX, float scaleY, int flags, int borderMode, const cv::Scalar &borderValue)
{
    float srcH = srcImage.size().height, srcW = srcImage.size().width;
    float cX = srcW / 2;
    float cY = srcH / 2;

    cv::Point2f srcP[] = {  cv::Point2f(srcW, 0), 
                            cv::Point2f(srcW ,srcH), 
                            cv::Point2f(0 ,srcH)
                         };

    cv::Point2f dstP[] = {  cv::Point2f((srcW - cX) * scaleX + cX, (0 - cY) * scaleY + cY), 
                            cv::Point2f((srcW - cX) * scaleX + cX, (srcH - cY) * scaleY + cY), 
                            cv::Point2f((0 - cX) * scaleX + cX, (srcH - cY) * scaleY + cY)
                         };

    cv::Mat warpM = cv::getAffineTransform(srcP, dstP);
    cv::warpAffine(srcImage, dstImage, warpM, srcImage.size(), flags, borderMode, borderValue);
}

// cv::Mat ImageDraw::cal_2vec_rvec(cv::Point3f vecOri, cv::Point3f vecDst)
// {
//     cv::Mat _vecOri = (cv::Mat_<float>(3, 1) << vecOri.x, vecOri.y, vecOri.z), _vecOri_;
//     cv::Mat _vecDst = (cv::Mat_<float>(3, 1) << vecDst.x, vecDst.y, vecDst.z), _vecDst_;
//     cv::Mat v, vs, vt, ca;

//     cv::normalize(_vecOri, _vecOri_, 0.0, 1.0);
//     cv::normalize(_vecDst, _vecDst_, 0.0, 1.0);

//     vs = _vecDst_ * _vecOri_;
//     cv::normalize(vs, v, 0.0, 1.0);

//     ca = _vecDst_.dot(_vecOri_);
//     vt = v * (1 - ca);

    
//     // rotationMatrix[0, 0] = vt.x * v.x + ca;
//     // rotationMatrix[1, 1] = vt.y * v.y + ca;
//     // rotationMatrix[2, 2] = vt.z * v.z + ca;
//     // vt.x *= v.y;
//     // vt.z *= v.x;
//     // vt.y *= v.z;

//     // rotationMatrix[0, 1] = vt.x - vs.z;
//     // rotationMatrix[0, 2] = vt.z + vs.y;
//     // rotationMatrix[1, 0] = vt.x + vs.z;
//     // rotationMatrix[1, 2] = vt.y - vs.x;
//     // rotationMatrix[2, 0] = vt.z - vs.y;
//     // rotationMatrix[2, 1] = vt.y + vs.x;
//     cv::Mat R = (cv::Mat_<float>(3, 3) <<   vt.at<float>(0) * v.at<float>(0) + ca, vt.at<float>(0) - vs.at<float>(2),
//                                             vt.at<float>(0) + vs.at<float>(2), vt.at<float>(1) * v.at<float>(1) + ca, vt.at<float>(1) - vs.at<float>(0),
//                                             vt.at<float>(2) - vs.at<float>(1), vt.at<float>(1) + vs.at<float>(0), vt.at<float>(2) * v.at<float>(2) + ca);

//     cv::Mat rvec;
//     cv::Rodrigues(R ,rvec);
//     return rvec;
// }


void ImageDraw::drawLine2d(cv::Mat &imgInputOutput, float x1, float y1, float z1, float x2, float y2, float z2, 
                                    cv::Mat rvec, cv::Mat tvec, cv::Scalar color, cv::Mat cameraMatrix, cv::Mat distCoeffs)
{   
    if(cameraMatrix.empty() || distCoeffs.empty())
    {
        if(setCameraMatrix.empty() || setDisCoffes.empty())
        {
            return;
        }
        cameraMatrix = setCameraMatrix;
        distCoeffs = setDisCoffes;
    }
    if(rvec.empty() || tvec.empty())
    {
        return;
    }
    vector<cv::Point3f> p3d;
    writeIn(p3d, x1, y1, z1);
    writeIn(p3d, x2, y2, z2);



    vector<cv::Point2f> p2d;
    cv::projectPoints(p3d, rvec, tvec, cameraMatrix, distCoeffs, p2d);
    
    cv::line(imgInputOutput, p2d[0], p2d[1], color, 3);
}
void ImageDraw::drawLine2d(cv::Mat &imgInputOutput, cv::Point3f point1, cv::Point3f point2,
                            cv::Mat rvec, cv::Mat tvec, cv::Scalar color, cv::Mat cameraMatrix, cv::Mat distCoeffs)
{
    if(cameraMatrix.empty() || distCoeffs.empty())
    {
        if(setCameraMatrix.empty() || setDisCoffes.empty())
        {
            return;
        }
        cameraMatrix = setCameraMatrix;
        distCoeffs = setDisCoffes;
    }
    if(rvec.empty() || tvec.empty())
    {
        return;
    }
    vector<cv::Point3f> p3d;
    writeIn(p3d, point1);
    writeIn(p3d, point2);

    vector<cv::Point2f> p2d;
    cv::projectPoints(p3d, rvec, tvec, cameraMatrix, distCoeffs, p2d);
    
    cv::line(imgInputOutput, p2d[0], p2d[1], color, 3);
}


cv::Scalar ImageDraw::intensityToRainbowColor(float intensity, float min_intensity, float max_intensity) 
{
    const float span = max_intensity - min_intensity;
    const float value = (span != 0) ? (intensity - min_intensity) / span : 1.0;

    float h = (1.0 - value) * 280.0;  
    float s = 1.0;
    float v = 1.0;

    int i = int(h / 60.0) % 6;
    float f = h / 60.0 - i;
    float p = v * (1 - s);
    float q = v * (1 - s * f);
    float t = v * (1 - s * (1 - f));
    float r, g, b;
    switch (i) {
        case 0: r = v, g = t, b = p; break;
        case 1: r = q, g = v, b = p; break;
        case 2: r = p, g = v, b = t; break;
        case 3: r = p, g = q, b = v; break;
        case 4: r = t, g = p, b = v; break;
        case 5: r = v, g = p, b = q; break;
    }
    return cv::Scalar(b * 255, g * 255, r * 255);  
}
cv::Scalar ImageDraw::xToRainbowColor(float x, float min_x, float max_x) 
{
    const float span = max_x - min_x;
    const float value = (span != 0) ? (x - min_x) / span : 1.0;

    float h = (1.0 - value) * 280.0;  
    float s = 1.0;
    float v = 1.0;

    int i = int(h / 60.0) % 6;
    float f = h / 60.0 - i;
    float p = v * (1 - s);
    float q = v * (1 - s * f);
    float t = v * (1 - s * (1 - f));
    float r, g, b;
    switch (i) {
        case 0: r = v, g = t, b = p; break;
        case 1: r = q, g = v, b = p; break;
        case 2: r = p, g = v, b = t; break;
        case 3: r = p, g = q, b = v; break;
        case 4: r = t, g = p, b = v; break;
        case 5: r = v, g = p, b = q; break;
    }
    return cv::Scalar(b * 255, g * 255, r * 255);  
}
cv::Scalar ImageDraw::yToRainbowColor(float y, float min_y, float max_y) 
{
    const float span = max_y - min_y;
    const float value = (span != 0) ? (y - min_y) / span : 1.0;

    float h = (1.0 - value) * 280.0;  
    float s = 1.0;
    float v = 1.0;

    int i = int(h / 60.0) % 6;
    float f = h / 60.0 - i;
    float p = v * (1 - s);
    float q = v * (1 - s * f);
    float t = v * (1 - s * (1 - f));
    float r, g, b;
    switch (i) {
        case 0: r = v, g = t, b = p; break;
        case 1: r = q, g = v, b = p; break;
        case 2: r = p, g = v, b = t; break;
        case 3: r = p, g = q, b = v; break;
        case 4: r = t, g = p, b = v; break;
        case 5: r = v, g = p, b = q; break;
    }
    return cv::Scalar(b * 255, g * 255, r * 255);  
}
cv::Scalar ImageDraw::zToRainbowColor(float z, float min_z, float max_z) 
{
    const float span = max_z - min_z;
    const float value = (span != 0) ? (z - min_z) / span : 1.0;

    float h = (1.0 - value) * 280.0;  
    float s = 1.0;
    float v = 1.0;

    int i = int(h / 60.0) % 6;
    float f = h / 60.0 - i;
    float p = v * (1 - s);
    float q = v * (1 - s * f);
    float t = v * (1 - s * (1 - f));
    float r, g, b;
    switch (i) {
        case 0: r = v, g = t, b = p; break;
        case 1: r = q, g = v, b = p; break;
        case 2: r = p, g = v, b = t; break;
        case 3: r = p, g = q, b = v; break;
        case 4: r = t, g = p, b = v; break;
        case 5: r = v, g = p, b = q; break;
    }
    return cv::Scalar(b * 255, g * 255, r * 255);  
}


void ImageDraw::projectPointsToImage(const pcl::PointCloud<pcl::PointXYZI>& cloud, std::vector<cv::Point2f>& imagePoints) 
{
    std::vector<cv::Point3f> cvPoints;
    for (const auto& p : cloud.points) 
    {
        cvPoints.push_back(cv::Point3f(p.x, p.y, p.z));
    }

    cv::projectPoints(cvPoints, cv::Vec3d(0,0,0), cv::Vec3d(0,0,0), setCameraMatrix, setDisCoffes, imagePoints);
}

void ImageDraw::drawPointsOnImageIntensity(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                                const std::vector<cv::Point2f>& points,
                                cv::Mat& image) 
{
    float min_intensity = std::numeric_limits<float>::max();
    float max_intensity = -std::numeric_limits<float>::max();

    for (const auto& point : cloud.points) 
    {
        min_intensity = std::min(min_intensity, point.intensity);
        max_intensity = std::max(max_intensity, point.intensity);
    }

    for (size_t i = 0; i < points.size(); i++) 
    {
        cv::Scalar color = ImageDraw::intensityToRainbowColor(cloud.points[i].intensity, min_intensity, max_intensity);
        cv::circle(image, points[i], 3, color, -1); 
    }
}
void ImageDraw::drawPointsOnImageX(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                                const std::vector<cv::Point2f>& points,
                                cv::Mat& image) 
{
    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();

    for (const auto& point : cloud.points) 
    {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
    }

    for (size_t i = 0; i < points.size(); i++) 
    {
        cv::Scalar color = ImageDraw::intensityToRainbowColor(cloud.points[i].x, min_x, max_x);
        cv::circle(image, points[i], 3, color, -1); 
    }
}
void ImageDraw::drawPointsOnImageY(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                                const std::vector<cv::Point2f>& points,
                                cv::Mat& image) 
{
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();

    for (const auto& point : cloud.points) 
    {
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }

    for (size_t i = 0; i < points.size(); i++) 
    {
        cv::Scalar color = ImageDraw::intensityToRainbowColor(cloud.points[i].y, min_y, max_y);
        cv::circle(image, points[i], 3, color, -1); 
    }
}
void ImageDraw::drawPointsOnImageZ(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                                const std::vector<cv::Point2f>& points,
                                cv::Mat& image) 
{
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();

    for (const auto& point : cloud.points) 
    {
        min_z = std::min(min_z, point.z);
        max_z = std::max(max_z, point.z);
    }

    for (size_t i = 0; i < points.size(); i++) 
    {
        cv::Scalar color = ImageDraw::intensityToRainbowColor(cloud.points[i].z, min_z, max_z);
        cv::circle(image, points[i], 3, color, -1); 
    }
}
