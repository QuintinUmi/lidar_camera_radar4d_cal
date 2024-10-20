#ifndef _DYNAMIC_RECONFIGURE_H_
#define _DYNAMIC_RECONFIGURE_H_


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <dynamic_reconfigure/server.h>
#include <lidar_camera_cal/TransformFilterConfig.h>


namespace lidar_camera_cal
{
    struct RQTConfig
    {

        struct _TransformFilterConfig_
        {
            float center_x;
            float center_y;
            float center_z;
            float length_x;
            float length_y;
            float length_z;
            float rotate_x;
            float rotate_y;
            float rotate_z;
        }TransformFilterConfig;

        
    };

    class PointcloudFilterReconfigure 
    {
        public:

            PointcloudFilterReconfigure();
            PointcloudFilterReconfigure(ros::NodeHandle nh);

            RQTConfig::_TransformFilterConfig_ getTransformConfigure();
            bool isUpdated();

        private:

            void TransformFilterReconfigureCallBack(lidar_camera_cal::TransformFilterConfig &pcTransformFilterConfig, uint32_t level);

        private:

            RQTConfig::_TransformFilterConfig_ TransformFilterConfig;
            dynamic_reconfigure::Server<lidar_camera_cal::TransformFilterConfig> transform_filter_server;
            dynamic_reconfigure::Server<lidar_camera_cal::TransformFilterConfig>::CallbackType transform_filter_f;

            bool is_updated_ = false;
            
    };

    
}



using namespace lidar_camera_cal;


PointcloudFilterReconfigure::PointcloudFilterReconfigure() :    transform_filter_server(ros::NodeHandle("TransformFilterReconfigure"))
{

    this->transform_filter_f = boost::bind(&PointcloudFilterReconfigure::TransformFilterReconfigureCallBack, this, _1, _2);
    this->transform_filter_server.setCallback(this->transform_filter_f);
    
}

PointcloudFilterReconfigure::PointcloudFilterReconfigure(ros::NodeHandle nh) :  transform_filter_server(ros::NodeHandle(nh.getNamespace() + "-TransformFilterReconfigure"))
{

    this->transform_filter_f = boost::bind(&PointcloudFilterReconfigure::TransformFilterReconfigureCallBack, this, _1, _2);
    this->transform_filter_server.setCallback(this->transform_filter_f);
    
}


void PointcloudFilterReconfigure::TransformFilterReconfigureCallBack(lidar_camera_cal::TransformFilterConfig &pcTransformFilterConfig, uint32_t level) 
{
    ROS_INFO("Transform Filter Reconfigure: center_x=%f, center_y=%f, center_z=%f, length_x=%f, length_y=%f, length_z=%f, rotate_x=%f, rotate_y=%f, rotate_z=%f",
            pcTransformFilterConfig.center_x, pcTransformFilterConfig.center_y, pcTransformFilterConfig.center_z, 
            pcTransformFilterConfig.length_x, pcTransformFilterConfig.length_y, pcTransformFilterConfig.length_z,
            pcTransformFilterConfig.rotate_x, pcTransformFilterConfig.rotate_y, pcTransformFilterConfig.rotate_z);
    
    this->TransformFilterConfig.center_x = pcTransformFilterConfig.center_x;
    this->TransformFilterConfig.center_y = pcTransformFilterConfig.center_y;
    this->TransformFilterConfig.center_z = pcTransformFilterConfig.center_z;
    this->TransformFilterConfig.length_x = pcTransformFilterConfig.length_x;
    this->TransformFilterConfig.length_y = pcTransformFilterConfig.length_y;
    this->TransformFilterConfig.length_z = pcTransformFilterConfig.length_z;
    this->TransformFilterConfig.rotate_x = pcTransformFilterConfig.rotate_x;
    this->TransformFilterConfig.rotate_y = pcTransformFilterConfig.rotate_y;
    this->TransformFilterConfig.rotate_z = pcTransformFilterConfig.rotate_z;

    this->is_updated_ = true;
}


RQTConfig::_TransformFilterConfig_ PointcloudFilterReconfigure::getTransformConfigure()
{
    return this->TransformFilterConfig;
}

bool PointcloudFilterReconfigure::isUpdated()
{
    if(this->is_updated_)
    {
        this->is_updated_ = false;
        return true;
    }
    return false;
}





#endif