#ifndef _DYNAMIC_RECONFIGURE_H_
#define _DYNAMIC_RECONFIGURE_H_


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <dynamic_reconfigure/server.h>
#include <lcr_cal/TransformFilterConfig.h>
#include <lcr_cal/Radar4DFilterConfig.h>


namespace lcr_cal
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

        struct _Radar4DFilterConfig_
        {
            float filter_distance;
            float filter_snr;
            int max_snr_points_num;
            float valid_distance_threshold;
        }Radar4DFilterConfig;

        
    };

    class PointcloudFilterReconfigure 
    {
        public:

            PointcloudFilterReconfigure();
            PointcloudFilterReconfigure(ros::NodeHandle nh);

            RQTConfig::_TransformFilterConfig_ getTransformConfigure();
            bool isUpdated();

        private:

            void TransformFilterReconfigureCallBack(lcr_cal::TransformFilterConfig &_pcTransformFilterConfig, uint32_t level);

        private:

            RQTConfig::_TransformFilterConfig_ TransformFilterConfig;
            dynamic_reconfigure::Server<lcr_cal::TransformFilterConfig> transform_filter_server;
            dynamic_reconfigure::Server<lcr_cal::TransformFilterConfig>::CallbackType transform_filter_f;

            bool is_updated_ = false;
            
    };

    class Radar4DFilterReconfigure 
    {
        public:

            Radar4DFilterReconfigure();
            Radar4DFilterReconfigure(ros::NodeHandle nh);

            RQTConfig::_Radar4DFilterConfig_ getRadar4DFilterConfigure();
            bool isUpdated();

        private:

            void Radar4DFilterReconfigureCallBack(lcr_cal::Radar4DFilterConfig &radar4DFilterConfig, uint32_t level);

        private:

            RQTConfig::_Radar4DFilterConfig_ _radar4DFilterConfig;
            dynamic_reconfigure::Server<lcr_cal::Radar4DFilterConfig> radar4D_filter_server;
            dynamic_reconfigure::Server<lcr_cal::Radar4DFilterConfig>::CallbackType radar4D_filter_f;

            bool is_updated_ = false;
            
    };

    
}



using namespace lcr_cal;


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


void PointcloudFilterReconfigure::TransformFilterReconfigureCallBack(lcr_cal::TransformFilterConfig &_pcTransformFilterConfig, uint32_t level) 
{
    ROS_INFO("Transform Filter Reconfigure: center_x=%f, center_y=%f, center_z=%f, length_x=%f, length_y=%f, length_z=%f, rotate_x=%f, rotate_y=%f, rotate_z=%f",
            _pcTransformFilterConfig.center_x, _pcTransformFilterConfig.center_y, _pcTransformFilterConfig.center_z, 
            _pcTransformFilterConfig.length_x, _pcTransformFilterConfig.length_y, _pcTransformFilterConfig.length_z,
            _pcTransformFilterConfig.rotate_x, _pcTransformFilterConfig.rotate_y, _pcTransformFilterConfig.rotate_z);
    
    this->TransformFilterConfig.center_x = _pcTransformFilterConfig.center_x;
    this->TransformFilterConfig.center_y = _pcTransformFilterConfig.center_y;
    this->TransformFilterConfig.center_z = _pcTransformFilterConfig.center_z;
    this->TransformFilterConfig.length_x = _pcTransformFilterConfig.length_x;
    this->TransformFilterConfig.length_y = _pcTransformFilterConfig.length_y;
    this->TransformFilterConfig.length_z = _pcTransformFilterConfig.length_z;
    this->TransformFilterConfig.rotate_x = _pcTransformFilterConfig.rotate_x;
    this->TransformFilterConfig.rotate_y = _pcTransformFilterConfig.rotate_y;
    this->TransformFilterConfig.rotate_z = _pcTransformFilterConfig.rotate_z;

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





Radar4DFilterReconfigure::Radar4DFilterReconfigure() :    radar4D_filter_server(ros::NodeHandle("Radar4DFilterReconfigure"))
{

    this->radar4D_filter_f = boost::bind(&Radar4DFilterReconfigure::Radar4DFilterReconfigureCallBack, this, _1, _2);
    this->radar4D_filter_server.setCallback(this->radar4D_filter_f);
    
}

Radar4DFilterReconfigure::Radar4DFilterReconfigure(ros::NodeHandle nh) :  radar4D_filter_server(ros::NodeHandle(nh.getNamespace() + "-Radar4DFilterReconfigure"))
{

    this->radar4D_filter_f = boost::bind(&Radar4DFilterReconfigure::Radar4DFilterReconfigureCallBack, this, _1, _2);
    this->radar4D_filter_server.setCallback(this->radar4D_filter_f);
    
}


void Radar4DFilterReconfigure::Radar4DFilterReconfigureCallBack(lcr_cal::Radar4DFilterConfig &radar4DFilterConfig, uint32_t level) 
{
    ROS_INFO("4D Radar Filter Reconfigure: filter_distance=%f, filter_snr=%f, max_snr_points_num=%d, valid_distance_threshold=%f",
        radar4DFilterConfig.filter_distance, radar4DFilterConfig.filter_snr, 
        radar4DFilterConfig.max_snr_points_num, radar4DFilterConfig.valid_distance_threshold);
    
    this->_radar4DFilterConfig.filter_distance = radar4DFilterConfig.filter_distance;
    this->_radar4DFilterConfig.filter_snr = radar4DFilterConfig.filter_snr;
    this->_radar4DFilterConfig.max_snr_points_num = radar4DFilterConfig.max_snr_points_num;
    this->_radar4DFilterConfig.valid_distance_threshold = radar4DFilterConfig.valid_distance_threshold;

    this->is_updated_ = true;
}


RQTConfig::_Radar4DFilterConfig_ Radar4DFilterReconfigure::getRadar4DFilterConfigure()
{
    return this->_radar4DFilterConfig;
}

bool Radar4DFilterReconfigure::isUpdated()
{
    if(this->is_updated_)
    {
        this->is_updated_ = false;
        return true;
    }
    return false;
}




#endif