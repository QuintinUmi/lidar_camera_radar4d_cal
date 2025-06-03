#ifndef _TIMESTAMP_COV_H_
#define _TIMESTAMP_COV_H_

#include <ros/ros.h>
#include <cstdint>

namespace lcr_cal {
    
    uint64_t rosTimeToTimestamp(const ros::Time& time) {
        return time.toNSec();  // 返回纳秒数
    }

    ros::Time timestampToRosTime(uint64_t timestamp) {
        return ros::Time().fromNSec(timestamp);  // 从纳秒数创建ros::Time对象
    }
}

#endif