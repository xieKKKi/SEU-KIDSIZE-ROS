#ifndef __SEU_COMMON_HPP
#define __SEU_COMMON_HPP

#include <stdint.h>
#include <ctime>
#include <cstdio>
#include <string>
#include <iomanip>
#include <unistd.h>
#include <ros/ros.h>

#define SEU_MAKEWORD(a, b)  ((uint16_t)(((uint8_t)(((uint64_t)(a)) & 0xff)) | ((uint16_t)((uint8_t)(((uint64_t)(b)) & 0xff))) << 8))
#define SEU_MAKEDWORD(a, b) ((uint32_t)(((uint16_t)(((uint64_t)(a)) & 0xffff)) | ((uint32_t)((uint16_t)(((uint64_t)(b)) & 0xffff))) << 16))
#define SEU_LOWORD(l)       ((uint16_t)(((uint64_t)(l)) & 0xffff))
#define SEU_HIWORD(l)       ((uint16_t)((((uint64_t)(l)) >> 16) & 0xffff))
#define SEU_LOBYTE(w)       ((uint8_t)(((uint64_t)(w)) & 0xff))
#define SEU_HIBYTE(w)       ((uint8_t)((((uint64_t)(w)) >> 8) & 0xff))

inline std::string get_current_time()
{
    time_t timep;
    std::time(&timep);
    char tmp[64];
    std::strftime(tmp, sizeof(tmp), "%Y-%m-%d_%H%M%S", std::localtime(&timep));
    return std::string(tmp);
}

inline uint32_t get_clock()
{
    return ros::Time::now().toNSec()/1E+6;
}

#endif