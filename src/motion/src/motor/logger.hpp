#ifndef __LOGGER_HPP
#define __LOGGER_HPP

#include <iostream>

enum log_level
{
    LOG_DEBUG,
    LOG_INFO,
    LOG_WARN,
    LOG_ERROR,
    LOG_HIGH
};

// print without specified colour
inline std::ostream& endll(std::ostream &os)
{
    return os<<"\033[0m"<<std::endl;
}

inline std::ostream& logger(log_level l)
{
    if(l==LOG_HIGH)
        return std::cout<<"\033[43;37m";
    else if(l==LOG_INFO) 
        return std::cout<<"\033[32m[ INFO]: ";
    else if(l==LOG_WARN)
        return std::cout<<"\033[33m[ WARN]: ";
    else if(l==LOG_ERROR)
        return std::cout<<"\033[31m[ERROR]: ";
    else
        return std::cout<<"[DEBUG]: ";
}

#define LOG(l) logger(l)

#endif
