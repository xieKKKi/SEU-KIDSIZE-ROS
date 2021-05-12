#ifndef __CLASS_EXCEPTION_HPP
#define __CLASS_EXCEPTION_HPP

#include <exception>
#include <iostream>
#include "logger.hpp"

template <class CLASS>
class ClassException
{
public:
    ClassException(std::string msg, int id = 0): msg_(msg), id_(id)
    {
        LOG(LOG_WARN) << "exception: " + msg_ << endll;
    }

    ~ClassException() {};

    inline char *what() const
    {
        return msg_.c_str();
    }

    inline int err_no() const
    {
        return id_;
    }
private:
    std::string msg_;
    int id_;
};

#endif
