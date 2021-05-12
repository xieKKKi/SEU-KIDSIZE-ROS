#ifndef __NUMBER_HPP
#define __NUMBER_HPP

#include <eigen3/Eigen/Dense>
#include <cmath>

namespace seumath
{
    inline bool isZero(const double &x)
    {
        return fabs(x) < 1E-4;
    }

    template <typename T>
    inline T sign(const T &x)
    {
        return (x >= 0) ? 1 : -1;
    }

    template <typename T>
    inline T bound(const T min, const T max, T &x)
    {
        if (max < min)
        {
            return x;
        }

        if (x < min)
        {
            x = min;
        }
        else if (x > max)
        {
            x = max;
        }

        return x;
    }

    inline Eigen::Matrix3d xRotationMat(const double &rad)
    {
        Eigen::AngleAxisd temp(rad, Eigen::Vector3d::UnitX());
        return temp.toRotationMatrix();
    }

    inline Eigen::Matrix3d yRotationMat(const double &rad)
    {
        Eigen::AngleAxisd temp(rad, Eigen::Vector3d::UnitY());
        return temp.toRotationMatrix();
    }

    inline Eigen::Matrix3d zRotationMat(const double &rad)
    {
        Eigen::AngleAxisd temp(rad, Eigen::Vector3d::UnitZ());
        return temp.toRotationMatrix();
    }
}

#endif
