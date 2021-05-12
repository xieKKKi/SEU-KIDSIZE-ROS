#ifndef __ANGLE_HPP
#define __ANGLE_HPP

#include <eigen3/Eigen/Dense>
#include <cmath>

namespace seumath
{
    inline double deg2rad(const double &x)
    {
        return x * M_PI / 180.0;
    }

    inline double rad2deg(const double &x)
    {
        return x * 180.0 / M_PI;
    }

    inline double cosDeg(const double &d)
    {
        return cos(deg2rad(d));
    }

    inline double sinDeg(const double &d)
    {
        return sin(deg2rad(d));
    }

    inline double tanDeg(const double &d)
    {
        return tan(deg2rad(d));
    }

    template<typename T>
    inline double azimuthDeg(const T &v)
    {
        return rad2deg(std::atan2(v.y(), v.x()));
    }

    template<typename T>
    inline double azimuthRad(const T &v)
    {
        return std::atan2(v.y(), v.x());
    }

    template<typename T>
    inline T normalizeDeg(T deg)
    {
        while (deg > 180.0)
        {
            deg -= 360.0;
        }

        while (deg < -180.0)
        {
            deg += 360.0;
        }

        return deg;
    }

    template<typename T>
    inline T normalizeRad(T rad)
    {
        while (rad > M_PI)
        {
            rad -= 2 * M_PI;
        }

        while (rad < -M_PI)
        {
            rad += 2 * M_PI;
        }

        return rad;
    }
}

#endif
