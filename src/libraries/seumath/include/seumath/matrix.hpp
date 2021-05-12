#ifndef __MATRIX_HPP
#define __MATRIX_HPP

#include <cmath>
#include <eigen3/Eigen/Dense>
#include "number.hpp"
#include "angle.hpp"

namespace seumath
{
    class TransformMatrix: public Eigen::Matrix4d
    {
    public:
        TransformMatrix()
        {
            *this = Eigen::Matrix4d::Identity(4, 4);
        }

        TransformMatrix(const TransformMatrix &m)
        {
            this->block<4, 4>(0, 0) = m;
        }

        TransformMatrix(const Eigen::Matrix4d &m)
        {
            this->block<4, 4>(0, 0) = m;
        }

        TransformMatrix(const double &x, const double &y, const double &z)
        {
            *this = Eigen::Matrix4d::Identity(4, 4);
            setP(Eigen::Vector3d(x, y, z));
        }

        TransformMatrix(const double &deg, const char &c = 'x')
        {
            *this = Eigen::Matrix4d::Identity(4, 4);
            Eigen::Vector3d temp = Eigen::Vector3d::UnitX();

            switch (c)
            {
                case 'x':
                case 'X':
                    temp = Eigen::Vector3d::UnitX();
                    break;

                case 'y':
                case 'Y':
                    temp = Eigen::Vector3d::UnitY();
                    break;

                case 'z':
                case 'Z':
                    temp = Eigen::Vector3d::UnitZ();
                    break;

                default:
                    break;
            }

            Eigen::AngleAxisd rotv(deg2rad(deg), temp);
            setR(rotv.toRotationMatrix());
        }

        TransformMatrix rotationX(const double &deg)
        {
            *this = (*this) * TransformMatrix(deg, 'x');
            return *this;
        }

        TransformMatrix rotationY(const double &deg)
        {
            *this = (*this) * TransformMatrix(deg, 'y');
            return *this;
        }

        TransformMatrix rotationZ(const double &deg)
        {
            *this = (*this) * TransformMatrix(deg, 'z');
            return *this;
        }

        TransformMatrix translation(const double &x, const double &y, const double &z)
        {
            *this = (*this) * TransformMatrix(x, y, z);
            return *this;
        }

        Eigen::Matrix3d R() const
        {
            return this->block<3, 3>(0, 0);
        }

        Eigen::Vector3d P() const
        {
            return this->block<3, 1>(0, 3);
        }

        Eigen::Vector3d n() const
        {
            return this->block<3, 1>(0, 0);
        }

        Eigen::Vector3d o() const
        {
            return this->block<3, 1>(0, 1);
        }

        Eigen::Vector3d a() const
        {
            return this->block<3, 1>(0, 2);
        }

        double xRotate() const
        {
            Eigen::Matrix3d rot = R();
            return atan2(rot(2, 1), rot(2, 2));
        }

        double yRotate() const
        {
            Eigen::Matrix3d rot = R();
            return atan2(-rot(2, 0), std::sqrt(std::pow(rot(2, 1), 2) + std::pow(rot(2, 2), 2)));
        }

        double zRotate() const
        {
            Eigen::Matrix3d rot = R();
            return atan2(rot(1, 0), rot(0, 0));
        }

        void setP(const Eigen::Vector3d &p)
        {
            this->block<3, 1>(0, 3) = p;
        }

        void setR(const Eigen::Matrix3d &r)
        {
            this->block<3, 3>(0, 0) = r;
        }

        TransformMatrix &operator=(const Eigen::Matrix4d &m)
        {
            this->block<4, 4>(0, 0) = m;
            return *this;
        }
    };
}

#endif
