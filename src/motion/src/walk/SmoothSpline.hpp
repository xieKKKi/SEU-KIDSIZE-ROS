#ifndef LEPH_SMOOTHSPLINE_HPP
#define LEPH_SMOOTHSPLINE_HPP

#include "Spline.hpp"

namespace Leph
{

    /**
     * SmoothSpline
     *
     * Implementation of 5th order polynomial
     * splines trajectory known to minimize jerk
     */
    class SmoothSpline : public Spline
    {
    public:

        /**
         * Simple point struture
         */
        struct Point
        {
            double time;
            double position;
            double velocity;
            double acceleration;
        };

        /**
         * Add a new point with its time, position value,
         * velocity and acceleration
         */
        void addPoint(double time, double position,
                      double velocity = 0.0, double acceleration = 0.0);

        /**
         * Access to points container
         */
        const std::vector<Point> &points() const;
        std::vector<Point> &points();

        /**
         * Recompute splines interpolation model
         */
        void computeSplines();

    protected:

        /**
         * Inherit
         * Load Points
         */
        virtual void importCallBack();

    private:

        /**
         * Points container
         */
        std::vector<Point> _points;

        /**
         * Fit a polynom between 0 and t with given
         * pos, vel and acc initial and final conditions
         */
        Polynom polynomFit(double t,
                           double pos1, double vel1, double acc1,
                           double pos2, double vel2, double acc2) const;
    };

}

#endif
