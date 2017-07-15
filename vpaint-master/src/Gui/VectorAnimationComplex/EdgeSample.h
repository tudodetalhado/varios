// Copyright (C) 2012-2016 The VPaint Developers.
// See the COPYRIGHT file at the top-level directory of this distribution
// and at https://github.com/dalboris/vpaint/blob/master/COPYRIGHT
//
// This file is part of VPaint, a vector graphics editor. It is subject to the
// license terms and conditions in the LICENSE.MIT file found in the top-level
// directory of this distribution and at http://opensource.org/licenses/MIT

#ifndef EDGESAMPLE_H
#define EDGESAMPLE_H

#include "Eigen.h"

namespace VectorAnimationComplex
{

class EdgeSample
{
public:
    // Access position
    inline double x() const { return d_[0]; }
    inline double y() const { return d_[1]; }
    inline void setX(double newX) { d_[0] = newX; }
    inline void setY(double newY) { d_[1] = newY; }

    // Access width
    inline double width() const { return d_[2]; }
    inline void setWidth(double newWidth) { d_[2] = newWidth; }

    // Constructor
    EdgeSample(double x = 0, double y = 0, double w = 0): d_(x, y, w) {}
    EdgeSample(const Eigen::Vector3d & d): d_(d) {}

    // Linear interpolation
    EdgeSample lerp(double u, const EdgeSample & other) const
    {
        return EdgeSample((1-u)*d_ + u*other.d_);
    }

    // Distance, in R^2, between two samples
    double distanceTo(const EdgeSample & other) const
    {
        double dx = other.d_[0] - d_[0];
        double dy = other.d_[1] - d_[1];
        double res2 = dx*dx + dy*dy;
        return std::sqrt(res2);
    }

    // Differential
    EdgeSample operator-(const EdgeSample & other) const
    {
        return EdgeSample(d_ - other.d_);
    }
    EdgeSample operator+(const EdgeSample & other) const
    {
        return EdgeSample(d_ + other.d_);
    }

    // For weighted sums
    EdgeSample operator*(double s) const
    {
        return EdgeSample(s * d_);
    }


    // Eigen alignement
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    // 2D position + width
    Eigen::Vector3d d_;
};

}

#endif // EDGESAMPLE_H
