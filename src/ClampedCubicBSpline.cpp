#include "ClampedCubicBSpline.h"

#include <boost/math/constants/constants.hpp>
#include <iostream>

using namespace bspline_rrt_planner;

const double ZERO = 10 * std::numeric_limits<double>::epsilon();
const double pi = boost::math::constants::pi<double>();

ClampedCubicBSpline::ClampedCubicBSpline(double maxCur, double maxAngel) {
    maxCur_ = maxCur;
    maxAngel_ = maxAngel;

    treeLength_ = sin(maxAngel_) * pow((1 + cos(maxAngel_) / 8), -1.5) / (6 * maxCur_);
}

void ClampedCubicBSpline::calculateControlPoints(const Point2D& W1, const Point2D& W2, const Point2D& W3, std::vector<Point2D>& controlPoints) {
    Point2D P1, P2, P3, P4, P5;

    P1 = W1, P3 = W2, P5 = W3;
    P2 = (P1 + P3) / 2;
    P4 = (P3 + P5) / 2;

    controlPoints.clear();
    controlPoints.push_back(P1);
    controlPoints.push_back(P2);
    controlPoints.push_back(P3);
    controlPoints.push_back(P4);
    controlPoints.push_back(P5);
}

void ClampedCubicBSpline::getCubicBSplinePoints(const std::vector<Point2D>& controlPoints, std::vector<Point2D>& pts) {
    if (controlPoints.size() != 5) {
        std::cout << "[ClampedCubicBSpline]: control points num error!\n";
        return;
    }

    pts.clear();

    Eigen::MatrixXd ctlpts(5, 2);
    ctlpts << controlPoints[0], controlPoints[1], controlPoints[2], controlPoints[3], controlPoints[4];
    // std::cout << "ctlpts:\n" << ctlpts << std::endl;

    Eigen::MatrixXd M(4, 5);
    M << 0.5, 1, -1, 1, -0.5, -3, 0, 6, -6, 3, 6, -6, -6, 12, -6, -4, 6, 0, -6, 4;
    // std::cout << "M:\n" << M << std::endl;

    std::vector<double> t = linspace(0.0, 1.0, 200);
    for (uint i = 0; i < t.size(); i++) {
        Eigen::MatrixXd u(1, 4);
        u << 1, t[i], pow(t[i], 2), pow(t[i], 3);
        Point2D pt;
        pt = u * M * ctlpts;
        pts.push_back(pt);
    }
}

template <typename T>
std::vector<double> ClampedCubicBSpline::linspace(T start_in, T end_in, int num_in) {
    std::vector<double> linspaced;

    double start = static_cast<double>(start_in);
    double end = static_cast<double>(end_in);
    double num = static_cast<double>(num_in);

    if (num == 0) {
        return linspaced;
    }
    if (num == 1) {
        linspaced.push_back(start);
        return linspaced;
    }

    double delta = (end - start) / (num - 1);

    for (int i = 0; i < num - 1; ++i) {
        linspaced.push_back(start + delta * i);
    }
    linspaced.push_back(end); // I want to ensure that start and end
                              // are exactly the same as the input
    return linspaced;
}
