#include "CubicBezierSpline.h"

#include <boost/math/constants/constants.hpp>
#include <iostream>

using namespace bspline_rrt_planner;

const double ZERO = 10 * std::numeric_limits<double>::epsilon();
const double pi = boost::math::constants::pi<double>();

const double CubicBezierSpline::c1_ = 7.2364;
const double CubicBezierSpline::c2_ = 0.579796;
const double CubicBezierSpline::c3_ = 0.346;
const double CubicBezierSpline::c4_ = 1.12259;

CubicBezierSpline::CubicBezierSpline(double maxCur, double maxAngel) : maxCur_(maxCur), maxAngel_(maxAngel) {
    double beta = maxAngel_ / 2;

    treeLength_ = c4_ * sin(beta) / (maxCur_ * cos(beta) * cos(beta));
    setHAndG(treeLength_);

    std::cout << "[CubicBezierSpline]: treeLength_" << treeLength_ << "/n";
    std::cout << "[CubicBezierSpline]: initial success!\n";
}

void CubicBezierSpline::calculateControlPoints(const Point2D& W1, const Point2D& W2, const Point2D& W3, std::vector<Point2D>& controlPointsB,
                                               std::vector<Point2D>& controlPointsE) {
    // unit vector u1 and u2
    Point2D u1, u2, ud;

    unitVector(W2, W1, u1);
    unitVector(W2, W3, u2);

    double gamma = pi - calculateAngel(u1, u2);

    double d = treeLength_;
    double ke, kb;
    ke = (6 * c3_ * cos(gamma / 2) * treeLength_) / (c2_ + 4);
    kb = ke;

    Point2D B0, B1, B2, B3;
    Point2D E0, E1, E2, E3;

    B0 = W2 + d * u1;
    B1 = B0 - gb_ * u1;
    B2 = B1 - hb_ * u1;

    E0 = W2 + d * u2;
    E1 = E0 - ge_ * u2;
    E2 = E1 - he_ * u2;

    unitVector(B2, E2, ud);
    B3 = B2 + kb * ud;
    E3 = E2 - ke * ud;

    controlPointsB.clear();
    controlPointsB.push_back(B0);
    controlPointsB.push_back(B1);
    controlPointsB.push_back(B2);
    controlPointsB.push_back(B3);

    controlPointsE.clear();
    controlPointsE.push_back(E0);
    controlPointsE.push_back(E1);
    controlPointsE.push_back(E2);
    controlPointsE.push_back(E3);
}

// unitVector AB
void CubicBezierSpline::unitVector(const Point2D& pointA, const Point2D& pointB, Point2D& uv) {
    Point2D AB = pointB - pointA;
    double ABmod = AB.norm();

    if (ABmod < ZERO)
        uv = Point2D(0, 0);
    else
        uv = AB / ABmod;
}

double CubicBezierSpline ::calculateAngel(const Point2D& W1, const Point2D& W2, const Point2D& W3) {
    Point2D va, vb;

    unitVector(W2, W1, va);
    unitVector(W2, W3, vb);

    return calculateAngel(va, vb);
}

double CubicBezierSpline ::calculateAngel(const Point2D& va, const Point2D& vb) {
    double cosGamma = va.dot(vb) / (va.norm() * vb.norm());
    cosGamma = std::max(-1.0, std::min(cosGamma, 1.0));
    // double gamma = pi - acos(cosGamma);

    return acos(cosGamma);
}

bool CubicBezierSpline::checkAngle(const Point2D& W1, const Point2D& W2, const Point2D& W3) {
    double angel = pi - calculateAngel(W1, W2, W3);
    return checkAngle(angel);
}

bool CubicBezierSpline::checkAngle(const double& angel) {
    if (angel < -ZERO) return false;
    if (angel > maxAngel_) return false;
    return true;
}

void CubicBezierSpline::getCubicBezierPoints(const std::vector<Point2D>& controlPoints, std::vector<Point2D>& pts) {
    pts.clear();

    if (controlPoints.size() != 4) {
        std::cout << "[CubicBezierSpline]: control points num error!\n";
        return;
    }

    // contorl points
    Eigen::MatrixXd ctlpts(4, 2);
    ctlpts << controlPoints[0], controlPoints[1], controlPoints[2], controlPoints[3];

    double length = calBezierLength(controlPoints);

    unsigned int k = std::ceil(length / resolution_ + 1);

    // knot vector
    std::vector<double> t = linspace(0.0, 1.0, k);

    // The basic matrices of cubic bezier curves
    Eigen::MatrixXd M(4, 4);

    M << 1, 0, 0, 0, -3, 3, 0, 0, 3, -6, 3, 0, -1, 3, -3, 1;
    for (uint i = 0; i < t.size(); i++) {
        Eigen::MatrixXd u(1, 4);
        u << 1, t[i], pow(t[i], 2), pow(t[i], 3);
        Point2D pt;
        pt = u * M * ctlpts;
        pts.push_back(pt);
    }
}

void CubicBezierSpline::getCubicBezierPoints(const Point2D& W1, const Point2D& W2, const Point2D& W3, std::vector<Point2D>& bezier_pts) {
    bezier_pts.clear();

    std::vector<Point2D> controlPointsA, controlPointsB, ptsA, ptsB;

    calculateControlPoints(W1, W2, W3, controlPointsA, controlPointsB);
    getCubicBezierPoints(controlPointsA, ptsA);
    getCubicBezierPoints(controlPointsB, ptsB);
    std::reverse(ptsB.begin(), ptsB.begin() + ptsB.size());
    bezier_pts.insert(bezier_pts.end(), ptsA.begin(), ptsA.begin() + ptsA.size());
    bezier_pts.insert(bezier_pts.end(), ptsB.begin(), ptsB.begin() + ptsB.size());
}

void CubicBezierSpline::getStraightLinePoints(const Point2D& P1, const Point2D& P2, std::vector<Point2D>& pts) {
    pts.clear();
    double dis = (P2 - P1).norm();
    int num = std::ceil(dis / this->resolution_) + 1;

    std::vector<double> step = linspace(0.0, dis, num);
    Point2D unit;
    unitVector(P1, P2, unit);

    for (auto it = step.begin(); it != step.end(); ++it) {
        Point2D point;
        point = (*it) * unit + P1;
        pts.push_back(point);
    }
}

double CubicBezierSpline::calBezierLength(const std::vector<Point2D>& points) {
    // Lc: length of the chord
    // Lp: sum of all polygon length
    double Lc, Lp = 0;

    Lc = (points[3] - points[0]).norm();
    for (unsigned int i = 0; i < points.size() - 1; ++i) {
        Lp = Lp + (points[i + 1] - points[i]).norm();
    }

    return (Lc + Lp) / 2;
}

template <typename T>
std::vector<double> CubicBezierSpline::linspace(T start_in, T end_in, int num_in) {
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

void CubicBezierSpline::setHAndG(double treeLength) {
    hb_ = c3_ * treeLength;
    gb_ = c2_ * c3_ * treeLength;

    he_ = hb_;
    ge_ = gb_;
}
