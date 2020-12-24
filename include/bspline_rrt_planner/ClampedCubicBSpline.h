#ifndef BSPLINE_RRT_BSPLINE_CLAMPED_CUBIC_B_SPLINE_H_
#define BSPLINE_RRT_BSPLINE_CLAMPED_CUBIC_B_SPLINE_H_

#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <vector>

typedef Eigen::RowVector2d Point2D;

namespace bspline_rrt_planner {
class ClampedCubicBSpline {
public:
    ClampedCubicBSpline(double maxCur, double maxAngel);
    ~ClampedCubicBSpline() = default;

    void setMaxCurvature(double curvature) { this->maxCur_ = curvature; }
    double getMaxCurvature() const { return maxCur_; }

    /** \brief Set the tree length and only related to tree length(hb, he, gb, ge) */
    void setTreeLength(double treeLength);
    double getTreeLength() const { return treeLength_; }

    void calculateControlPoints(const Point2D& W1, const Point2D& W2, const Point2D& W3, std::vector<Point2D>& controlPoints);

    void getCubicBSplinePoints(const std::vector<Point2D>& controlPoints, std::vector<Point2D>& pts);

    template <typename T>
    std::vector<double> linspace(T start_in, T end_in, int num_in);

private:
    double maxCur_, maxAngel_;
    double treeLength_;
};
} // namespace bspline_rrt_planner
#endif
