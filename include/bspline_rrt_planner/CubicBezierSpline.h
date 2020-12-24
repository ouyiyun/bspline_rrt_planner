#ifndef BSPLINE_RRT_BSPLINE_CUBIC_BEZIER_SPLINE_H_
#define BSPLINE_RRT_BSPLINE_CUBIC_BEZIER_SPLINE_H_

#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <vector>

typedef Eigen::RowVector2d Point2D;

namespace bspline_rrt_planner {
class CubicBezierSpline {
public:
    // CubicBezierSpline() = default;
    CubicBezierSpline(double maxCur, double maxAngel);
    ~CubicBezierSpline() = default;

    void setMaxCurvature(double curvature) { this->maxCur_ = curvature; }
    double getMaxCurvature() const { return maxCur_; }

    /** \brief Set the tree length and only related to tree length(hb, he, gb, ge) */
    void setTreeLength(double treeLength);
    double getTreeLength() const { return treeLength_; }

    void calculateControlPoints(const Point2D& W1, const Point2D& W2, const Point2D& W3, std::vector<Point2D>& controlPointsB,
                                std::vector<Point2D>& controlPointsE);

    double calculateAngel(const Point2D& W1, const Point2D& W2, const Point2D& W3);
    double calculateAngel(const Point2D& va, const Point2D& vb);

    bool checkAngle(const Point2D& W1, const Point2D& W2, const Point2D& W3);
    bool checkAngle(const double& angle);

    static void unitVector(const Point2D& pointA, const Point2D& pointB, Point2D& uv);

    void getCubicBezierPoints(const std::vector<Point2D>& controlPoints, std::vector<Point2D>& pts);
    void getCubicBezierPoints(const Point2D& W1, const Point2D& W2, const Point2D& W3, std::vector<Point2D>& bezier_pts);
    void getStraightLinePoints(const Point2D& P1, const Point2D& P2, std::vector<Point2D>& pts);

    template <typename T>
    std::vector<double> linspace(T start_in, T end_in, int num_in);

    double calBezierLength(const std::vector<Point2D>& points);

    void setResolution(double resolution) { this->resolution_ = resolution; }

private:
    /** \brief Set hb, he, gb and ge */
    void setHAndG(double treeLength);

    static const double c1_, c2_, c3_, c4_;

    /** \brief The maximum curvature */
    double maxCur_;

    /** \brief The maximum allowed angle */
    double maxAngel_;

    /** \brief The length of tree(d) */
    double treeLength_;

    /** \brief Only related to tree length */
    double hb_, he_, gb_, ge_;

    /** \brief collision check interval(map resolussion) */
    double resolution_{.5};

    /** \ebrif Related to tree length and variables beta */
    // double kb_, ke_;
};

} // namespace bspline_rrt_planner
#endif