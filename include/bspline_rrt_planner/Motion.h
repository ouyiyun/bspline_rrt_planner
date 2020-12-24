#ifndef BSPLINE_RRT_PLANNER_MOTION_H_
#define BSPLINE_RRT_PLANNER_MOTION_H_

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <vector>

typedef Eigen::RowVector2d Point2D;

namespace bspline_rrt_planner {
class Motion {
public:
    Motion() : state(0, 0), yaw(.0), parent(nullptr) {
        cost = .0;
        costToGoal = .0;
        children.clear();
        bezier_path.clear();
    }

    Motion(double x, double y, double yaw) {
        this->state[0] = x;
        this->state[1] = y;
        this->yaw = yaw;
        parent = nullptr;
        cost = .0;
        costToGoal = .0;
        children.clear();
        bezier_path.clear();
    }

    ~Motion() = default;

    double getX() const { return state[0]; }
    void setX(const double& x) { state[0] = x; }

    double getY() const { return state[1]; }
    void setY(const double& y) { state[1] = y; }

    double getYaw() const { return yaw; }
    void setYaw(const double& yaw) { this->yaw = yaw; }

    Point2D getState() const { return state; }
    void setState(Point2D& state) { this->state = state; }

    void setXY(const double& x, const double& y) {
        state[0] = x;
        state[1] = y;
    }

    Motion* getParent() const { return parent; }
    void setParent(Motion* parent) { this->parent = parent; }

    std::vector<Motion*> getChildren() const { return children; }
    void addChild(Motion* child) { children.push_back(child); }

    double getCost() const { return cost; }
    void setCost(const double& cost) { this->cost = cost; }

    double getCostToGoal() const { return costToGoal; }
    void setCostToGoal(const double& costToGoal) { this->costToGoal = costToGoal; }

    double getAllCost() const { return (cost + costToGoal); }

    void setBezierPath(std::vector<Point2D> pts) { bezier_path = pts; }
    std::vector<Point2D> getBezierPath() const { return bezier_path; }

private:
    Point2D state;

    double yaw;

    Motion* parent;

    double cost;

    double costToGoal;

    std::vector<Motion*> children;

    std::vector<Point2D> bezier_path; // curent node,  curent node's parent and curent node's gradfather
};
} // namespace bspline_rrt_planner
#endif