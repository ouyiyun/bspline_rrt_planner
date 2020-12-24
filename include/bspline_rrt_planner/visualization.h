#ifndef BSPLINE_RRT_PLANNER_VISUALIZATION_H_
#define BSPLINE_RRT_PLANNER_VISUALIZATION_H_

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include "CubicBezierSpline.h"
#include "Motion.h"

namespace bspline_rrt_planner {
class visualization {
public:
    visualization(ros::NodeHandle& n, std::string frame_id) : nh(n), frame_id_(frame_id) {
        pubBezierCurve = nh.advertise<visualization_msgs::MarkerArray>("/bezierCurve", 10);
        pubTreeNodes = nh.advertise<visualization_msgs::MarkerArray>("/TreeNodes", 10);
        pubDubinsPath = nh.advertise<visualization_msgs::MarkerArray>("/dubins", 10);
    }

    void clear();

    void publishBezierCurve(const std::vector<Point2D>& pts);

    void publishTreeNode(const Point2D& p1, const Point2D& p2);

    void publishDubinsPath(const std::vector<Point2D>& pts);

    void setResolution(double resolution) { this->resolution_ = resolution_; }

    ~visualization() = default;

private:
    ros::NodeHandle nh;

    std::string frame_id_;

    // publish bezier curve
    ros::Publisher pubBezierCurve;

    // publish TreeNodes
    ros::Publisher pubTreeNodes;

    // publish dubins
    ros::Publisher pubDubinsPath;

    // save bezier curve
    visualization_msgs::MarkerArray bcs;

    // save TreeNodes
    visualization_msgs::MarkerArray nodes;

    // save dubins
    visualization_msgs::MarkerArray dubins;

    double resolution_{0.1};
};
} // namespace bspline_rrt_planner
#endif