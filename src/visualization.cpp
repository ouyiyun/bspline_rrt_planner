#include "visualization.h"

using namespace bspline_rrt_planner;

void visualization::clear() {
    bcs.markers.clear();

    // clear bezier curve path
    visualization_msgs::MarkerArray BezierCurves;
    visualization_msgs::Marker BezierCurve;
    BezierCurve.header.frame_id = frame_id_;
    BezierCurve.ns = "bezier curve";
    BezierCurve.header.stamp = ros::Time::now();
    BezierCurve.id = 0;
    BezierCurve.action = 3;
    BezierCurves.markers.push_back(BezierCurve);
    pubBezierCurve.publish(BezierCurves);

    // clear dubsin
    visualization_msgs::MarkerArray dubins_markers;
    visualization_msgs::Marker dubins_marker;
    dubins_marker.header.frame_id = frame_id_;
    dubins_marker.ns = "dubins path";
    dubins_marker.header.stamp = ros::Time::now();
    dubins_marker.id = 0;
    dubins_marker.action = 3; // delete all
    dubins_markers.markers.push_back(dubins_marker);
    pubDubinsPath.publish(dubins_markers);

    // clear TreeNodes
    visualization_msgs::Marker node;
    node.header.frame_id = frame_id_;
    node.ns = "TreeNodes";
    node.header.stamp = ros::Time::now();
    node.id = 0;
    node.action = 3;
    nodes.markers.push_back(node);
    pubTreeNodes.publish(nodes);
    nodes.markers.clear();
}

void visualization::publishBezierCurve(const std::vector<Point2D>& pts) {
    visualization_msgs::Marker pt_vis;

    pt_vis.header.frame_id = frame_id_;
    pt_vis.header.stamp = ros::Time();
    pt_vis.ns = "bezier curve";
    pt_vis.type = visualization_msgs::Marker::LINE_STRIP;
    pt_vis.action = visualization_msgs::Marker::ADD;
    pt_vis.id = bcs.markers.size();

    pt_vis.pose.orientation.x = 0.0;
    pt_vis.pose.orientation.y = 0.0;
    pt_vis.pose.orientation.z = 0.0;
    pt_vis.pose.orientation.w = 1.0;

    // red
    pt_vis.color.a = 0.5;
    pt_vis.color.r = 1.0;
    pt_vis.color.g = .0;
    pt_vis.color.b = .0;

    pt_vis.scale.x = resolution_;
    pt_vis.scale.y = .0;
    pt_vis.scale.z = .0;

    geometry_msgs::Point pt;
    for (auto it = pts.begin(); it != pts.begin() + pts.size(); ++it) {
        pt.x = (*it)[0];
        pt.y = (*it)[1];
        pt.z = 0.0;

        pt_vis.points.push_back(pt);
    }

    bcs.markers.push_back(pt_vis);
    // std::cout << "bcs.size() = " << bcs.markers.size() << std::endl;

    pubBezierCurve.publish(bcs);
}

void visualization::publishDubinsPath(const std::vector<Point2D>& pts) {
    visualization_msgs::Marker pt_vis;

    pt_vis.header.frame_id = frame_id_;
    pt_vis.header.stamp = ros::Time();
    pt_vis.ns = "dubins path";
    pt_vis.type = visualization_msgs::Marker::LINE_STRIP;
    pt_vis.action = visualization_msgs::Marker::ADD;
    pt_vis.id = dubins.markers.size();

    pt_vis.pose.orientation.x = 0.0;
    pt_vis.pose.orientation.y = 0.0;
    pt_vis.pose.orientation.z = 0.0;
    pt_vis.pose.orientation.w = 1.0;

    // yellow
    pt_vis.color.a = 1.0;
    pt_vis.color.r = 1.0;
    pt_vis.color.g = 1.0;
    pt_vis.color.b = .0;

    pt_vis.scale.x = resolution_;
    pt_vis.scale.y = .0;
    pt_vis.scale.z = .0;

    geometry_msgs::Point pt;
    for (auto it = pts.begin(); it != pts.begin() + pts.size(); ++it) {
        pt.x = (*it)[0];
        pt.y = (*it)[1];
        pt.z = 0.0;

        pt_vis.points.push_back(pt);
    }

    dubins.markers.push_back(pt_vis);

    pubDubinsPath.publish(dubins);
}

void visualization::publishTreeNode(const Point2D& p1, const Point2D& p2) {
    visualization_msgs::Marker line_strip;

    line_strip.header.frame_id = frame_id_;
    line_strip.header.stamp = ros::Time::now();
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.ns = "TreeNodes";
    line_strip.pose.orientation.w = 1.0;

    line_strip.id = nodes.markers.size();

    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    line_strip.scale.x = resolution_;
    line_strip.scale.y = .0;
    line_strip.scale.z = .0;

    // Purple
    line_strip.color.r = 0.63;
    line_strip.color.g = 0.125;
    line_strip.color.b = 0.941;
    line_strip.color.a = 1.0;

    geometry_msgs::Point pt;
    std::vector<Point2D> pp{p1, p2};
    for (uint i = 0; i < pp.size(); ++i) {
        Eigen::Vector2d coord = pp[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = 0.0;

        line_strip.points.push_back(pt);
    }

    nodes.markers.push_back(line_strip);
    pubTreeNodes.publish(nodes);
}
