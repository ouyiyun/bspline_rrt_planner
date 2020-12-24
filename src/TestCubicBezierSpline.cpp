#include "ClampedCubicBSpline.h"
#include "CubicBezierSpline.h"

#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <boost/math/constants/constants.hpp>

double pi = boost::math::constants::pi<double>();

void publish(std::vector<Point2D> pts, ros::Publisher publisher) {
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";

    for (auto it = pts.begin(); it != pts.end(); ++it) {
        geometry_msgs::PoseStamped pt;
        pt.pose.position.x = (*it)(0);
        pt.pose.position.y = (*it)(1);
        path.poses.push_back(pt);
    }

    publisher.publish(path);
}

using namespace bspline_rrt_planner;
int main(int argc, char **argv) {
    ros::init(argc, argv, "neighbors_test");

    ros::NodeHandle nh;
    ros::Publisher line_pub = nh.advertise<nav_msgs::Path>("line", 1);
    ros::Publisher bezier_pub = nh.advertise<nav_msgs::Path>("bezier", 1);
    ros::Publisher bSpline_pub = nh.advertise<nav_msgs::Path>("bSpline", 1);

    // k = 0.21 maxAngel = 0.5435
    // CubicBezierSpline bezier(0.1, 0.4 * pi);
    CubicBezierSpline bezier(0.21, 0.5435);
    ClampedCubicBSpline bSpline(0.1, 0.4 * pi);

    Point2D p1(0, 0), p2(10.08, 0), p3(13.2, -9.57);
    std::vector<Point2D> line;
    line.push_back(p1);
    line.push_back(p2);
    line.push_back(p3);

    double angel = bezier.calculateAngel(p1, p2, p3);

    std::vector<Point2D> bzA, bzB, ctsBSpline;
    bezier.calculateControlPoints(p1, p2, p3, bzA, bzB);
    bSpline.calculateControlPoints(p1, p2, p3, ctsBSpline);

    std::vector<Point2D> ptsA, ptsB, ptsBspline;
    bezier.getCubicBezierPoints(bzA, ptsA);
    bezier.getCubicBezierPoints(bzB, ptsB);
    bSpline.getCubicBSplinePoints(ctsBSpline, ptsBspline);

    double L = sin(0.5435) * pow((1 + cos(0.5435)) / 8, -1.5) / 6 / 0.21;
    std::cout << "L = " << L << std::endl;

    ros::Rate loop_rate(1);
    while (nh.ok()) {
        publish(line, line_pub);
        publish(ptsA, bezier_pub);
        publish(ptsB, bezier_pub);
        publish(ptsBspline, bSpline_pub);
        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << "angel = " << angel << std::endl;
    std::cout << "Test success!\n";

    return 0;
}