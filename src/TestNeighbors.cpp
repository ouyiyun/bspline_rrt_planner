#include "NearestNeighborsFLANN.h"

/**...........ros..............**/
#include <ros/ros.h>

#include <functional>
#include <memory>

#include "CubicBezierSpline.h"
#include "Motion.h"

namespace bspline_rrt_planner {
class Test {
public:
    std::shared_ptr<flann::HierarchicalClusteringIndexParams> kd;
    // std::shared_ptr<NearestNeighborsFLANN<Motion *, FLANNDistance<Motion *>>> nn_;
    std::shared_ptr<NearestNeighbors<Motion *>> nn_;

public:
    Test() {
        kd = std::make_shared<flann::HierarchicalClusteringIndexParams>();
        // nn_ = std::make_shared<NearestNeighborsFLANN<Motion *, FLANNDistance<Motion *>>>(kd);
        nn_ = std::make_shared<NearestNeighborsFLANN<Motion *, FLANNDistance<Motion *>>>(kd);

        nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    }
    ~Test() {}
    double distanceFunction(const Motion *a, const Motion *b) const {
        double diffX = a->getX() - b->getX();
        double diffY = a->getY() - b->getY();

        double result = diffX * diffX + diffY * diffY;
        return result;
    }
};
} // namespace bspline_rrt_planner

using namespace bspline_rrt_planner;
int main(int argc, char **argv) {
    ros::init(argc, argv, "neighbors_test");

    Test test;

    Motion *p1 = new Motion();
    Motion *p2 = new Motion();
    Motion *p3 = new Motion();
    Motion *p4 = new Motion();
    Motion *p5 = new Motion();
    Motion *p6 = new Motion();

    Point2D p = Point2D(1, 1);
    // p1->setXY(1, 1);
    p1->setState(p);
    p2->setXY(-1, 0);
    p3->setXY(0, 3);
    p4->setXY(4, 2);
    p5->setXY(0, 0);
    p6->setXY(2, 2);

    test.nn_->add(p1);
    test.nn_->add(p2);
    test.nn_->add(p3);
    test.nn_->add(p4);
    test.nn_->add(p5);
    std::cout << "size() = " << test.nn_->size() << std::endl;

    std::vector<Motion *> nbh;
    test.nn_->nearestK(p6, 10, nbh);
    // test.nn_->list(nbh);
    for (auto it = nbh.begin(); it != nbh.end(); ++it) {
        Motion *nn = *it;
        std::cout << nn->getX() << "  " << nn->getY() << std::endl;
    }

    double c1 = 7.2364;
    double c2 = 0.4 * (sqrt(6) - 1);
    double c3 = (c2 + 4) / (c1 + 6);
    double c4 = pow(c2 + 4, 2) / (54 * c3);

    std::cout << "c1: " << c1 << "\nc2: " << c2 << "\nc3: " << c3 << "\nc4: " << c4 << std::endl;
    std::cout << "Test success!\n";
    return 0;
}