#include "PlannerCore.h"

#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>

#include <boost/heap/binomial_heap.hpp>
#include <boost/math/constants/constants.hpp>
#include <iostream>

namespace bspline_rrt_planner {

#define VIS_DEBUG false
// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(bspline_rrt_planner::BSplineRRTPlanner, nav_core::BaseGlobalPlanner);

// const double ZERO = 10 * std::numeric_limits<double>::epsilon();
const double pi = boost::math::constants::pi<double>();
const double inf = std::numeric_limits<double>::infinity();

struct costCompare {
    bool operator()(Motion* ma, Motion* mb) const { return (ma->getAllCost() < mb->getAllCost()); }
};

struct allCostCompare {
    bool operator()(const Motion* lhs, const Motion* rhs) const { return lhs->getCostToGoal() > rhs->getCostToGoal(); }
};

// create a border at the edges of the costmap with cost value (e.g. lethal cost for the planner expander not to expand beyond the costmap bounds)
void BSplineRRTPlanner::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value) {
    unsigned char* pc = costarr;
    for (int i = 0; i < nx; i++) {
        *pc++ = value;
    }

    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++) {
        *pc++ = value;
    }

    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx) {
        *pc = value;
    }

    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx) {
        *pc = value;
    }
}

BSplineRRTPlanner::BSplineRRTPlanner() : costmap_(NULL), initialized_(false), allow_unknow_(true) {}

BSplineRRTPlanner::BSplineRRTPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) : BSplineRRTPlanner() {
    // initialize the planner
    initialize(name, costmap, frame_id);
}

/************* TODO ***************/
BSplineRRTPlanner::~BSplineRRTPlanner() {
    if (vis) delete[] vis;
}

void BSplineRRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void BSplineRRTPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) {
    if (!initialized_) {
        ros::NodeHandle private_nh("/" + name);
        costmap_ = costmap;
        frame_id_ = frame_id;
        map_x_upper_ = costmap_->getSizeInMetersX() + costmap_->getOriginX();
        map_x_lower_ = costmap_->getOriginX();
        map_y_upper_ = costmap_->getSizeInMetersY() + costmap_->getOriginY();
        map_y_lower_ = costmap_->getOriginY();

        // unsigned int cx = costmap_->getSizeInCellsX(), cy = costmap_->getSizeInCellsY();

        convert_offset_ = .5;

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

        private_nh.param("goal_bias", goal_bias_, 0.5);
        private_nh.param("allow_unknown", allow_unknow_, true);
        private_nh.param("outline_map", outline_map_, true);
        private_nh.param("max_curvature", maxCur_, 0.28);
        private_nh.param("max_angel", maxAngel_, pi / 3);
        private_nh.param("penalty_angel", penalty_angel_, 10.0);
        private_nh.param("dubins_shot_range", dubins_shot_range_, 10.0);

        // make_plan_srv_ = private_nh.advertiseService("make_plan", &BSplineRRTPlanner::makePlanService, true);

        // choose nearest neighbor struct and set distance function
        nn_ = std::make_shared<NearestNeighborsFLANNHierarchicalClustering<Motion*, FLANNDistance<Motion*>>>();
        nn_->setDistanceFunction([this](const Motion* a, const Motion* b) { return distanceFunction(a, b); });

        // bezier
        bezier_ = new CubicBezierSpline(maxCur_, maxAngel_);
        bezier_->setResolution(costmap_->getResolution());

        ROS_WARN("stepsize = %lf", bezier_->getTreeLength());
        // visualization
        vis = new visualization(private_nh, frame_id_);
        vis->setResolution(costmap_->getResolution());

        initialized_ = true;
    } else {
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
    }
}

bool BSplineRRTPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
    bool solve = makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = frame_id_;

    return solve;
}

void BSplineRRTPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + (mx + convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my + convert_offset_) * costmap_->getResolution();
}

bool BSplineRRTPlanner::worldToMap(double wx, double wy, double& mx, double& my) {
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    if (wx < origin_x || wy < origin_y) return false;

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY()) return true;

    return false;
}

void BSplineRRTPlanner::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my) {
    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

bool BSplineRRTPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                 std::vector<geometry_msgs::PoseStamped>& plan) {
    boost::mutex::scoped_lock lock(mutex_);
    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    plan.clear();

    ros::NodeHandle nh;
    std::string global_frame = frame_id_;

    // until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (goal.header.frame_id != global_frame) {
        ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(),
                  goal.header.frame_id.c_str());
        return false;
    }

    if (start.header.frame_id != global_frame) {
        ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(),
                  start.header.frame_id.c_str());
        return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    // index for start and goal
    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    // double start_x, start_y, goal_x, goal_y;

    // start  out of map
    if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
        ROS_WARN(
            "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly "
            "localized?");
        return false;
    }
    // worldToMap(wx, wy, start_x, start_y);

    // check goal position
    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
        ROS_WARN_THROTTLE(1.0, "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }
    // worldToMap(wx, wy, goal_x, goal_y);

    // clear the starting cell within the costmap because we know it can't be an obstacle
    clearRobotCell(start, start_x_i, start_y_i);

    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();

    // draw border of lethal obstacle around costmap
    if (outline_map_) {
        outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);
    }

    // bool** bitmap;
    // costmapToBitmap(bitmap, costmap_);

    ros::Time t1 = ros::Time::now();
    std::vector<Point2D> path;
    bool solved = solve(start, goal, path);
    ros::Time t2 = ros::Time::now();

    if (!solved) return false;

    ROS_WARN("Plan time: %f ms", (t2 - t1).toSec() * 1000);
    // get plan from Point2D path
    for (auto it = path.begin(); it != path.begin() + path.size(); ++it) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = frame_id_;
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = (*it)[0];
        pose.pose.position.y = (*it)[1];
        pose.pose.orientation.w = 1.0;

        plan.push_back(pose);
    }

    publishPlan(plan);

    return !plan.empty();
}

void BSplineRRTPlanner::costmapToBitmap(bool**& map, costmap_2d::Costmap2D*& costmap) {
    int sizeX, sizeY;

    sizeX = costmap->getSizeInCellsX();
    sizeY = costmap->getSizeInCellsY();

    map = new bool*[sizeX];

    for (int i = 0; i < sizeX; ++i) {
        (map)[i] = new bool[sizeY];
    }

    for (int y = sizeY - 1; y >= 0; y--) {
        for (int x = 0; x < sizeX; x++) {
            unsigned char c = costmap->getCost(x, y);

            if (c == costmap_2d::FREE_SPACE || c == costmap_2d::NO_INFORMATION)
                (map)[x][y] = false; // cell is free
            else
                (map)[x][y] = true; // cell is occupied
        }
    }
}

double BSplineRRTPlanner::distanceFunction(const Motion* a, const Motion* b) const {
    double diffX = a->getX() - b->getX();
    double diffY = a->getY() - b->getY();

    double result = diffX * diffX + diffY * diffY;
    return result;
}

void BSplineRRTPlanner::startRegionSample(const Motion& pmotion, Point2D& rstate, double length) {
    double yaw = pmotion.getYaw();
    double angel = uniformReal(yaw - maxAngel_, yaw + maxAngel_);

    double x = pmotion.getX() + length * cos(angel);
    double y = pmotion.getY() + length * sin(angel);

    rstate = Point2D(x, y);
}

void BSplineRRTPlanner::motionSteer(const Motion& motion, Motion& dmotion, double length) {
    double x = motion.getX() + length * cos(motion.getYaw());
    double y = motion.getY() + length * sin(motion.getYaw());

    dmotion.setXY(x, y);
}

void BSplineRRTPlanner::steer(const Point2D& rstate, const Point2D& nstate, Point2D& dstate) {
    Point2D uv;

    CubicBezierSpline::unitVector(nstate, rstate, uv);

    dstate = nstate + 2.0 * bezier_->getTreeLength() * uv;
}

bool BSplineRRTPlanner::findBestNeighbor(Motion* rmotion, Motion*& nmotion) {
    std::vector<Motion*> neigbors;

    unsigned int k = 10;
    nn_->nearestK(rmotion, k, neigbors);

    Point2D rstate = rmotion->getState();

    std::sort(neigbors.begin(), neigbors.end(), costCompare());

    double bestCost = inf;

    // 查找最优邻居
    for (unsigned int i = 0; i < neigbors.size(); ++i) {
        // 角度检查
        Point2D nstate = neigbors[i]->getState();
        Point2D pstate = neigbors[i]->getParent()->getState();

        double gamma = pi - bezier_->calculateAngel(pstate, nstate, rstate);

        if (bezier_->checkAngle(gamma)) {
            nmotion = neigbors[i];
            // return true;
        }

        // double cost = (neigbors[i]->getState() - rmotion->getState()).norm();
        double cost = penalty_angel_ * pow(gamma, 2) + (neigbors[i]->getState() - rmotion->getState()).norm();
        // double cost = neigbors[i]->getCostToGoal();
        if (cost < bestCost) {
            bestCost = cost;
            nmotion = neigbors[i];
        }
    }

    // return false;
    return bestCost < inf ? true : false;
}

bool BSplineRRTPlanner::isValidPoint(const Point2D& point) {
    unsigned int mx, my;
    if (!costmap_->worldToMap(point[0], point[1], mx, my)) {
        return false;
    }

    unsigned char c = costmap_->getCost(mx, my);
    if (c != costmap_2d::FREE_SPACE && c != costmap_2d::NO_INFORMATION) {
        return false;
    }

    return true;
}

bool BSplineRRTPlanner::collisionCheck(const std::vector<Point2D>& pts) {
    unsigned int mx, my;
    // unsigned int size = pts.size();
    for (auto it = pts.begin(); it != pts.begin() + pts.size(); ++it) {
        if (!isValidPoint(*it)) {
            return false;
        }
    }

    return true;
}

bool BSplineRRTPlanner::collisionCheck(const Point2D& pnstate, const Point2D& nstate, const Point2D& dstate) {
    std::vector<Point2D> pts;

    std::vector<Point2D> controlPointsA, controlPointsB;
    bezier_->calculateControlPoints(pnstate, nstate, dstate, controlPointsA, controlPointsB);

    bezier_->getCubicBezierPoints(controlPointsA, pts);
    if (!collisionCheck(pts)) return false;

    bezier_->getCubicBezierPoints(controlPointsB, pts);
    if (!collisionCheck(pts)) return false;

    return true;
}

bool BSplineRRTPlanner::dubinsShot(const Motion& init, const Motion& end, std::vector<Point2D>& dubinsPath) {
    dubinsPath.clear();
    // start
    double q0[] = {init.getX(), init.getY(), init.getYaw()};
    // goal
    double q1[] = {end.getX(), end.getY(), end.getYaw()};
    // initialize the path
    DubinsPath path;

    // calculate the path
    dubins_init(q0, q1, 1 / maxCur_, &path);

    double dubins_length = dubins_path_length(&path);
    double x = .0;
    dubinsPath.clear();

    while (x < dubins_length) {
        Point2D point;
        double q[3];
        dubins_path_sample(&path, x, q);
        point << q[0], q[1];

        // collision check
        if (isValidPoint(point)) {
            dubinsPath.push_back(point);

            x += costmap_->getResolution();
        } else {
            dubinsPath.clear();
            return false;
        }
    }

    // goal state push
    Point2D point;
    point << q1[0], q1[1];
    dubinsPath.push_back(point);

    return true;
}

void BSplineRRTPlanner::updateH(const Motion& end, Motion& init) {
    Point2D init_state = (init.getState() + init.getParent()->getState()) / 2;

    double q0[] = {init_state[0], init_state[1], init.getYaw()};
    double q1[] = {end.getX(), end.getY(), end.getYaw()};

    DubinsPath path;

    // calculate the path
    dubins_init(q0, q1, 1 / maxCur_, &path);

    double dubins_length = dubins_path_length(&path);
    // double euclidean_distance = (end.getState() - init.getState()).norm();

    init.setCostToGoal(dubins_length);
}

void BSplineRRTPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    // create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

void BSplineRRTPlanner::pruneTree(const std::vector<Point2D>& treeNode, std::vector<Point2D>& puredTreeNode) {
    puredTreeNode.clear();

    if (treeNode.size() < 5) {
        puredTreeNode = treeNode;
        return;
    }

    // initial
    puredTreeNode.push_back(treeNode[0]);
    puredTreeNode.push_back(treeNode[1]);

    // state
    Point2D p1, p2, p3, p4, p5;

    unsigned int n = treeNode.size();

    for (uint i = 2; i < n - 2; ++i) {
        int m = puredTreeNode.size();
        p1 = puredTreeNode[m - 2];
        p2 = puredTreeNode[m - 1];
        p3 = treeNode[i];
        p4 = treeNode[i + 1];
        p5 = treeNode[i + 2];

        // // distance check
        // double dis = (p4 - p2).norm();
        // if (dis < 2 * bezier_->getTreeLength()) {
        //     // p3 is not redundant point
        //     puredTreeNode.push_back(p3);
        //     continue;
        // }

        // check angle
        if (!bezier_->checkAngle(p1, p2, p4) || !bezier_->checkAngle(p2, p4, p5)) {
            // p3 is not redundant point
            puredTreeNode.push_back(p3);
            continue;
        }

        // collision check

        std::vector<Point2D> bc1, bc2, line;
        bezier_->getCubicBezierPoints(p1, p2, p4, bc1);
        bezier_->getCubicBezierPoints(p2, p4, p5, bc2);
        bezier_->getStraightLinePoints(bc1.back(), bc2.front(), line);

        // std::cout << "bc1\n";
        // for (auto it = bc1.begin(); it != bc1.end(); ++it) {
        //     std::cout << *it << std::endl;
        // }
        // std::cout << "bc2\n";
        // for (auto it = bc2.begin(); it != bc2.end(); ++it) {
        //     std::cout << *it << std::endl;
        // }
        // std::cout << "line\n";
        // for (auto it = line.begin(); it != line.end(); ++it) {
        //     std::cout << *it << std::endl;
        // }

        bool bc1_free = collisionCheck(bc1);
        bool bc2_free = collisionCheck(bc2);
        bool line_free = collisionCheck(line);

        if (!bc1_free || !bc1_free || !line_free) {
            puredTreeNode.push_back(p3);
            continue;
        }

        // if (!collisionCheck(bc1); || !collisionCheck(bc2) || !collisionCheck(line)) {
        //     puredTreeNode.push_back(p3);
        //     continue;
        // }
    }

    puredTreeNode.push_back(treeNode[n - 2]);
    puredTreeNode.push_back(treeNode[n - 1]);
}

void BSplineRRTPlanner::getPlanFromTree(const std::vector<Point2D>& states, std::vector<Point2D>& path) {
    path.clear();

    // Point2D bcback, bcFront;

    for (uint i = 0; i < states.size() - 2; ++i) {
        std::vector<Point2D> bc, line;

        bezier_->getCubicBezierPoints(states[i], states[i + 1], states[i + 2], bc);
        path.insert(path.end(), bc.begin(), bc.begin() + bc.size());
    }
}

bool BSplineRRTPlanner::solve(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<Point2D>& path) {
    // visualization clear
    vis->clear();
    nn_->clear();
    path.clear();

    // priority queue priorityQueue;
    boost::heap::binomial_heap<Motion*, boost::heap::compare<allCostCompare>> priorityQueue;
    // boost::heap::binomial_heap<Motion*, boost::heap::compare<allCostCompare>> Q;
    priorityQueue.clear();

    // init tree
    double treeLength = bezier_->getTreeLength();
    Motion* start_motion = new Motion();
    double start_yaw = tf::getYaw(start.pose.orientation);
    start_motion->setXY(start.pose.position.x, start.pose.position.y);
    start_motion->setYaw(start_yaw);

    Motion* goal_motion = new Motion();
    double goal_yaw = tf::getYaw(goal.pose.orientation);
    goal_motion->setXY(goal.pose.position.x, goal.pose.position.y);
    goal_motion->setYaw(goal_yaw);

    // 额外的起点，将其压入最近邻
    Motion* fake_start_motion = new Motion();
    motionSteer(*start_motion, *fake_start_motion, treeLength);
    fake_start_motion->setYaw(start_yaw);
    fake_start_motion->setParent(start_motion);
    updateH(*goal_motion, *fake_start_motion);
    nn_->add(fake_start_motion);
    priorityQueue.push(fake_start_motion);

    int maxIterationNumber = 3000;
    int iterationNumber = 0;

    std::vector<Motion*> Tree;
    Tree.push_back(start_motion);
    Tree.push_back(fake_start_motion);

#if VIS_DEBUG
    vis->publishTreeNode(start_motion->getState(), fake_start_motion->getState());
#endif
    // iterationNumber < maxIterationNumber
    while (1) {
        // sampling state
        ++iterationNumber;
        // add motion
        Motion* dmotion = new Motion();
        Point2D dstate;

        // rand motion and state
        Motion* rmotion = new Motion();
        Point2D rstate;

        // near motion which is the best neighbor
        Motion* nmotion;
        Point2D nstate;

        // nmotion's parent
        Motion* pnmotion;
        Point2D pnstate;

        // std::cout << "[PlannerCore]: nn_->size(): " << nn_->size() << std::endl;
        int nn_size = static_cast<int>(nn_->size());
        double startSampleProbability = std::exp(-(nn_size - 1));
        // std::cout << "[PlannerCore]: startSampleProbability: " << startSampleProbability << std::endl;
        if (uniform01() < startSampleProbability) {
            startRegionSample(*fake_start_motion, rstate, bezier_->getTreeLength());
        } else {
            if (uniform01() < goal_bias_) {
                rstate = goal_motion->getState();
            } else {
                rstate[0] = uniformReal(map_x_lower_, map_x_upper_);
                rstate[1] = uniformReal(map_y_lower_, map_y_upper_);
            }
        }
        rmotion->setState(rstate);

        // find best nearest
        if (!findBestNeighbor(rmotion, nmotion)) {
            delete rmotion;
            // delete nmotion;
            delete dmotion;
            continue;
        }

        rstate = rmotion->getState();
        nstate = nmotion->getState();

        // steer
        steer(rstate, nstate, dstate);

        pnmotion = nmotion->getParent();
        pnstate = pnmotion->getState();

        // vis->publishTreeNode(nstate, dstate);
        // collision check
        std::vector<Point2D> controlPointsA, controlPointsB, ptsA, ptsB, bezier_pts;
        bezier_->calculateControlPoints(pnstate, nstate, dstate, controlPointsA, controlPointsB);
        bezier_->getCubicBezierPoints(controlPointsA, ptsA);
        bezier_->getCubicBezierPoints(controlPointsB, ptsB);
        std::reverse(ptsA.begin(), ptsA.begin() + ptsA.size());
        bezier_pts.insert(bezier_pts.end(), ptsB.begin(), ptsB.begin() + ptsB.size());
        bezier_pts.insert(bezier_pts.end(), ptsA.begin(), ptsA.begin() + ptsA.size());

        // TODO
        if (!collisionCheck(bezier_pts)) {
            delete dmotion;
            delete rmotion;
            continue;
        }

        bool is_in_dubins_shot_range = (dstate - goal_motion->getState()).norm() < dubins_shot_range_;

        // TODO:dubin one shot
        if (nn_size % 50 == 0) {
            // Q = priorityQueue;
            // while (!Q.empty()) {
            //     Motion* pre = Q.top();
            //     Q.pop();
            //     std::cout << "pre state: " << pre->getState() << "  cost: " << pre->getCost() << "  cost to goal: " << pre->getCostToGoal()
            //               << std::endl;
            // }
            Motion* priorityMoiton = priorityQueue.top();
            priorityQueue.pop();
            Motion* ds_motion = new Motion();
            Point2D ds_state = (priorityMoiton->getState() + priorityMoiton->getParent()->getState()) / 2;
            ds_motion->setState(ds_state);
            ds_motion->setYaw(priorityMoiton->getYaw());

            std::vector<Point2D> dubins_path;
            if (dubinsShot(*ds_motion, *goal_motion, dubins_path)) {
                // std::reverse(dubins_path.begin(), dubins_path.begin() + dubins_path.size());
                // path.insert(path.end(), dubins_path.begin(), dubins_path.begin() + dubins_path.size());

                std::vector<Point2D> treeNodes, puredTreeNodes;

                Motion* cur = priorityMoiton;
                while (cur != fake_start_motion) {
                    // std::cout << "[PlannerCore]:cur state " << cur->getState() << std::endl;
                    std::vector<Point2D> cur_path = cur->getBezierPath();
                    // path.insert(path.end(), cur_path.begin(), cur_path.begin() + cur_path.size());
                    treeNodes.push_back(cur->getState());
                    cur = cur->getParent();
                }
                treeNodes[0] = ds_state;
                treeNodes.push_back(fake_start_motion->getState());
                treeNodes.push_back(start_motion->getState());
                std::reverse(treeNodes.begin(), treeNodes.begin() + treeNodes.size());

                pruneTree(treeNodes, puredTreeNodes);
                getPlanFromTree(puredTreeNodes, path);
                path.insert(path.end(), dubins_path.begin(), dubins_path.begin() + dubins_path.size());
#if VIS_DEBUG
                vis->publishDubinsPath(dubins_path);
#endif
                return true;
            } else {
                //! dubins failed, we should remove this node from priorityQueue
                priorityQueue.pop();
            }
        }

        // calculate incCost
        double gamma = bezier_->calculateAngel(pnstate, nstate, dstate);
        double incCost = penalty_angel_ * pow(gamma, 2) + bezier_->calBezierLength(controlPointsA) + bezier_->calBezierLength(controlPointsB);
        double cost = incCost + pnmotion->getCost();

        // set cost、incCost、parent、yaw and state for dmotion
        dmotion->setCost(cost);
        dmotion->setParent(nmotion);
        // dmotion->setYaw();
        dmotion->setState(dstate);

        double dyaw = std::atan2(dstate[1] - nstate[1], dstate[0] - nstate[0]);
        dmotion->setYaw(dyaw);
        // record bezier path
        dmotion->setBezierPath(bezier_pts);

        // update H
        updateH(*goal_motion, *dmotion);

        // add child
        nmotion->addChild(dmotion);

        // add to tree and nn_
        Tree.push_back(dmotion);
        nn_->add(dmotion);
        priorityQueue.push(dmotion);

#if VIS_DEBUG
        vis->publishTreeNode(nstate, dstate);
        vis->publishBezierCurve(bezier_pts);
        ros::Duration(0.01).sleep();
#endif
    }

    // if (iterationNumber < maxIterationNumber)
    //     std::cout << "[PlannerCore]: Found path!\n";
    // else
    //     std::cout << "[PlannerCore]: Can't found path!\n";

    return false;
}
} // namespace bspline_rrt_planner