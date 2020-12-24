#ifndef BSPLINE_RRT_PLANNER_PLANNER_CORE_H_
#define BSPLINE_RRT_BSPLINE_PLANNER_CORE_H_

#include <assert.h>
#include <random>
#include <vector>

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include "CubicBezierSpline.h"
#include "Motion.h"
#include "NearestNeighborsFLANN.h"
#include "dubins.h"
#include "visualization.h"

namespace bspline_rrt_planner {
class BSplineRRTPlanner : public nav_core::BaseGlobalPlanner {
public:
    /**
     * @brief default constructor
     */
    BSplineRRTPlanner();

    /**
     * @brief constructor
     * @param name planner name
     * @param costmap 2D static cost map as provided by the ros navigation stack
     * @param frame_id global frame ID
     */
    BSplineRRTPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

    ~BSplineRRTPlanner();
    /**
     * @brief  Initialization function for the PlannerCore object
     * @param  name The name of this planner
     * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
     */
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

    /**
     * @brief generate a plan from a start position to a goal position. Usually
     *        called from move_base
     * @param start start position
     * @param goal goal position
     * @param plan the planned path
     * @return whether a valid path was found
     */
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

    /**
     * @brief generate a plan from a service request
     * @param req service request
     * @param resp service response
     * @return whether a valid path was found
     */
    bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);

    void costmapToBitmap(bool**& map, costmap_2d::Costmap2D*& costmap);

    bool solve(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<Point2D>& path);

    void startRegionSample(const Motion& pmotion, Point2D& rstate, double length);

    /** \brief uniform [0,1]*/
    double uniform01() { return uniDist_(generator_); }

    double uniformReal(double lower_bound, double upper_bound) {
        assert(lower_bound <= upper_bound);
        return (upper_bound - lower_bound) * uniDist_(generator_) + lower_bound;
    }

    /**
     * @brief steer function
     * @param req service request
     * @param resp service response
     */
    void steer(const Point2D& nstate, const Point2D& rstate, Point2D& dstate);

    /**
     * @brief Publish a path for visualization purposes
     *
     */
    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

    void motionSteer(const Motion& motion, Motion& dmotion, double length);

    bool findBestNeighbor(Motion* rmotion, Motion*& nmotion);

    bool collisionCheck(const Point2D& pnstate, const Point2D& nstate, const Point2D& dstate);
    bool collisionCheck(const std::vector<Point2D>& pts);
    bool isValidPoint(const Point2D& point);

    bool dubinsShot(const Motion& init, const Motion& end, std::vector<Point2D>& dubinsPath);
    void pruneTree(const std::vector<Point2D>& treeNode, std::vector<Point2D>& puredTreeNode);

    void getPlanFromTree(const std::vector<Point2D>& states, std::vector<Point2D>& path);

    void updateH(const Motion& end, Motion& init);

protected:
    /**
     * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
     */
    costmap_2d::Costmap2D* costmap_;
    std::string frame_id_;
    bool initialized_{false}, allow_unknow_;
    double map_x_upper_, map_x_lower_;
    double map_y_upper_, map_y_lower_;

private:
    /** \brief Compute distance between motions*/
    double distanceFunction(const Motion* a, const Motion* b) const;

    void mapToWorld(double mx, double my, double& wx, double& wy);
    bool worldToMap(double wx, double wy, double& mx, double& my);
    void clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my);
    void outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);

    // double planner_window_x_, planner_window_y_, default_tolerance_;
    boost::mutex mutex_;
    ros::ServiceServer make_plan_srv_;

    // pub plan
    ros::Publisher plan_pub_;

    bool outline_map_;
    float convert_offset_{.5};

    /** \brief A nearest-neighbors datastructure containing the tree of motions */
    std::shared_ptr<NearestNeighbors<Motion*>> nn_;

    /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
    double goal_bias_{.05};

    CubicBezierSpline* bezier_;
    double maxCur_, maxAngel_;

    /** \brief penalty angel prefer small angel */
    double penalty_angel_;
    double dubins_shot_range_;

    /** \brief The random seed*/
    // std::random_device rand_dev_;
    std::mt19937 generator_;
    std::uniform_real_distribution<double> uniDist_{0, 1};

    visualization* vis;
};

} // namespace bspline_rrt_planner

#endif