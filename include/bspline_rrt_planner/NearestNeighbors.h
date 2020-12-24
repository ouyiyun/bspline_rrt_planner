#ifndef BSPLINE_RRT_PLANNER_NEAREST_NEIGHBORS_H_
#define BSPLINE_RRT_PLANNER_NEAREST_NEIGHBORS_H_

#include <functional>
#include <vector>

namespace bspline_rrt_planner {
template <typename _T>
class NearestNeighbors {
public:
    using DistanceFunction = std::function<double(const _T &, const _T &)>;

    NearestNeighbors() = default;

    virtual ~NearestNeighbors() = default;

    virtual void setDistanceFunction(const DistanceFunction &distFun) { distFun_ = distFun; }

    const DistanceFunction &getDistanceFunction() const { return distFun_; }

    virtual bool reportsSortedResults() const = 0;

    virtual void clear() = 0;

    virtual void add(const _T &data) = 0;

    virtual void add(const std::vector<_T> &data) {
        for (const auto &elt : data) add(elt);
    }

    virtual bool remove(const _T &data) = 0;

    virtual _T nearest(const _T &data) const = 0;

    virtual void nearestK(const _T &data, std::size_t k, std::vector<_T> &nbh) const = 0;

    virtual void nearestR(const _T &data, double radius, std::vector<_T> &nbh) const = 0;

    virtual std::size_t size() const = 0;

    virtual void list(std::vector<_T> &data) const = 0;

protected:
    DistanceFunction distFun_;
};
} // namespace bspline_rrt_planner

#endif