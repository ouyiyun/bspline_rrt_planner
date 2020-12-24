#ifndef BSPLINE_RRT_PLANNER_NEAREST_NEIGHBORS_FLANN_H_
#define BSPLINE_RRT_PLANNER_NEAREST_NEIGHBORS_FLANN_H_

#include "NearestNeighbors.h"

#include <flann/flann.hpp>
#include <functional>
#include <vector>

#include <memory>

namespace bspline_rrt_planner {
template <typename _T>
class FLANNDistance {
public:
    using ElementType = _T;
    using ResultType = double;

    FLANNDistance(const typename NearestNeighbors<_T>::DistanceFunction &distFun) : distFun_(distFun) {}

    template <typename Iterator1, typename Iterator2>
    ResultType operator()(Iterator1 a, Iterator2 b, size_t /*size*/, ResultType /*worst_dist*/ = -1) const {
        return distFun_(*a, *b);
    }

    template <typename U, typename V>
    inline ResultType accum_dist(const U &a, const V &b, int) const {
        return (a - b) * (a - b);
    }

protected:
    const typename NearestNeighbors<_T>::DistanceFunction &distFun_;
};

template <typename _T, typename _Dist = FLANNDistance<_T>>
class NearestNeighborsFLANN : public NearestNeighbors<_T> {
public:
    NearestNeighborsFLANN(std::shared_ptr<flann::IndexParams> params)
        : index_(nullptr), params_(std::move(params)), searchParams_(32, 0., true), dimension_(1) {}

    ~NearestNeighborsFLANN() override {
        if (index_) delete index_;
    }

    void clear() override {
        if (index_) {
            delete index_;
            index_ = nullptr;
        }
        data_.clear();
    }

    bool reportsSortedResults() const override { return searchParams_.sorted; }

    void setDistanceFunction(const typename NearestNeighbors<_T>::DistanceFunction &distFun) override {
        NearestNeighbors<_T>::setDistanceFunction(distFun);
        rebuildIndex();
    }

    void add(const _T &data) override {
        bool rebuild = index_ && (data_.size() + 1 > data_.capacity());

        if (rebuild) rebuildIndex(2 * data_.capacity());

        data_.push_back(data);
        const flann::Matrix<_T> mat(&data_.back(), 1, dimension_);

        if (index_)
            index_->addPoints(mat, std::numeric_limits<float>::max() / size());
        else
            createIndex(mat);
    }
    void add(const std::vector<_T> &data) override {
        if (data.empty()) return;
        unsigned int oldSize = data_.size();
        unsigned int newSize = oldSize + data.size();
        bool rebuild = index_ && (newSize > data_.capacity());

        if (rebuild) rebuildIndex(std::max(2 * oldSize, newSize));

        if (index_) {
            std::copy(data.begin(), data.end(), data_.begin() + oldSize);
            const flann::Matrix<_T> mat(&data_[oldSize], data.size(), dimension_);
            index_->addPoints(mat, std::numeric_limits<float>::max() / size());
        } else {
            data_ = data;
            const flann::Matrix<_T> mat(&data_[0], data_.size(), dimension_);
            createIndex(mat);
        }
    }
    bool remove(const _T &data) override {
        if (!index_) return false;
        auto &elt = const_cast<_T &>(data);
        const flann::Matrix<_T> query(&elt, 1, dimension_);
        std::vector<std::vector<size_t>> indices(1);
        std::vector<std::vector<double>> dists(1);
        index_->knnSearch(query, indices, dists, 1, searchParams_);
        if (*index_->getPoint(indices[0][0]) == data) {
            index_->removePoint(indices[0][0]);
            rebuildIndex();
            return true;
        }
        return false;
    }
    _T nearest(const _T &data) const override {
        if (size()) {
            auto &elt = const_cast<_T &>(data);
            const flann::Matrix<_T> query(&elt, 1, dimension_);
            std::vector<std::vector<size_t>> indices(1);
            std::vector<std::vector<double>> dists(1);
            index_->knnSearch(query, indices, dists, 1, searchParams_);
            return *index_->getPoint(indices[0][0]);
        }
        // throw Exception("No elements found in nearest neighbors data structure");
    }
    void nearestK(const _T &data, std::size_t k, std::vector<_T> &nbh) const override {
        auto &elt = const_cast<_T &>(data);
        const flann::Matrix<_T> query(&elt, 1, dimension_);
        std::vector<std::vector<size_t>> indices;
        std::vector<std::vector<double>> dists;
        k = index_ ? index_->knnSearch(query, indices, dists, k, searchParams_) : 0;
        nbh.resize(k);
        for (std::size_t i = 0; i < k; ++i) nbh[i] = *index_->getPoint(indices[0][i]);
    }
    void nearestR(const _T &data, double radius, std::vector<_T> &nbh) const override {
        auto &elt = const_cast<_T &>(data);
        flann::Matrix<_T> query(&elt, 1, dimension_);
        std::vector<std::vector<size_t>> indices;
        std::vector<std::vector<double>> dists;
        int k = index_ ? index_->radiusSearch(query, indices, dists, radius, searchParams_) : 0;
        nbh.resize(k);
        for (int i = 0; i < k; ++i) nbh[i] = *index_->getPoint(indices[0][i]);
    }

    std::size_t size() const override { return index_ ? index_->size() : 0; }

    void list(std::vector<_T> &data) const override {
        std::size_t sz = size();
        if (sz == 0) {
            data.resize(0);
            return;
        }
        const _T &dummy = *index_->getPoint(0);
        int checks = searchParams_.checks;
        searchParams_.checks = size();
        nearestK(dummy, sz, data);
        searchParams_.checks = checks;
    }

    virtual void setIndexParams(const std::shared_ptr<flann::IndexParams> &params) {
        params_ = params;
        rebuildIndex();
    }

    virtual const std::shared_ptr<flann::IndexParams> &getIndexParams() const { return params_; }

    virtual void setSearchParams(const flann::SearchParams &searchParams) { searchParams_ = searchParams; }

    flann::SearchParams &getSearchParams() { return searchParams_; }

    const flann::SearchParams &getSearchParams() const { return searchParams_; }

    unsigned int getContainerSize() const { return dimension_; }

protected:
    void createIndex(const flann::Matrix<_T> &mat) {
        index_ = new flann::Index<_Dist>(mat, *params_, _Dist(NearestNeighbors<_T>::distFun_));
        index_->buildIndex();
    }

    void rebuildIndex(unsigned int capacity = 0) {
        if (index_) {
            std::vector<_T> data;
            list(data);
            clear();
            if (capacity != 0u) data_.reserve(capacity);
            add(data);
        }
    }

    std::vector<_T> data_;

    flann::Index<_Dist> *index_;

    std::shared_ptr<flann::IndexParams> params_;

    mutable flann::SearchParams searchParams_;

    unsigned int dimension_;
};

template <>
inline void NearestNeighborsFLANN<double, flann::L2<double>>::createIndex(const flann::Matrix<double> &mat) {
    index_ = new flann::Index<flann::L2<double>>(mat, *params_);
    index_->buildIndex();
}

template <typename _T, typename _Dist = FLANNDistance<_T>>
class NearestNeighborsFLANNLinear : public NearestNeighborsFLANN<_T, _Dist> {
public:
    NearestNeighborsFLANNLinear() : NearestNeighborsFLANN<_T, _Dist>(std::shared_ptr<flann::LinearIndexParams>(new flann::LinearIndexParams())) {}
};

template <typename _T, typename _Dist = FLANNDistance<_T>>
class NearestNeighborsFLANNHierarchicalClustering : public NearestNeighborsFLANN<_T, _Dist> {
public:
    NearestNeighborsFLANNHierarchicalClustering()
        : NearestNeighborsFLANN<_T, _Dist>(
              std::shared_ptr<flann::HierarchicalClusteringIndexParams>(new flann::HierarchicalClusteringIndexParams())) {}
};
} // namespace bspline_rrt_planner
#endif