#include <cmath>
#include <nanoflann.hpp>

#include "searcher.h"
#include "utils.h"

template <typename T>
void toNanoflannPoint(
    PointCloud<T>& point, const std::vector<Point>& points)
{
    const size_t N = points.size();
    point.pts.resize(N);
    for (size_t i = 0; i < N; i++) {
        point.pts[i].x = points[i].m_xyz[0];
        point.pts[i].y = points[i].m_xyz[1];
        point.pts[i].z = points[i].m_xyz[2];
    }
}

template <typename num_t>
bool pointFound(
    const std::vector<Point>& points, const size_t& indexOfQueryPoint,
    const size_t& k)
{
    const size_t N = points.size();
    PointCloud<num_t> cloud;

    /** alias kd-tree index */
    typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<
        nanoflann::L2_Simple_Adaptor<num_t, PointCloud<num_t>>,
        PointCloud<num_t>, 3>
        my_kd_tree_t;

    my_kd_tree_t index(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));

    /** adapt points to nanoflann::PointCloud<T> */
    toNanoflannPoint(cloud, points);

    /** parse query point */
    num_t queryPoint[3] = { points[indexOfQueryPoint].m_xyz[0],
        points[indexOfQueryPoint].m_xyz[1],
        points[indexOfQueryPoint].m_xyz[2] };

    /** do knn */
    size_t chunk_size = 100;
    for (size_t i = 0; i < N; i = i + chunk_size) {
        size_t end = std::min(size_t(i + chunk_size), N - 1);

        /** Inserts all points from [i, end] */
        index.addPoints(i, end);
    }
    size_t removePointIndex = N - 1;
    index.removePoint(removePointIndex);
    size_t ret_index[k];
    num_t out_dist_sqr[k];
    nanoflann::KNNResultSet<num_t> resultSet(k);
    resultSet.init(ret_index, out_dist_sqr);
    index.findNeighbors(resultSet, queryPoint, nanoflann::SearchParams(10));

    bool found = false;
    for (size_t i = 0; i < resultSet.size(); ++i) {
    if(out_dist_sqr[i] == 0){
        found = true;
    }
}
    return found;
}



bool knn::compute(
    std::vector<Point>& points, const int& k, const int& indexOfQueryPoint)
{
    bool found = pointFound<float>(points, indexOfQueryPoint, k);
    return  found;
}
