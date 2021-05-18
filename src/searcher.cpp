#include <cmath>
#include <nanoflann.hpp>

#include "searcher.h"
#include "utils.h"

template <typename T>
void toNanoflannPoint(PointCloud<T>& point, const std::vector<Point>& points)
{
    const size_t N = points.size();
    point.pts.resize(N);
    for (size_t i = 0; i < N; i++) {
        point.pts[i].x = points[i].m_xyz[0];
        point.pts[i].y = points[i].m_xyz[1];
        point.pts[i].z = points[i].m_xyz[2];
    }
}

bool findPoint(const std::vector<Point>& points, const Point& point)
{
    const int K = 1; // <-- only find the 1st nearest neighbor
    const size_t N = points.size();
    PointCloud<float> cloud;

    /** alias kd-tree index */
    typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<
        nanoflann::L2_Simple_Adaptor<float, PointCloud<float>>,
        PointCloud<float>, 3>
        my_kd_tree_t;

    my_kd_tree_t index(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));

    /** adapt points to nanoflann::PointCloud<T> */
    toNanoflannPoint(cloud, points);

    /** parse query point */
    float queryPoint[3] = { point.m_xyz[0], point.m_xyz[1], point.m_xyz[2] };

    /** do knn */
    size_t chunk_size = 100;
    for (size_t i = 0; i < N; i = i + chunk_size) {
        size_t end = std::min(size_t(i + chunk_size), N - 1);

        /** Inserts all points from [i, end] */
        index.addPoints(i, end);
    }
    size_t removePointIndex = N - 1;
    index.removePoint(removePointIndex);
    size_t ret_index[K];
    float out_dist_sqr[K];
    nanoflann::KNNResultSet<float> resultSet(K);
    resultSet.init(ret_index, out_dist_sqr);
    index.findNeighbors(resultSet, queryPoint, nanoflann::SearchParams(10));

    bool found = false;
    for (size_t i = 0; i < resultSet.size(); ++i) {
        if (out_dist_sqr[i] == 0) {
            found = true;
        }
    }
    return found;
}

bool searcher::pointFound(std::vector<Point>& points, const Point& point)
{
    bool found = findPoint(points, point);
    return found;
}
