#ifndef KNN_H
#define KNN_H

#include <string>
#include <vector>

#include "point.h"

namespace knn {

/** pointFound
 *    Wraps nanoflann's knn implementation into a tiny interface
 *
 *  @param points
 *    Set of 3D points (x, y, z).
 *
 *  @param point
 *    query point
 *
 *  @retval
 *    true or false, i.e., found or not found
 * */
bool pointFound(std::vector<Point>& points, const Point& point);
}
#endif /* KNN_H */
