#### Example use for this tiny searcher lib based on [nanoflann](https://github.com/jlblancoc/nanoflann)

Massive thanks to [Jose](https://github.com/jlblancoc) for sharing this adaptation of [flann](https://github.com/mariusmuja/flann) 👏🍻🍻

#### Does my 3D point exist in a given `dense` set/cluster of 3D points?
This tiny library uses `nanoflann's` `kd-tree to` help you answer that question ... in the fastest time possible!
Here's an example of how to use it.
```
   int main()
{
    std::vector<Point> points = readPoints(); // <-- our dense set of points

    std::vector<Point> queryPoints(6);
    queryPoints[0] = Point(4.0, 5.0, 6.0);    // point does not exist in our set
    queryPoints[1] = points[100];             // point does not exist in our set
    queryPoints[2] = Point(6.0, 3.0, 6.0);    // point does not exist in our set
    queryPoints[3] = Point(1.0, 10.0, 600.0); // point does not exist in our set
    queryPoints[4] = points[1000];            // point does not exist in our set
    queryPoints[5] = points[500];             // point does not exist in our set

    for (auto& queryPoint : queryPoints) {
        if (knn::pointFound(points, queryPoint)) {
            std::cout << "-- point found" << std::endl;
        } else {
            std::cout << "-- point not found" << std::endl;
        }
    }
    return 0;
}

```
The `Point` Class used in this example given below:
```
struct Point {
std::array<float, 3> m_xyz {};
Point(float x, float y, float z)
: m_xyz({ x, y, z })
{
}
};
```
