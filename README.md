# polygon_ros
Messages and libraries for operating on two dimensional polygons.

In the standard package [`geometry_msgs`](https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/Polygon.msg), `Polygon.msg` and `PolygonStamped.msg` are defined, but have three primary limitations.
 * The `Polygon` is defined by `Point32` which uses `float32` limited precision to store the coordinates.
 * The point is also defined in three dimensions, leading to an often ignored Z coordinate.
 * There is no support for [complex polygons](https://en.wikipedia.org/wiki/Complex_polygon) i.e. polygons with holes in them.

This led to the creation of the new messages defined in this package. This work is forked from the ROS 1 version of the [`robot_navigation`](https://github.com/locusrobotics/robot_navigation).

## Data Types

 * The base datatype is `Point2D`, which defines an x and a y coordinate, in 64 bit precision.
 * The `Polygon2D` type defines a [simple polygon](https://en.wikipedia.org/wiki/Simple_polygon) with an ordered list of `Point2D` that are the vertices of the polygon. The first point is connected to the second, the second is connected to the third, and so on, and the last point is also connected to the first.
 * `Polygon2DStamped` is a single simple polygon with a header.
 * `ComplexPolygon2D` defines a complex polygon with one polygon that is the outer perimeter, and an arbitrary number of polygons that define the holes.
 * `Polygon2DCollection` is a list of simple polygons, all with the same frame. There is also an optional per-polygon colors field for display purposes.
 * Simiarly, `ComplexPolygon2DCollection` is a list of complex polygons, also with a header, and an optional colors field.
