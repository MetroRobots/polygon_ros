/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Locus Robotics
 *  Copyright (c) 2023, Metro Robots
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: David V. Lu!! */

#include <polygon_utils/polygon_utils.hpp>
#include <mapbox/earcut.hpp>

namespace polygon_utils
{
geometry_msgs::msg::Point32 pointToPoint32(const polygon_msgs::msg::Point2D& point)
{
  geometry_msgs::msg::Point32 output;
  output.x = point.x;
  output.y = point.y;
  return output;
}

polygon_msgs::msg::Point2D pointToPoint2D(const geometry_msgs::msg::Point32& point)
{
  polygon_msgs::msg::Point2D output;
  output.x = point.x;
  output.y = point.y;
  return output;
}

geometry_msgs::msg::Polygon polygon2Dto3D(const polygon_msgs::msg::Polygon2D& polygon_2d)
{
  geometry_msgs::msg::Polygon polygon;
  polygon.points.reserve(polygon_2d.points.size());
  for (const auto& pt : polygon_2d.points)
  {
    polygon.points.push_back(pointToPoint32(pt));
  }
  return polygon;
}

polygon_msgs::msg::Polygon2D polygon3Dto2D(const geometry_msgs::msg::Polygon& polygon_3d)
{
  polygon_msgs::msg::Polygon2D polygon;
  polygon.points.reserve(polygon_3d.points.size());
  for (const auto& pt : polygon_3d.points)
  {
    polygon.points.push_back(pointToPoint2D(pt));
  }
  return polygon;
}

geometry_msgs::msg::PolygonStamped polygon2Dto3D(const polygon_msgs::msg::Polygon2DStamped& polygon_2d)
{
  geometry_msgs::msg::PolygonStamped polygon;
  polygon.header = polygon_2d.header;
  polygon.polygon = polygon2Dto3D(polygon_2d.polygon);
  return polygon;
}

polygon_msgs::msg::Polygon2DStamped polygon3Dto2D(const geometry_msgs::msg::PolygonStamped& polygon_3d)
{
  polygon_msgs::msg::Polygon2DStamped polygon;
  polygon.header = polygon_3d.header;
  polygon.polygon = polygon3Dto2D(polygon_3d.polygon);
  return polygon;
}

bool equals(const polygon_msgs::msg::Polygon2D& polygon0, const polygon_msgs::msg::Polygon2D& polygon1)
{
  if (polygon0.points.size() != polygon1.points.size())
  {
    return false;
  }
  for (unsigned int i = 0; i < polygon0.points.size(); i++)
  {
    if (polygon0.points[i].x != polygon1.points[i].x || polygon0.points[i].y != polygon1.points[i].y)
    {
      return false;
    }
  }
  return true;
}

bool equals(const polygon_msgs::msg::ComplexPolygon2D& polygon0, const polygon_msgs::msg::ComplexPolygon2D& polygon1)
{
  if (polygon0.inner.size() != polygon1.inner.size())
  {
    return false;
  }
  if (!equals(polygon0.outer, polygon1.outer))
  {
    return false;
  }
  for (unsigned int i = 0; i < polygon0.inner.size(); i++)
  {
    if (!equals(polygon0.inner[i], polygon1.inner[i]))
    {
      return false;
    }
  }
  return true;
}

polygon_msgs::msg::Polygon2D movePolygonToPose(const polygon_msgs::msg::Polygon2D& polygon,
                                               const geometry_msgs::msg::Pose2D& pose)
{
  polygon_msgs::msg::Polygon2D new_polygon;
  new_polygon.points.resize(polygon.points.size());
  double cos_th = cos(pose.theta);
  double sin_th = sin(pose.theta);
  for (unsigned int i = 0; i < polygon.points.size(); ++i)
  {
    polygon_msgs::msg::Point2D& new_pt = new_polygon.points[i];
    new_pt.x = pose.x + polygon.points[i].x * cos_th - polygon.points[i].y * sin_th;
    new_pt.y = pose.y + polygon.points[i].x * sin_th + polygon.points[i].y * cos_th;
  }
  return new_polygon;
}

polygon_msgs::msg::ComplexPolygon2D movePolygonToPose(const polygon_msgs::msg::ComplexPolygon2D& polygon,
                                                      const geometry_msgs::msg::Pose2D& pose)
{
  polygon_msgs::msg::ComplexPolygon2D new_polygon;
  new_polygon.outer = movePolygonToPose(polygon.outer, pose);
  for (const auto& inner_p : polygon.inner)
  {
    new_polygon.inner.push_back(movePolygonToPose(inner_p, pose));
  }
  return new_polygon;
}

bool isInside(const polygon_msgs::msg::Polygon2D& polygon, const double x, const double y)
{
  // Determine if the given point is inside the polygon using the number of crossings method
  // https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html
  int n = polygon.points.size();
  int cross = 0;
  // Loop from i = [0 ... n - 1] and j = [n - 1, 0 ... n - 2]
  // Ensures first point connects to last point
  for (int i = 0, j = n - 1; i < n; j = i++)
  {
    // Check if the line to x,y crosses this edge
    if (((polygon.points[i].y > y) != (polygon.points[j].y > y)) &&
        (x < (polygon.points[j].x - polygon.points[i].x) * (y - polygon.points[i].y) /
                     (polygon.points[j].y - polygon.points[i].y) +
                 polygon.points[i].x))
    {
      cross++;
    }
  }
  // Return true if the number of crossings is odd
  return cross % 2 > 0;
}
}  // namespace polygon_utils

// Adapt Point2D for use with the triangulation library
namespace mapbox
{
namespace util
{
template <>
struct nth<0, polygon_msgs::msg::Point2D>
{
  inline static double get(const polygon_msgs::msg::Point2D& point)
  {
    return point.x;
  };
};

template <>
struct nth<1, polygon_msgs::msg::Point2D>
{
  inline static double get(const polygon_msgs::msg::Point2D& point)
  {
    return point.y;
  };
};

}  // namespace util
}  // namespace mapbox

namespace polygon_utils
{
using polygon_msgs::msg::Point2D;
using polygon_msgs::msg::Polygon2D;

std::vector<Point2D> triangulate(const polygon_msgs::msg::ComplexPolygon2D& polygon)
{
  // Compute the triangulation
  std::vector<std::vector<Point2D>> rings;
  rings.reserve(1 + polygon.inner.size());
  rings.push_back(polygon.outer.points);
  for (const Polygon2D& inner : polygon.inner)
  {
    rings.push_back(inner.points);
  }
  std::vector<unsigned int> indices = mapbox::earcut<unsigned int>(rings);

  // Create a sequential point index. The triangulation results are indices in this vector.
  std::vector<Point2D> points;
  for (const auto& ring : rings)
  {
    for (const Point2D& point : ring)
    {
      points.push_back(point);
    }
  }

  // Construct the output triangles
  std::vector<Point2D> result;
  result.reserve(indices.size());
  for (unsigned int index : indices)
  {
    result.push_back(points[index]);
  }
  return result;
}

std::vector<Point2D> triangulate(const Polygon2D& polygon)
{
  polygon_msgs::msg::ComplexPolygon2D complex;
  complex.outer = polygon;
  return triangulate(complex);
}

}  // namespace polygon_utils
