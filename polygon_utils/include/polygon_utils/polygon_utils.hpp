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

#pragma once

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <polygon_msgs/msg/point2_d.hpp>
#include <polygon_msgs/msg/polygon2_d.hpp>
#include <polygon_msgs/msg/polygon2_d_stamped.hpp>
#include <polygon_msgs/msg/complex_polygon2_d.hpp>
#include <vector>

namespace polygon_utils
{
// Polygon Conversions
geometry_msgs::msg::Polygon polygon2Dto3D(const polygon_msgs::msg::Polygon2D& polygon_2d);
polygon_msgs::msg::Polygon2D polygon3Dto2D(const geometry_msgs::msg::Polygon& polygon_3d);
geometry_msgs::msg::PolygonStamped polygon2Dto3D(const polygon_msgs::msg::Polygon2DStamped& polygon_2d);
polygon_msgs::msg::Polygon2DStamped polygon3Dto2D(const geometry_msgs::msg::PolygonStamped& polygon_3d);

/**
 * @brief check if two polygons are equal.
 */
bool equals(const polygon_msgs::msg::Polygon2D& polygon0, const polygon_msgs::msg::Polygon2D& polygon1);

/**
 * @brief check if two complex polygons are equal.
 */
bool equals(const polygon_msgs::msg::ComplexPolygon2D& polygon0, const polygon_msgs::msg::ComplexPolygon2D& polygon1);

/**
 * @brief Translate and rotate a polygon to a new pose
 * @param polygon The polygon
 * @param pose The x, y and theta to use when moving the polygon
 * @return A new moved polygon
 */
polygon_msgs::msg::Polygon2D movePolygonToPose(const polygon_msgs::msg::Polygon2D& polygon,
                                               const geometry_msgs::msg::Pose2D& pose);

/**
 * @brief Translate and rotate a polygon to a new pose
 * @param polygon The polygon
 * @param pose The x, y and theta to use when moving the polygon
 * @return A new moved polygon
 */
polygon_msgs::msg::ComplexPolygon2D movePolygonToPose(const polygon_msgs::msg::ComplexPolygon2D& polygon,
                                                      const geometry_msgs::msg::Pose2D& pose);

/**
 * @brief Check if a given point is inside of a polygon
 *
 * Borders are considered outside.
 *
 * @param polygon Polygon to check
 * @param x x coordinate
 * @param y y coordinate
 * @return true if point is inside polygon
 */
bool isInside(const polygon_msgs::msg::Polygon2D& polygon, const double x, const double y);

/**
 * @brief Decompose a complex polygon into a set of triangles.
 *
 * See https://en.wikipedia.org/wiki/Polygon_triangulation
 *
 * Implementation from https://github.com/mapbox/earcut.hpp
 *
 * @param polygon The complex polygon to deconstruct
 * @return A vector of points where each set of three points represents a triangle
 */
std::vector<polygon_msgs::msg::Point2D> triangulate(const polygon_msgs::msg::ComplexPolygon2D& polygon);

/**
 * @brief Decompose a simple polygon into a set of triangles.
 *
 * See https://en.wikipedia.org/wiki/Polygon_triangulation
 *
 * Implementation from https://github.com/mapbox/earcut.hpp
 *
 * @param polygon The polygon to deconstruct
 * @return A vector of points where each set of three points represents a triangle
 */
std::vector<polygon_msgs::msg::Point2D> triangulate(const polygon_msgs::msg::Polygon2D& polygon);

}  // namespace polygon_utils
