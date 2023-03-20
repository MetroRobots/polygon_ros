/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Locus Robotics
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

#include <polygon_msgs/msg/point2_d.hpp>
#include <polygon_msgs/msg/polygon2_d.hpp>
#include <rviz_common/validate_floats.hpp>
#include <vector>

namespace polygon_rviz_plugins
{
inline bool validateFloats(const polygon_msgs::msg::Point2D& point)
{
  return rviz_common::validateFloats(point.x) && rviz_common::validateFloats(point.y);
}

template <typename T>
inline bool validateFloats(const std::vector<T>& vec)
{
  for (const auto& element : vec)
  {
    if (!validateFloats(element))
      return false;
  }
  return true;
}

inline bool validateFloats(const polygon_msgs::msg::Polygon2D& msg)
{
  return validateFloats(msg.points);
}

inline bool validateFloats(const polygon_msgs::msg::ComplexPolygon2D& msg)
{
  if (!validateFloats(msg.outer))
  {
    return false;
  }
  for (const auto& inner_p : msg.inner)
  {
    if (!validateFloats(inner_p))
    {
      return false;
    }
  }
  return true;
}

}  // namespace polygon_rviz_plugins
