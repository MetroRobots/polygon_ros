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

#include <polygon_rviz_plugins/polygons_display.hpp>
#include <polygon_rviz_plugins/validate_floats.hpp>
#include <vector>

namespace polygon_rviz_plugins
{

void PolygonsDisplay::updateStyle()
{
  updateMenuVisibility();
}

void PolygonsDisplay::processMessage(const polygon_msgs::msg::Polygon2DCollection::ConstSharedPtr msg)
{
  std::vector<polygon_msgs::msg::Polygon2D> outlines;
  std::vector<polygon_msgs::msg::ComplexPolygon2D> fillers;

  for (const auto& polygon : msg->polygons)
  {
    if (!validateFloats(polygon))
    {
      setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
                "Message contained invalid floating point values (nans or infs)");
      return;
    }
    outlines.push_back(polygon);
    polygon_msgs::msg::ComplexPolygon2D complex;
    complex.outer = polygon;
    fillers.push_back(complex);
  }

  if (msg->colors.empty())
  {
    saved_colors_.push_back(std_msgs::msg::ColorRGBA());
  }
  else
  {
    saved_colors_ = msg->colors;
  }
  setPolygons(context_, scene_manager_, scene_node_, outlines, fillers, msg->header.frame_id, msg->header.stamp);
  updateColors();
}

}  // namespace polygon_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(polygon_rviz_plugins::PolygonsDisplay, rviz_common::Display)
