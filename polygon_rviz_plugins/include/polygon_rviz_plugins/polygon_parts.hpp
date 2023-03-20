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

#include <polygon_msgs/msg/polygon2_d.hpp>
#include <polygon_msgs/msg/complex_polygon2_d.hpp>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/parse_color.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <string>

/**
 * @brief Several reusable pieces for displaying polygons
 */
namespace polygon_rviz_plugins
{
/**
 * @brief Given a Color Property and an optional Float property, return a ROS Color message
 *
 * @param color_property Color selector
 * @param alpha_property Optional alpha selector. If null, alpha is assumed to be 1.0
 * @return std_msgs::msg::ColorRGBA with all four channels in range [0, 1]
 */
std_msgs::msg::ColorRGBA getColor(rviz_common::properties::ColorProperty* color_property,
                                  rviz_common::properties::FloatProperty* alpha_property = nullptr);

/**
 * @brief Constructs a manual ogre object to display the polygon outline as a line strip
 */
class PolygonOutline
{
public:
  PolygonOutline(Ogre::SceneManager& scene_manager, Ogre::SceneNode& scene_node);
  virtual ~PolygonOutline();

  void reset();

  /**
   * @brief Set the object to display the outline of the given polygon, in a specific color, possibly offset in z
   * @param polygon Polygon to display
   * @param color Ogre color value (doesn't include alpha)
   * @param z_offset Amount in meters to offset each two dimensional pose into the z direction
   */
  void setPolygon(const polygon_msgs::msg::Polygon2D& polygon, const Ogre::ColourValue& color, double z_offset);

protected:
  Ogre::ManualObject* manual_object_;

  Ogre::SceneManager& scene_manager_;
  Ogre::SceneNode& scene_node_;
};

/**
 * @brief Constructs a manual ogre object to display the polygon area as a triangle mesh
 */
class PolygonFill
{
public:
  PolygonFill(Ogre::SceneManager& scene_manager, Ogre::SceneNode& scene_node, const std::string& material_name);
  virtual ~PolygonFill();

  void reset();

  /**
   * @brief Set the object to display the area of the given polygon, in a specific color, possibly offset in z
   * @param polygon Polygon to display
   * @param color ROS color value (includes alpha)
   * @param z_offset Amount in meters to offset each two dimensional pose into the z direction
   */
  void setPolygon(const polygon_msgs::msg::Polygon2D& polygon, const std_msgs::msg::ColorRGBA& color, double z_offset);

  /**
   * @brief Set the object to display the area of the given polygon, in a specific color, possibly offset in z
   * @param polygon Complex Polygon to display
   * @param color ROS color value (includes alpha)
   * @param z_offset Amount in meters to offset each two dimensional pose into the z direction
   */
  void setPolygon(const polygon_msgs::msg::ComplexPolygon2D& polygon, const std_msgs::msg::ColorRGBA& color,
                  double z_offset);

protected:
  Ogre::ManualObject* manual_object_;
  unsigned int last_vertex_count_{0};

  Ogre::SceneManager& scene_manager_;
  Ogre::SceneNode& scene_node_;
  std::string material_name_;
};

/**
 * @brief Wrapper that creates a flat semi-transparent Ogre material and gives it a name
 */
class PolygonMaterial
{
public:
  PolygonMaterial();
  virtual ~PolygonMaterial();
  const std::string& getName() const
  {
    return name_;
  }

protected:
  std::string name_;
  Ogre::MaterialPtr material_;
};

/**
 * @brief Wrapper for EnumProperty + enum for whether to display the outline, area, or both
 */
class PolygonDisplayModeProperty
{
public:
  explicit PolygonDisplayModeProperty(rviz_common::properties::Property* parent);

  template <typename T>
  void connectProperties(T receiver, const char* changed_slot)
  {
    property_->connect(property_, SIGNAL(changed()), receiver, changed_slot);
  }

  bool shouldDrawOutlines() const
  {
    return getDisplayMode() != DisplayMode::FILLED;
  }
  bool shouldDrawFiller() const
  {
    return getDisplayMode() != DisplayMode::OUTLINE;
  }

protected:
  enum struct DisplayMode { OUTLINE, FILLED, FILLED_OUTLINE };
  DisplayMode getDisplayMode() const
  {
    return static_cast<DisplayMode>(property_->getOptionInt());
  }
  rviz_common::properties::EnumProperty* property_;
};

}  // namespace polygon_rviz_plugins
