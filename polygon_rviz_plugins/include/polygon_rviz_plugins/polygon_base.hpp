/*
 * Software License Agreement (BSD License)
 *
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

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/logging.hpp>
#include <polygon_rviz_plugins/polygon_parts.hpp>
#include <color_util/convert.hpp>
#include <color_util/named_colors.hpp>
#include <OgreManualObject.h>
#include <OgreMaterial.h>
#include <string>
#include <vector>

namespace polygon_rviz_plugins
{
template <typename T>
class PolygonBase : public rviz_common::MessageFilterDisplay<T>
{
public:
  PolygonBase()
  {
    mode_property_ = new PolygonDisplayModeProperty(this);
    outline_color_property_ = new rviz_common::properties::ColorProperty("Outline Color", QColor(36, 64, 142),
                                                                         "Color to draw the polygon.", this);

    filler_color_property_ = new rviz_common::properties::ColorProperty("Fill Color", QColor(165, 188, 255),
                                                                        "Color to fill the polygon.", this);
    filler_alpha_property_ = new rviz_common::properties::FloatProperty(
        "Alpha", 0.8, "Amount of transparency to apply to the filler.", this);
    filler_alpha_property_->setMin(0.0);
    filler_alpha_property_->setMax(1.0);

    zoffset_property_ = new rviz_common::properties::FloatProperty("Z-Offset", 0.0, "Offset in the Z direction.", this);
  }

  virtual ~PolygonBase()
  {
    for (auto& outline_object : outline_objects_)
    {
      delete outline_object;
    }
    for (auto filler_object : filler_objects_)
    {
      delete filler_object;
    }
  }

  void reset() override
  {
    rviz_common::MessageFilterDisplay<T>::reset();
    resetOutlines();
    resetFillers();
  }

protected:
  template <typename RECEIVER_TYPE>
  void connectProperties(RECEIVER_TYPE receiver, const char* changed_slot)
  {
    mode_property_->connectProperties(receiver, changed_slot);
    zoffset_property_->connect(zoffset_property_, SIGNAL(changed()), receiver, changed_slot);
    outline_color_property_->connect(outline_color_property_, SIGNAL(changed()), receiver, changed_slot);
    filler_color_property_->connect(filler_color_property_, SIGNAL(changed()), receiver, changed_slot);
    filler_color_property_->connect(filler_color_property_, SIGNAL(changed()), receiver, changed_slot);
  }

  void resetOutlines()
  {
    for (auto& outline_object : outline_objects_)
    {
      outline_object->reset();
    }
  }
  void resetFillers()
  {
    for (auto filler_object : filler_objects_)
    {
      filler_object->reset();
    }
  }

  void setPolygons(rviz_common::DisplayContext* context, Ogre::SceneManager* scene_manager, Ogre::SceneNode* scene_node,
                   std::vector<polygon_msgs::msg::Polygon2D>& outlines,
                   std::vector<polygon_msgs::msg::ComplexPolygon2D>& fillers, const std::string& frame_id,
                   const rclcpp::Time& time)
  {
    saved_outlines_.swap(outlines);
    saved_fillers_.swap(fillers);

    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if (!context->getFrameManager()->getTransform(frame_id, time, position, orientation))
    {
      RVIZ_COMMON_LOG_DEBUG_STREAM("Error transforming from frame '" << frame_id << "'");
    }

    scene_node->setPosition(position);
    scene_node->setOrientation(orientation);

    unsigned int num_outlines = saved_outlines_.size();

    while (outline_objects_.size() > num_outlines)
    {
      delete outline_objects_.back();
      outline_objects_.pop_back();
    }
    while (outline_objects_.size() < num_outlines)
    {
      outline_objects_.push_back(new PolygonOutline(*scene_manager, *scene_node));
    }

    unsigned int num_fillers = saved_fillers_.size();
    while (filler_objects_.size() > num_fillers)
    {
      delete filler_objects_.back();
      filler_objects_.pop_back();
    }
    while (filler_objects_.size() < num_fillers)
    {
      filler_objects_.push_back(new PolygonFill(*scene_manager, *scene_node, polygon_material_.getName()));
    }
  }

  void setFillColors(std::vector<std_msgs::msg::ColorRGBA>& filler_colors)
  {
    filler_colors_.swap(filler_colors);
    updateProperties();
  }

  void updateProperties()
  {
    double z_offset = zoffset_property_->getFloat();

    resetOutlines();
    if (mode_property_->shouldDrawOutlines())
    {
      Ogre::ColourValue outline_color = rviz_common::properties::qtToOgre(outline_color_property_->getColor());
      for (unsigned int i = 0; i < saved_outlines_.size(); ++i)
      {
        outline_objects_[i]->setPolygon(saved_outlines_[i], outline_color, z_offset);
      }
    }

    if (!mode_property_->shouldDrawFiller() || saved_fillers_.empty())
    {
      resetFillers();
    }
    else
    {
      for (unsigned int i = 0; i < saved_fillers_.size(); ++i)
      {
        filler_objects_[i]->setPolygon(saved_fillers_[i], filler_colors_[i % filler_colors_.size()], z_offset);
      }
    }
  }

  std_msgs::msg::ColorRGBA getFillerColor() const
  {
    return getColor(filler_color_property_, filler_alpha_property_);
  }

  void updateVisibility()
  {
    updateOutlineVisibility();
    setFillerPropertyVisibility(mode_property_->shouldDrawFiller());
  }

  void updateOutlineVisibility()
  {
    if (mode_property_->shouldDrawOutlines())
    {
      outline_color_property_->show();
    }
    else
    {
      outline_color_property_->hide();
    }
  }

  void setFillerPropertyVisibility(bool visible)
  {
    if (visible)
    {
      filler_color_property_->show();
      filler_alpha_property_->show();
    }
    else
    {
      filler_color_property_->hide();
      filler_alpha_property_->hide();
    }
  }

  std::vector<PolygonOutline*> outline_objects_;
  std::vector<polygon_msgs::msg::Polygon2D> saved_outlines_;

  std::vector<PolygonFill*> filler_objects_;
  std::vector<polygon_msgs::msg::ComplexPolygon2D> saved_fillers_;
  std::vector<std_msgs::msg::ColorRGBA> filler_colors_;

  PolygonMaterial polygon_material_;

  PolygonDisplayModeProperty* mode_property_;
  rviz_common::properties::FloatProperty* zoffset_property_;
  rviz_common::properties::ColorProperty* outline_color_property_;
  rviz_common::properties::ColorProperty* filler_color_property_;
  rviz_common::properties::FloatProperty* filler_alpha_property_;
};

template <typename T>
class PolygonsBase : public PolygonBase<T>
{
public:
  PolygonsBase() : PolygonBase<T>()
  {
    color_mode_property_ = new rviz_common::properties::EnumProperty("Fill Color Mode", "Single Color",
                                                                     "Color scheme for coloring each polygon", this);
    color_mode_property_->addOption("Single Color", static_cast<int>(FillColorMode::SINGLE));
    color_mode_property_->addOption("From Message", static_cast<int>(FillColorMode::FROM_MSG));
    color_mode_property_->addOption("Unique", static_cast<int>(FillColorMode::UNIQUE));

    for (const auto& color : color_util::getNamedColors())
    {
      if (color.a == 0.0)
        continue;
      unique_colors_.push_back(color_util::toMsg(color));
    }
  }

protected:
  enum struct FillColorMode { SINGLE, FROM_MSG, UNIQUE };
  FillColorMode getFillColorMode() const
  {
    return static_cast<FillColorMode>(color_mode_property_->getOptionInt());
  }

  void updateMenuVisibility()
  {
    this->updateOutlineVisibility();

    if (this->mode_property_->shouldDrawFiller())
    {
      color_mode_property_->show();

      FillColorMode coloring = getFillColorMode();
      if (coloring == FillColorMode::SINGLE)
      {
        this->setFillerPropertyVisibility(true);
      }
      else
      {
        this->setFillerPropertyVisibility(false);
      }
    }
    else
    {
      color_mode_property_->hide();
      this->setFillerPropertyVisibility(false);
    }
    updateColors();
  }

  void updateColors()
  {
    if (this->mode_property_->shouldDrawFiller())
    {
      std::vector<std_msgs::msg::ColorRGBA> filler_colors;
      FillColorMode coloring = getFillColorMode();
      if (coloring == FillColorMode::SINGLE)
      {
        filler_colors.push_back(this->getFillerColor());
      }
      else if (coloring == FillColorMode::UNIQUE)
      {
        filler_colors = unique_colors_;
      }
      else  // coloring == FillColorMode::FROM_MSG
      {
        filler_colors = saved_colors_;
      }
      this->setFillColors(filler_colors);
    }
    else
    {
      this->updateProperties();
    }
    this->queueRender();
  }

  rviz_common::properties::EnumProperty* color_mode_property_;

  std::vector<std_msgs::msg::ColorRGBA> unique_colors_;
  std::vector<std_msgs::msg::ColorRGBA> saved_colors_;
};

}  // namespace polygon_rviz_plugins
