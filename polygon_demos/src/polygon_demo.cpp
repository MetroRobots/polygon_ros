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

#include <rclcpp/rclcpp.hpp>
#include <angles/angles.h>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <polygon_msgs/msg/polygon2_d_stamped.hpp>
#include <polygon_msgs/msg/polygon2_d_collection.hpp>
#include <polygon_msgs/msg/complex_polygon2_d_stamped.hpp>
#include <polygon_msgs/msg/complex_polygon2_d_collection.hpp>
#include <polygon_utils/polygon_utils.hpp>
#include <color_util/named_colors.hpp>
#include <color_util/convert.hpp>
#include <vector>

polygon_msgs::msg::Polygon2D makeStar(double base_angle, double outer = 3.0, double inner = 1.0, unsigned int N = 5)
{
  polygon_msgs::msg::Polygon2D polygon;
  polygon.points.resize(2 * N + 1);

  double inc = M_PI / N;
  for (unsigned int i = 0; i < 2 * N; ++i)
  {
    double length = i % 2 ? outer : inner;
    polygon.points[i].x = length * cos(base_angle + i * inc);
    polygon.points[i].y = length * sin(base_angle + i * inc);
  }
  polygon.points[2 * N] = polygon.points[0];
  return polygon;
}

using color_util::NamedColor;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node node("polygon_display");
  rclcpp::Rate r(33);

  rclcpp::Publisher<polygon_msgs::msg::Polygon2DStamped>::SharedPtr pub0 =
      node.create_publisher<polygon_msgs::msg::Polygon2DStamped>("polygon", 1);
  rclcpp::Publisher<polygon_msgs::msg::Polygon2DCollection>::SharedPtr pub1 =
      node.create_publisher<polygon_msgs::msg::Polygon2DCollection>("polygons", 1);
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub2 =
      node.create_publisher<geometry_msgs::msg::PolygonStamped>("polygon3d", 1);
  rclcpp::Publisher<polygon_msgs::msg::ComplexPolygon2DCollection>::SharedPtr pub3 =
      node.create_publisher<polygon_msgs::msg::ComplexPolygon2DCollection>("complex_polygons", 1);
  rclcpp::Publisher<polygon_msgs::msg::ComplexPolygon2DStamped>::SharedPtr pub4 =
      node.create_publisher<polygon_msgs::msg::ComplexPolygon2DStamped>("complex_polygon", 1);

  double degrees = 0.0;

  std::vector<color_util::ColorRGBA24> rainbow = {
      color_util::get(NamedColor::RED),   color_util::get(NamedColor::ORANGE), color_util::get(NamedColor::YELLOW),
      color_util::get(NamedColor::GREEN), color_util::get(NamedColor::BLUE),   color_util::get(NamedColor::PURPLE)};

  unsigned int CIRCLE_STARS = 100;
  double circle_angle = 2 * M_PI / CIRCLE_STARS;
  polygon_msgs::msg::Polygon2D circle_star = makeStar(0.0, 0.1, 0.05);

  polygon_msgs::msg::ComplexPolygon2D star_outline;
  star_outline.outer = makeStar(0.0, 1.0, 0.8, 10);
  star_outline.inner.push_back(makeStar(0.0, 0.7, 0.6, 12));

  while (rclcpp::ok())
  {
    rclcpp::Time t = node.now();

    polygon_msgs::msg::Polygon2DStamped polygon;
    polygon.header.stamp = t;
    polygon.header.frame_id = "map";
    polygon.polygon.points.resize(11);

    double rad = angles::from_degrees(degrees);
    polygon.polygon = makeStar(rad);
    pub0->publish(polygon);

    polygon_msgs::msg::Polygon2DCollection polygons;
    polygons.header.stamp = t;
    polygons.header.frame_id = "map";

    polygon_msgs::msg::ComplexPolygon2DCollection complex_polygons;
    complex_polygons.header.stamp = t;
    complex_polygons.header.frame_id = "map";

    for (unsigned int i = 0; i < rainbow.size(); ++i)
    {
      polygon_msgs::msg::ComplexPolygon2D complex;
      double w = i * 0.2;
      complex.outer = makeStar(rad, 4.2 + w, 2.2 + w);
      complex.inner.push_back(makeStar(rad, 4.0 + w, 2.0 + w));

      complex_polygons.polygons.push_back(complex);
      complex_polygons.colors.push_back(color_util::toMsg(rainbow[i]));
    }

    for (unsigned int i = 0; i < CIRCLE_STARS; i++)
    {
      geometry_msgs::msg::Pose2D circle_pose;
      double angle = rad + circle_angle * i;
      circle_pose.x = 6.0 * cos(angle);
      circle_pose.y = 6.0 * sin(angle);

      polygons.polygons.push_back(polygon_utils::movePolygonToPose(circle_star, circle_pose));
    }

    pub1->publish(polygons);
    pub3->publish(complex_polygons);

    geometry_msgs::msg::Pose2D pose;
    pose.x = 5.0 * cos(rad);
    pose.y = 5.0 * sin(rad);

    polygon_msgs::msg::Polygon2DStamped small_star;
    small_star.header.stamp = t;
    small_star.header.frame_id = "map";
    small_star.polygon = polygon_utils::movePolygonToPose(makeStar(-rad, 0.5, 0.25), pose);

    pub2->publish(polygon_utils::polygon2Dto3D(small_star));

    polygon_msgs::msg::ComplexPolygon2DStamped complex;
    complex.header.stamp = t;
    complex.header.frame_id = "map";
    complex.polygon = polygon_utils::movePolygonToPose(star_outline, pose);

    pub4->publish(complex);

    r.sleep();
    degrees += 1;
  }

  return 0;
}
