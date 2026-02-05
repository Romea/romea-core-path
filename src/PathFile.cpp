// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// std
#include <cstdio>
#include <exception>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// romea
#include "romea_core_common/geodesy/ENUConverter.hpp"
#include "romea_core_path/PathFile.hpp"

// json
#include "nlohmann/json.hpp"

static bool endsWith(std::string_view str, std::string_view suffix)
{
  return str.size() >= suffix.size() &&
         0 == str.compare(str.size() - suffix.size(), suffix.size(), suffix);
}

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
PathFile::PathFile(const std::string & filename)
: coordinate_system_(),
  world_to_path_(),
  wgs84_anchor_(),
  way_points_(),
  file_(filename),
  linear_spacing_v4_(0.1)
{
  if (file_.is_open()) {
    auto data = nlohmann::json::parse(file_);
    std::string version = data["version"];
    if (version == "1") {
      loadHeader_();
      loadWayPoints_();
    } else if (version == "2") {
      loadV2_(data);
    } else if (version == "3") {
      loadV3_(data);
    } else {
      throw(std::runtime_error("Version of path file " + filename + " not supported"));
    }
  } else {
    throw(std::runtime_error("Failed to open path file " + filename));
  }
}

//-----------------------------------------------------------------------------
void PathFile::loadHeader_()
{
  std::string header;
  file_ >> header;
  coordinate_system_ = header;

  if (header.compare("WGS84") == 0) {
    double reference_latitude;
    double reference_longitude;
    double reference_altitude;

    file_ >> reference_latitude >> reference_longitude >> reference_altitude;

    wgs84_anchor_ = makeGeodeticCoordinates(
      reference_latitude / 180. * M_PI, reference_longitude / 180. * M_PI, reference_altitude);

    world_to_path_ = ENUConverter(*wgs84_anchor_).getEnuToEcefTransform();
  } else if (header.compare("ENU") == 0 || header.compare("PIXEL") == 0) {
    world_to_path_ = Eigen::Affine3d::Identity();
  } else {
    throw(std::runtime_error("Failed to extract path header"));
  }
}

//-----------------------------------------------------------------------------
void PathFile::loadWayPoints_()
{
  size_t number_of_sections;

  std::string line;
  // parse number of sections (a line with only one number)
  if (file_.get() == '\n' && file_ >> number_of_sections && file_.get() == '\n') {
    std::cout << " number of sections " << number_of_sections << std::endl;
  } else {
    throw std::runtime_error{"Failed to parse the number of sections"};
  }

  way_points_.resize(number_of_sections);
  for (size_t i = 0; i < number_of_sections; ++i) {
    size_t number_of_way_points, number_of_columns;
    if (!(file_ >> number_of_way_points && file_ >> number_of_columns && file_.get() == '\n')) {
      throw std::runtime_error{"Failed to parse the section header"};
    }

    std::cout << " number of way points " << number_of_way_points << std::endl;
    std::cout << " number of columns " << number_of_columns << std::endl;

    way_points_[i].resize(number_of_way_points);
    for (size_t j = 0; j < number_of_way_points; ++j) {
      auto & wp = way_points_[i][j];

      file_ >> wp.position.x();
      file_ >> wp.position.y();

      if (number_of_columns >= 3) {
        file_ >> wp.desired_speed;
      }

      // The 4th column correspond to a marker counter (incremented using joystick)
      if (number_of_columns >= 4) {
        int marker_count;
        file_ >> marker_count;
      }

      // std::cout << j <<" "
      //           << wp.position.x() << " "
      //           << wp.position.y()<<" "
      //           << wp.desired_speed<<std::endl;
    }
  }
}

//-----------------------------------------------------------------------------
void PathFile::loadV2_(const nlohmann::json & data)
{
  loadHeaderV2_(data);
  loadWayPointsV2_(data);
  loadAnnotations(data);
}

//-----------------------------------------------------------------------------
void PathFile::loadHeaderV2_(const nlohmann::json & data)
{
  const auto & origin = data["origin"];
  if (origin["type"] == "WGS84") {
    const auto & coords = origin["coordinates"];
    wgs84_anchor_ = makeGeodeticCoordinates(
      coords[0].get<double>() / 180. * M_PI,
      coords[1].get<double>() / 180. * M_PI,
      coords[2].get<double>());

    world_to_path_ = ENUConverter(*wgs84_anchor_).getEnuToEcefTransform();
    coordinate_system_ = origin["type"];
  } else {
    throw(std::runtime_error("Only 'WGS84' origin type is currently supported"));
  }
}

//-----------------------------------------------------------------------------
void PathFile::loadWayPointsV2_(const nlohmann::json & data)
{
  const auto & points = data["points"];
  const auto & columns = points["columns"];
  const auto & section_indexes = data["sections"];
  const auto & values = points["values"];

  std::map<std::string, std::size_t> col_indexes;
  std::size_t i = 0;
  for (const auto & col : columns) {
    col_indexes[col] = i;
    ++i;
  }

  way_points_.reserve(values.size());

  auto section_it = section_indexes.cbegin();
  i = 0;
  for (const auto & point : values) {
    // Create a new section when the point index reaches the next index in the section list
    if (section_it != section_indexes.cend() && i == *section_it) {
      way_points_.emplace_back();
      ++section_it;
    }

    if (way_points_.empty()) {
      throw std::runtime_error("The first section index of the traj must be 0");
    }

    way_points_.back().emplace_back(PathWayPointFromJSON(col_indexes, point));
    ++i;
  }
}

//-----------------------------------------------------------------------------
void PathFile::loadAnnotations(const nlohmann::json & data)
{
  const auto & values = data["annotations"];
  for (const auto & a : values) {
    if (a.contains("point_index")) {
      annotations_.insert(
        std::pair<size_t, PathAnnotation>(a["point_index"].get<std::size_t>(), a));
    }
  }
}

void PathFile::loadV3_(const nlohmann::json & data)
{
  loadHeaderV3_(data);
  loadWayPointsV3_(data);
}

void PathFile::loadHeaderV3_(const nlohmann::json & data)
{
  const auto & origin = data["origin"];
  if (origin["type"] == "WGS84") {
    const auto & coords = origin["coordinates"];
    wgs84_anchor_ = makeGeodeticCoordinates(
      coords["lat"].get<double>() / 180. * M_PI,
      coords["lon"].get<double>() / 180. * M_PI,
      coords["alt"].get<double>());

    world_to_path_ = ENUConverter(*wgs84_anchor_).getEnuToEcefTransform();
    coordinate_system_ = origin["type"];
  } else {
    throw(std::runtime_error("Only 'WGS84' origin type is currently supported"));
  }
}

void PathFile::loadWayPointsV3_(const nlohmann::json & data)
{
  const auto & points = data["points"];
  way_points_.emplace_back();
  for (const auto & segment : points) {
    std::string segment_type = segment["segment_type"];
    const auto & columns = segment["columns"];
    const auto & values = segment["values"];
    //way_points_.back().reserve(values.size());
    if (segment_type == "row_path") {
      loadRowPathV3_(columns, values);
    } else if (segment_type == "row_line") {
      loadRowLineV3_(columns, values);
    } else if (segment_type == "turn_path") {
      loadTurnPathV3_(columns, values);
    }
  }
}

void PathFile::loadRowPathV3_(const nlohmann::json columns, const nlohmann::json values)
{
  // check if last section is backwards
  if (!way_points_.back().empty() && way_points_.back().front().desired_speed < 0) {
    way_points_.emplace_back();
  }

  std::map<std::string, std::size_t> col_indexes;
  std::size_t i = 0;
  for (const auto & col : columns) {
    col_indexes[col] = i;
    ++i;
  }
  for (const auto & point : values) {
    PathWayPoint2D new_point = PathWayPointFromJSON(col_indexes, point);
    // nan should pass
    assert(!(new_point.desired_speed < 0));
    way_points_.back().emplace_back(new_point);
  }
}

void PathFile::loadRowLineV3_(const nlohmann::json columns, const nlohmann::json values)
{
  // check if last section is backwards
  if (!way_points_.back().empty() && way_points_.back().front().desired_speed < 0) {
    way_points_.emplace_back();
  }

  std::map<std::string, std::size_t> col_indexes;
  std::size_t i = 0;
  for (const auto & col : columns) {
    col_indexes[col] = i;
    ++i;
  }

  PathWayPoint2D start_point = PathWayPointFromJSON(col_indexes, values[0]);
  // nan should pass
  assert(!(start_point.desired_speed < 0));
  for (i = 1; i < values.size(); ++i) {
    const auto & end_value = values[i];

    PathWayPoint2D end_point = PathWayPointFromJSON(col_indexes, end_value);
    // nan should pass
    assert(!(end_point.desired_speed < 0));
    const auto line_points = linearInterpolate_(start_point, end_point);

    way_points_.back().insert(way_points_.back().end(), line_points.cbegin(), line_points.cend());
    start_point = end_point;
  }
}

void PathFile::loadTurnPathV3_(const nlohmann::json columns, const nlohmann::json values)
{
  int curr_sign = 1;
  if (!way_points_.back().empty() && way_points_.back().front().desired_speed < 0) {
    curr_sign = -1;
  }

  std::map<std::string, std::size_t> col_indexes;
  std::size_t i = 0;
  for (const auto & col : columns) {
    col_indexes[col] = i;
    ++i;
  }
  for (const auto & point : values) {
    PathWayPoint2D new_point = PathWayPointFromJSON(col_indexes, point);
    double new_speed = new_point.desired_speed;
    // not a nan and different sign
    if (new_speed == new_speed && curr_sign * new_speed < 0) {
      way_points_.emplace_back();
      curr_sign = -curr_sign;
    }
    way_points_.back().emplace_back(new_point);
  }
}

std::vector<PathWayPoint2D> PathFile::linearInterpolate_(
  const PathWayPoint2D start, const PathWayPoint2D end)
{
  bool const_speed = (start.desired_speed == end.desired_speed);
  Eigen::Vector2d dir = (end.position - start.position).normalized();
  double step = linear_spacing_v4_;
  std::vector<PathWayPoint2D> line_points;

  Eigen::Vector2d pos = start.position;

  while ((pos - start.position).norm() < (end.position - start.position).norm()) {
    if (const_speed) {
      line_points.emplace_back(pos, start.desired_speed);
    } else {
      line_points.emplace_back(pos);
    }
    pos += dir * step;
  }

  line_points.emplace_back(end);
  return line_points;
}

//-----------------------------------------------------------------------------
const std::vector<std::vector<PathWayPoint2D>> & PathFile::getWayPoints() const
{
  return way_points_;
}

//-----------------------------------------------------------------------------
const std::string & PathFile::getCoordinateSystemDescription() const
{
  return coordinate_system_;
}

//-----------------------------------------------------------------------------
const Eigen::Affine3d & PathFile::getWorldToPathTransformation() const
{
  return world_to_path_;
}

//-----------------------------------------------------------------------------
const std::optional<GeodeticCoordinates> & PathFile::getWGS84Anchor() const
{
  return wgs84_anchor_;
}

PathWayPoint2D PathWayPointFromJSON(
  std::map<std::string, std::size_t> col_indexes, const nlohmann::json & point)
{
  bool has_speed = col_indexes.count("speed");

  PathWayPoint2D path_point;
  Eigen::Vector2d pos;
  pos << point[col_indexes["x"]], point[col_indexes["y"]];
  if (has_speed) {
    path_point = PathWayPoint2D(pos, point[col_indexes["speed"]]);
  } else {
    path_point = PathWayPoint2D(pos);
  }
  return path_point;
}

}  // namespace core
}  // namespace romea
