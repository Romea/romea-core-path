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
#include <cmath>
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

// fields2cover

#include "fields2cover/path_planning/dubins_curves.h"
#include "fields2cover/path_planning/dubins_curves_cc.h"
#include "fields2cover/path_planning/path_planning.h"
#include "fields2cover/path_planning/reeds_shepp_curves.h"
#include "fields2cover/path_planning/reeds_shepp_curves_hc.h"
#include "fields2cover/path_planning/turning_base.h"

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
    } else if (version == "4") {
      loadV4_(data);
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

  // std::map<std::string, std::size_t> col_indexes;
  // std::size_t i = 0;
  // for (const auto & col : columns) {
  //   col_indexes[col] = i;
  //   ++i;
  // }
  std::map<std::string, std::size_t> col_indexes = getColIndices(columns);

  way_points_.reserve(values.size());

  auto section_it = section_indexes.cbegin();
  std::size_t i = 0;
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

void PathFile::addAnnotation(const std::string type, const std::string value, const size_t index)
{
  nlohmann::json annotation{
    {"type", type}, {"use_point_index", true}, {"value", value}, {"point_index", index}};
  annotations_.insert(
    std::pair<size_t, PathAnnotation>(annotation["point_index"].get<std::size_t>(), annotation));
}

void PathFile::loadV4_(const nlohmann::json & data)
{
  loadHeaderV4_(data);
  loadRobotV4_(data);
  loadWayPointsV4_(data);
}

void PathFile::loadHeaderV4_(const nlohmann::json & data)
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

void PathFile::loadRobotV4_(const nlohmann::json & data)
{
  if (data.contains("robot")) {
    const auto & r = data["robot"];
    robot_ =
      Robot(r["width"], r["tool_width"], 1. / double(r["min_curve_radius"]), r["max_diff_curve"]);
  }
}

void PathFile::loadWayPointsV4_(const nlohmann::json & data)
{
  const auto & points = data["points"];
  way_points_.emplace_back();
  forward_section_ = true;
  for (auto it = points.begin(); it != points.end(); ++it) {
    const auto & segment = *it;
    std::string segment_type = segment["segment_type"];
    //way_points_.back().reserve(values.size());
    if (segment_type == "row_path") {
      loadRowPathV4_(segment);
    } else if (segment_type == "row_line") {
      loadRowLineV4_(segment);
    } else if (segment_type == "turn_path") {
      loadTurnPathV4_(segment);
    } else if (segment_type == "turn_segment") {
      if (it == points.begin() || std::next(it) == points.end()) {
        throw std::runtime_error(
          "turn_segment cannot be the first or last segment in a v4 trajectory");
      }
      const auto & next = *(std::next(it));
      const auto & prec = *(std::prev(it));
      loadTurnV4_(prec, segment, next);
    }
  }
}

void PathFile::loadRowPathV4_(const nlohmann::json & segment)
{
  const auto & columns = segment["columns"];
  const auto & values = segment["values"];
  std::map<std::string, std::size_t> col_indexes = getColIndices(columns);

  bool has_speed = col_indexes.count("speed") > 0;
  bool has_working_zone = col_indexes.count("working_zone") > 0;
  bool in_working_zone = false;

  for (const auto & point : values) {
    PathWayPoint2D new_point = PathWayPointFromJSON(col_indexes, point);

    if (has_speed && new_point.desired_speed < 0) {
      throw std::runtime_error("row_path segment contains a negative speed value");
    }

    if (has_working_zone) {
      bool working = (point[col_indexes["working_zone"]].get<int>() == 1);
      size_t current_index = getWaypointsSize();

      if (working && !in_working_zone) {
        addAnnotation("zone_enter", "work", current_index);
        in_working_zone = true;
      } else if (!working && in_working_zone) {
        addAnnotation("zone_exit", "work", current_index);
        in_working_zone = false;
      }
    }

    addPoint(new_point);
  }

  if (in_working_zone) {
    addAnnotation("zone_exit", "work", getWaypointsSize() - 1);
  }
}

void PathFile::loadRowLineV4_(const nlohmann::json & segment)
{
  const auto & columns = segment["columns"];
  const auto & values = segment["values"];
  std::map<std::string, std::size_t> col_indexes = getColIndices(columns);

  bool has_speed = col_indexes.count("speed") > 0;
  bool has_working_zone = col_indexes.count("working_zone") > 0;
  bool in_working_zone = false;

  PathWayPoint2D start_point = PathWayPointFromJSON(col_indexes, values[0]);
  if (has_speed && start_point.desired_speed < 0) {
    throw std::runtime_error("row_line segment contains a negative speed value");
  }
  // add p0 only if it is the very first point of the trajectory;
  // otherwise the previous segment already added it as its last point
  if (getWaypointsSize() == 0) {
    addPoint(start_point);
  }

  for (size_t i = 1; i < values.size(); ++i) {
    PathWayPoint2D end_point = PathWayPointFromJSON(col_indexes, values[i]);
    if (has_speed && end_point.desired_speed < 0) {
      throw std::runtime_error("row_line segment contains a negative speed value");
    }

    if (has_working_zone) {
      // working_zone[i-1] controls segment (p_{i-1} -> p_i)
      // last control point's working_zone is never read (i-1 stops at values.size()-2)
      bool working = (values[i - 1][col_indexes["working_zone"]].get<int>() == 1);
      // p_{i-1} is already in the trajectory at index getWaypointsSize()-1

      if (working && !in_working_zone) {
        addAnnotation("zone_enter", "work", getWaypointsSize() - 1);
        in_working_zone = true;
      } else if (!working && in_working_zone) {
        // p_{i-1} is the shared boundary point: first point of the non-working segment
        addAnnotation("zone_exit", "work", getWaypointsSize() - 1);
        in_working_zone = false;
      }
    }

    // skip the first interpolated point: p_{i-1} is already in the trajectory
    const auto line_points = linearInterpolate_(start_point, end_point);
    for (size_t j = 1; j < line_points.size(); ++j) {
      addPoint(line_points[j]);
    }

    start_point = end_point;
  }

  if (in_working_zone) {
    addAnnotation("zone_exit", "work", getWaypointsSize() - 1);
  }
}

void PathFile::loadTurnV4_(
  const nlohmann::json & prev, const nlohmann::json & curr, const nlohmann::json & next)
{
  const auto & prec_columns = prev["columns"];
  const auto & curr_columns = curr["columns"];
  const auto & next_columns = next["columns"];

  std::map<std::string, std::size_t> prec_col_indexes = getColIndices(prec_columns);
  std::map<std::string, std::size_t> curr_col_indexes = getColIndices(curr_columns);
  std::map<std::string, std::size_t> next_col_indexes = getColIndices(next_columns);

  addAnnotation("zone_enter", "uturn", getWaypointsSize());

  const auto & values = curr["values"];
  F2CPoint start{values[0][curr_col_indexes["x"]], values[0][curr_col_indexes["y"]]};
  F2CPoint end{values[1][curr_col_indexes["x"]], values[1][curr_col_indexes["y"]]};

  double angle_start = getSegmentOrientation(prev);
  double angle_end = getSegmentOrientation(next);

  std::unique_ptr<f2c::pp::TurningBase> turn;
  std::string turn_type = curr["turn_type"];
  if (turn_type == "Dubins") {
    turn = std::make_unique<f2c::pp::DubinsCurves>();
  } else if (turn_type == "DubinsCC") {
    turn = std::make_unique<f2c::pp::DubinsCurvesCC>();
  } else if (turn_type == "Reeds_Shepp") {
    turn = std::make_unique<f2c::pp::ReedsSheppCurves>();
  } else if (turn_type == "Reeds_SheppHC") {
    turn = std::make_unique<f2c::pp::ReedsSheppCurvesHC>();
  }
  turn->setDiscretization(linear_spacing_v4_);

  f2c::pp::PathPlanning path_planner;
  F2CPath f2c_path = path_planner.planPathForConnection(
    robot_, start, angle_start, F2CMultiPoint(), end, angle_end, *turn);

  for (const auto & f2c_point : f2c_path) {
    PathWayPoint2D point = PathWayPointFromF2C(f2c_point);
    addPoint(point);
  }

  addAnnotation("zone_exit", "uturn", getWaypointsSize() - 1);
}

void PathFile::loadTurnPathV4_(const nlohmann::json & segment)
{
  const auto & columns = segment["columns"];
  const auto & values = segment["values"];
  std::map<std::string, std::size_t> col_indexes = getColIndices(columns);

  addAnnotation("zone_enter", "uturn", getWaypointsSize());

  for (const auto & point : values) {
    PathWayPoint2D new_point = PathWayPointFromJSON(col_indexes, point);
    addPoint(new_point);
  }

  addAnnotation("zone_exit", "uturn", getWaypointsSize() - 1);
}

void PathFile::addPoint(const PathWayPoint2D & point)
{
  if (!std::isnan(point.desired_speed) && point.desired_speed && forward_section_ != point.desired_speed > 0) {
    way_points_.emplace_back();
    forward_section_ = !forward_section_;
  }
  way_points_.back().emplace_back(point);
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

std::size_t PathFile::getWaypointsSize()
{
  int i = 0;
  for (const auto & section : way_points_) {
    i += section.size();
  }
  return i;
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

PathWayPoint2D PathWayPointFromF2C(F2CPathState f2c_point)
{
  PathWayPoint2D path_point{
    Eigen::Vector2d(f2c_point.point.getX(), f2c_point.point.getY()), f2c_point.velocity};
  if (f2c_point.dir != f2c::types::PathDirection::FORWARD) {
    path_point.desired_speed *= -1;
  }
  return path_point;
}

double getSegmentOrientation(const nlohmann::json & segment)
{
  std::map<std::string, std::size_t> col_indexes = getColIndices(segment["columns"]);
  const auto & values = segment["values"];
  if (values.size() < 2) {
    throw std::runtime_error("segment adjacent to turn_segment must have at least 2 points");
  }

  const auto & p1_json = values[values.size() - 2];
  PathWayPoint2D p1 = PathWayPointFromJSON(col_indexes, p1_json);

  const auto & p2_json = values[values.size() - 1];
  PathWayPoint2D p2 = PathWayPointFromJSON(col_indexes, p2_json);

  const auto & vec = p2.position - p1.position;
  return std::atan2(vec.y(), vec.x());
}

std::map<std::string, std::size_t> getColIndices(const nlohmann::json columns)
{
  std::map<std::string, std::size_t> col_indexes;
  std::size_t i = 0;
  for (const auto & col : columns) {
    col_indexes[col] = i;
    ++i;
  }
  return col_indexes;
}

}  // namespace core
}  // namespace romea
