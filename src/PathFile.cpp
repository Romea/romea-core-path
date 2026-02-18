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

// fields2cover

#include "fields2cover/path_planning/path_planning.h"
#include "fields2cover/path_planning/turning_base.h"
#include "fields2cover/path_planning/dubins_curves.h"
#include "fields2cover/path_planning/dubins_curves_cc.h"
#include "fields2cover/path_planning/reeds_shepp_curves.h"
#include "fields2cover/path_planning/reeds_shepp_curves_hc.h"

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
  std::map<std::string, std::size_t> col_indexes = getColIndexes(columns);
  
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
  const auto & r = data["robot"];
  robot_ = Robot(
      r["width"],
      r["tool_width"],
      1. / double(r["min_curve_radius"]),
      r["max_diff_curve"]);
}

void PathFile::loadWayPointsV4_(const nlohmann::json & data)
{
  const auto & points = data["points"];
  way_points_.emplace_back();
  for (auto it = points.begin(); it != points.end(); ++it) {
    const auto& segment = *it;
    std::string segment_type = segment["segment_type"];
    const auto & columns = segment["columns"];
    const auto & values = segment["values"];
    //way_points_.back().reserve(values.size());
    if (segment_type == "row_path") {
      loadRowPathV4_(columns, values);
    } else if (segment_type == "row_line") {
      loadRowLineV4_(columns, values);
    } else if (segment_type == "turn_path") {
      loadTurnPathV4_(columns, values);
    } else if (segment_type == "turn_segment") {
      const auto & next = *(std::next(it));
      const auto & prec = *(std::prev(it));
      loadTurnV4_(prec, segment, next);
    }
  }
}

void PathFile::loadRowPathV4_(const nlohmann::json columns, const nlohmann::json values)
{
  std::map<std::string, std::size_t> col_indexes = getColIndexes(columns);
  if (newSectionNeeded(col_indexes, values)) {
    way_points_.emplace_back();
  }

  for (const auto & point : values) {
    PathWayPoint2D new_point = PathWayPointFromJSON(col_indexes, point);
    // nan should pass
    assert(!(new_point.desired_speed < 0));
    way_points_.back().emplace_back(new_point);
  }
}

void PathFile::loadRowLineV4_(const nlohmann::json columns, const nlohmann::json values)
{
  std::map<std::string, std::size_t> col_indexes = getColIndexes(columns);

  // if (newSectionNeeded(col_indexes, values)) {
  //   way_points_.emplace_back();
  // }

  addAnnotation("zone_enter", "work", getWaypointsSize());
  PathWayPoint2D start_point = PathWayPointFromJSON(col_indexes, values[0]);
  for (size_t i = 1; i < values.size(); ++i) {
    const auto & end_value = values[i];

    PathWayPoint2D end_point = PathWayPointFromJSON(col_indexes, end_value);
    const auto line_points = linearInterpolate_(start_point, end_point);

    way_points_.back().insert(way_points_.back().end(), line_points.cbegin(), line_points.cend());

    start_point = end_point;
  }
  addAnnotation("zone_exit", "work", getWaypointsSize() - 1);
}

void PathFile::loadTurnV4_(const nlohmann::json prec, const nlohmann::json curr, 
    const nlohmann::json next)
{

  const auto & prec_columns = prec["columns"];
  const auto & curr_columns = curr["columns"];
  const auto & next_columns = next["columns"];

  std::map<std::string, std::size_t> prec_col_indexes = getColIndexes(prec_columns);
  std::map<std::string, std::size_t> curr_col_indexes = getColIndexes(curr_columns);
  std::map<std::string, std::size_t> next_col_indexes = getColIndexes(next_columns);

  // if (newSectionNeeded(curr_col_indexes, values)) {
  //   way_points_.emplace_back();
  // }

  addAnnotation("zone_enter", "uturn", getWaypointsSize());

  const auto & values = curr["values"];
  F2CPoint start {values[0][curr_col_indexes["x"]], values[0][curr_col_indexes["y"]]};
  F2CPoint end {values[1][curr_col_indexes["x"]], values[1][curr_col_indexes["y"]]};

  double angle_start = getSegmentOrientation(prec);
  double angle_end = getSegmentOrientation(next);

  std::unique_ptr<f2c::pp::TurningBase> turn;
  std::string turn_type = curr["turn_type"];
  if (turn_type == "Dubins") 
  {
    turn = std::make_unique<f2c::pp::DubinsCurves>(); 
  }
  else if (turn_type == "DubinsCC")
  {
    turn = std::make_unique<f2c::pp::DubinsCurvesCC>();
  }
  else if (turn_type == "Reeds_Shepp")
  {
    turn = std::make_unique<f2c::pp::ReedsSheppCurves>();
  }
  else if (turn_type == "Reeds_SheppHC") 
  {
    turn = std::make_unique<f2c::pp::ReedsSheppCurvesHC>();
  }
  turn->setDiscretization(linear_spacing_v4_);

  f2c::pp::PathPlanning path_planner;
  F2CPath f2c_path = path_planner.planPathForConnection(
      robot_,
      start, 
      angle_start, 
      F2CMultiPoint(),
      end,
      angle_end,
      *turn);

  bool section_direction = true;
  for (const auto & f2c_point : f2c_path)
  {
    PathWayPoint2D point = PathWayPointFromF2C(f2c_point);
    if (section_direction != (point.desired_speed > 0)) {
      way_points_.emplace_back();
      section_direction = !section_direction;
    }
    way_points_.back().emplace_back(point);
  }

  addAnnotation("zone_exit", "uturn", getWaypointsSize() - 1);
}

void PathFile::loadTurnPathV4_(const nlohmann::json columns, const nlohmann::json values)
{
  std::map<std::string, std::size_t> col_indexes = getColIndexes(columns);
  // if (newSectionNeeded(col_indexes, values)) {
  //   way_points_.emplace_back();
  // }

  size_t next_index = getWaypointsSize();
  addAnnotation("zone_enter", "uturn", next_index);

  PathWayPoint2D prev = PathWayPointFromJSON(col_indexes, values[0]);
  way_points_.back().emplace_back(prev);
  PathWayPoint2D curr = PathWayPointFromJSON(col_indexes, values[1]);
  for (size_t i = 1; i < values.size() - 1; ++i) {
    PathWayPoint2D next = PathWayPointFromJSON(col_indexes, values[i + 1]);
    if (!sameDirection(prev, curr, next)) {
      way_points_.emplace_back();
    }
    way_points_.back().emplace_back(curr);
    prev = curr;
    curr = next;
  }
  way_points_.back().emplace_back(curr);

  addAnnotation("zone_exit", "uturn", next_index + values.size() - 1);
}

bool PathFile::newSectionNeeded(const nlohmann::json columns, const nlohmann::json values)
{
  if (way_points_.back().empty()) {
    return false;
  }
  PathWayPoint2D sec_to_last_point = way_points_.back()[way_points_.size() - 2];
  PathWayPoint2D last_point = way_points_.back().back();
  PathWayPoint2D first_point = PathWayPointFromJSON(columns, values[0]);
  PathWayPoint2D sec_point = PathWayPointFromJSON(columns, values[1]);

  return (
    !sameDirection(sec_to_last_point, last_point, first_point) ||
    !sameDirection(last_point, first_point, sec_point));
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
  PathWayPoint2D path_point {
    Eigen::Vector2d(f2c_point.point.getX(), f2c_point.point.getY()),
    f2c_point.velocity
  };
  if (f2c_point.dir != f2c::types::PathDirection::FORWARD)
  {
    path_point.desired_speed *= -1;
  }
  return path_point;
}

double getSegmentOrientation(const nlohmann::json & segment)
{
  std::map<std::string, std::size_t> col_indexes = getColIndexes(segment["columns"]);
  const auto & values = segment["values"];

  const auto & p1_json = values[values.size() - 2];
  PathWayPoint2D p1 = PathWayPointFromJSON(col_indexes, p1_json);

  const auto & p2_json = values[values.size() - 1];
  PathWayPoint2D p2 = PathWayPointFromJSON(col_indexes, p2_json);

  const auto & vec = p2.position - p1.position; 
  return std::atan2(vec.y(), vec.x());
}

bool sameDirection(const PathWayPoint2D & p1, const PathWayPoint2D & p2, const PathWayPoint2D & p3)
{
  return (p2.position - p1.position).dot(p3.position - p2.position) > 0;
}

std::map<std::string, std::size_t> getColIndexes(const nlohmann::json columns)
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
