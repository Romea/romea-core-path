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
#include <stdexcept>
#include <string>

// romea
#include "romea_core_path/PathFile.hpp"
#include <romea_core_common/geodesy/ENUConverter.hpp>

// json
#include "nlohmann/json.hpp"

static bool endsWith(std::string_view str, std::string_view suffix)
{
  return str.size() >= suffix.size() &&
         0 == str.compare(str.size() - suffix.size(), suffix.size(), suffix);
}

namespace romea
{

//-----------------------------------------------------------------------------
PathFile::PathFile(const std::string & filename)
: coordinate_system_(), world_to_path_(), way_points_(), file_(filename)
{
  if (file_.is_open()) {
    if (endsWith(filename, ".traj")) {
      loadV2_();
    } else {
      loadHeader_();
      loadWayPoints_();
    }
  } else {
    throw(std::runtime_error("Failed to open path file " + filename));
  }

  //  if(revert)
  //  {
  //    std::reverse(std::begin(points_), std::end(points_));
  //  }
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

    romea::GeodeticCoordinates anchor = makeGeodeticCoordinates(
      reference_latitude / 180. * M_PI, reference_longitude / 180. * M_PI, reference_altitude);

    world_to_path_ = ENUConverter(anchor).getEnuToEcefTransform();
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
void PathFile::loadV2_()
{
  auto data = nlohmann::json::parse(file_);

  if (data["version"] != "2") {
    throw std::runtime_error("Only version '2' of trajectory file is currently supported");
  }

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
    romea::GeodeticCoordinates anchor = makeGeodeticCoordinates(
      coords[0].get<double>() / 180. * M_PI, coords[1].get<double>() / 180. * M_PI,
      coords[2].get<double>());

    world_to_path_ = ENUConverter(anchor).getEnuToEcefTransform();
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
  bool has_speed = col_indexes.count("speed");

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

    Eigen::Vector2d pos;
    pos << point[col_indexes["x"]], point[col_indexes["y"]];
    if (has_speed) {
      way_points_.back().emplace_back(pos, point[col_indexes["speed"]]);
    } else {
      way_points_.back().emplace_back(pos);
    }

    ++i;
  }
}

//-----------------------------------------------------------------------------
void PathFile::loadAnnotations(const nlohmann::json & data)
{
  const auto & values = data["annotations"];
  for (const auto & a : values) {
    if (a.contains("point_index")) {
      annotations_.insert({a["point_index"].get<std::size_t>(), a});
    }
  }
}

//-----------------------------------------------------------------------------
const std::vector<std::vector<PathWayPoint2D>> & PathFile::getWayPoints() const
{
  return way_points_;
}

//-----------------------------------------------------------------------------
const std::string & PathFile::getCoordinateSystemDescription() const {return coordinate_system_;}

//-----------------------------------------------------------------------------
const Eigen::Affine3d & PathFile::getWorldToPathTransformation() const {return world_to_path_;}

}  // namespace romea
