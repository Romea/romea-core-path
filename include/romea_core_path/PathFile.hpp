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

#ifndef ROMEA_CORE_PATH__PATHFILE_HPP_
#define ROMEA_CORE_PATH__PATHFILE_HPP_

// Eigen
#include <Eigen/Geometry>

// std
#include <fstream>
#include <string>
#include <vector>

// romea
#include "romea_core_path/Path2D.hpp"
#include "romea_core_path/PathAnnotation.hpp"
#include "romea_core_path/PathWayPoint2D.hpp"

namespace romea
{
namespace core
{

class PathFile
{
public:
  using Annotations = Path2D::Annotations;

public:
  PathFile();
  explicit PathFile(const std::string & filename);

  const std::vector<std::vector<PathWayPoint2D>> & getWayPoints() const;
  const std::string & getCoordinateSystemDescription() const;
  const Eigen::Affine3d & getWorldToPathTransformation() const;
  const Annotations & getAnnotations() const {return annotations_;}

private:
  void loadHeader_();
  void loadWayPoints_();

  void loadV2_();
  void loadHeaderV2_(const nlohmann::json & data);
  void loadWayPointsV2_(const nlohmann::json & data);
  void loadAnnotations(const nlohmann::json & data);

private:
  std::string coordinate_system_;
  Eigen::Affine3d world_to_path_;
  std::vector<std::vector<PathWayPoint2D>> way_points_;
  std::ifstream file_;
  Annotations annotations_;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_PATH__PATHFILE_HPP_
