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

#ifndef ROMEA_CORE_PATH__PATHANNOTATION_HPP_
#define ROMEA_CORE_PATH__PATHANNOTATION_HPP_

// std
#include <string>
#include <vector>

// json
#include "nlohmann/json.hpp"

namespace romea
{
namespace core
{

struct PathAnnotation
{
  std::string type;
  bool use_point_index;
  std::size_t point_index;
  std::string value;
  double abscissa;

  explicit PathAnnotation(nlohmann::json const & data);
};

inline bool operator<(PathAnnotation const & a, PathAnnotation const & b)
{
  return a.point_index < b.point_index;
}

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_PATH__PATHANNOTATION_HPP_
