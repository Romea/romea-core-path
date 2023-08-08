#ifndef romea_PathAnnotation_hpp
#define romea_PathAnnotation_hpp

#include <string>
#include <nlohmann/json.hpp>

namespace romea
{

struct PathAnnotation
{
  std::string type;
  bool use_point_index;
  std::size_t point_index;
  std::string value;
  double abscissa;

  PathAnnotation(nlohmann::json const & data);
};

inline bool operator<(PathAnnotation const & a, PathAnnotation const & b)
{
  return a.point_index < b.point_index;
}

}

#endif
