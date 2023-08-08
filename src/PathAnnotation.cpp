#include "romea_core_path/PathAnnotation.hpp"

namespace romea
{

PathAnnotation::PathAnnotation(nlohmann::json const & data):
  type(data["type"]),
  value(data["value"]),
  abscissa()
{
  if(data.contains("point_index"))          
  {
    point_index = data["point_index"];
  }
}

}
