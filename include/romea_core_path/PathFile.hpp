#ifndef romea_PathFile_HPP
#define romea_PathFile_HPP

//std
#include <fstream>
#include <string>

//others
#include <Eigen/Geometry>
#include <boost/optional.hpp>

//romea
#include <romea_core_path/Path2D.hpp>
#include <romea_core_path/PathAnnotation.hpp>
#include <romea_core_path/PathWayPoint2D.hpp>

namespace romea
{

class PathFile
{
public:
  using Annotations = Path2D::Annotations;

public:
  PathFile();
  PathFile(const std::string & filename);

  const std::vector<std::vector<PathWayPoint2D>> & getWayPoints() const;
  const std::string & getCoordinateSystemDescription() const;
  const Eigen::Affine3d & getWorldToPathTransformation() const;
  const Annotations & getAnnotations() const { return annotations_; }

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

}  // namespace romea

#endif
