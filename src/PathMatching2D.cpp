//romea
#include "romea_path/PathMatching2D.hpp"
#include <romea_common/math/EulerAngles.hpp>
#include <romea_common/math/Algorithm.hpp>

//std
#include <iterator>
#include <iostream>
#include <list>

namespace {

//-----------------------------------------------------------------------------
size_t findNearestCurveIndex(const romea::PathSection2D & section,
                             const Eigen::Vector2d & vehiclePosition,
                             const romea::Interval<size_t> indexRange,
                             const double & researchRadius)
{

  //  std::cout << indexRange.upper()<< " " << section.size() << std::endl;
  assert(indexRange.upper()<section.size());

  //find neareast point on path
  const auto X = section.getX();
  const auto Y = section.getY();

  size_t nearestPointIndex= indexRange.upper();
  double minimalDistance = researchRadius;

  for(size_t n=indexRange.lower();n<=indexRange.upper();++n)
  {
    double distance = (vehiclePosition-Eigen::Vector2d(X[n],Y[n])).norm();
    //    std::cout << " ditance of " << n << " : " << distance<<" "<<  vehiclePosition.transpose() <<" "<<X[n]<<" "<<Y[n]<< std::endl;

    if(distance < minimalDistance){
      minimalDistance=distance;
      nearestPointIndex=n;
    }
  }

//  std::cout << " nearestPointIndex " <<nearestPointIndex <<" "<< minimalDistance<<" "<< indexRange.upper()<< std::endl;

  return nearestPointIndex;
}


//-----------------------------------------------------------------------------
boost::optional<romea::PathMatchedPoint2D> match_impl(const romea::PathSection2D & section,
                                                      const romea::Pose2D & vehiclePose,
                                                      const double & vehicleSpeed,
                                                      const double & time_horizon,
                                                      const romea::Interval<size_t> & rangeIndex,
                                                      const double & researchRadius)
{

  boost::optional<romea::PathMatchedPoint2D> matchedPoint;

  size_t nearestCurveIndex = findNearestCurveIndex(section,
                                                   vehiclePose.position,
                                                   rangeIndex,
                                                   researchRadius);

  if(nearestCurveIndex!=section.size())
  {
    matchedPoint = match(section.getCurve(nearestCurveIndex),
                         vehiclePose,
                         section.getSpeeds()[nearestCurveIndex]);
  }

  if(matchedPoint.is_initialized())
  {
    matchedPoint->curveIndex = findNearestCurveIndex(section,
                                                     matchedPoint->pathPosture.position,
                                                     section.getCurve(nearestCurveIndex).getIndexInterval(),
                                                     researchRadius);

    matchedPoint->desiredSpeed = section.getSpeeds()[matchedPoint->curveIndex];

    double futureCurvilinearAbscissa = matchedPoint->frenetPose.curvilinearAbscissa+std::abs(vehicleSpeed)* time_horizon;
    size_t futureCurveIndex = section.findIndex(futureCurvilinearAbscissa,matchedPoint->curveIndex);
    matchedPoint->futureCurvature  = section.getCurve(futureCurveIndex).computeCurvature(futureCurvilinearAbscissa);
  }


  return matchedPoint;
}


//-----------------------------------------------------------------------------
boost::optional<romea::PathMatchedPoint2D> match_impl(const romea::PathSection2D & section,
                                                      const romea::Pose2D & vehiclePose,
                                                      const double & vehicleSpeed,
                                                      const double & time_horizon,
                                                      const double & researchRadius)
{
  return match_impl(section,
                    vehiclePose,
                    vehicleSpeed,
                    time_horizon,
                    romea::Interval<size_t>(0,section.size()-1),
                    researchRadius);

}


//----------------------------------------------------------------------------
void match_impl(const romea::Path2D & path,
                const romea::Pose2D & vehiclePose,
                const double & vehicleSpeed,
                const double & time_horizon,
                const double & researchRadius,
                std::list<romea::PathMatchedPoint2D> & matchedPoints)
{
  for(size_t n=0; n< path.size(); ++n)
  {

    //    std::cout << "\n global match section "<< n<< std::endl;
    auto matchedPoint = match_impl(path.getSection(n),
                                   vehiclePose,
                                   vehicleSpeed,
                                   time_horizon,
                                   researchRadius);

    if(matchedPoint)
    {
      matchedPoint->sectionIndex=n;
      matchedPoints.push_back(*matchedPoint);
    }
  }
}


//-----------------------------------------------------------------------------
void match_impl(const romea::Path2D & path,
                const romea::Pose2D & vehiclePose,
                const double & vehicleSpeed,
                const size_t & sectionIndex,
                const size_t & curveIndex,
                const romea::Interval<double> & curvilinearAbscissaResearchInterval,
                const double & time_horizon,
                const double & researchRadius,
                std::list<romea::PathMatchedPoint2D> & matchedPoints)
{


//  std::cout <<"\n\n local"<< std::endl;
  const auto & section = path.getSection(sectionIndex);
  romea::Interval<size_t> rangeIndex = section.
      findIntervalBoundIndexes(curveIndex,curvilinearAbscissaResearchInterval);

  auto matched_point =match_impl(section,
                                 vehiclePose,
                                 vehicleSpeed,
                                 time_horizon,
                                 rangeIndex,
                                 researchRadius);
  if(matched_point.is_initialized())
  {
    matched_point->sectionIndex=sectionIndex;
    matchedPoints.push_back(*matched_point);
  }


  if(rangeIndex.lower()==0 &&
     sectionIndex!=0 &&
     curvilinearAbscissaResearchInterval.lower() < section.getCurvilinearAbscissa().initialValue())
  {

    const auto & previousSection = path.getSection(sectionIndex-1);
    const size_t previousCurveIndex = path.getSection(sectionIndex-1).size()-1;
    romea::Interval<size_t> previousrangeIndex = previousSection.
        findIntervalBoundIndexes(previousCurveIndex,curvilinearAbscissaResearchInterval);


    auto previousMatchedPoint =match_impl(previousSection,
                                            vehiclePose,
                                            vehicleSpeed,
                                            time_horizon,
                                            previousrangeIndex,
                                            researchRadius);
    if(previousMatchedPoint.is_initialized())
    {
      previousMatchedPoint->sectionIndex=sectionIndex-1;
      matchedPoints.push_front(*previousMatchedPoint);
    }

  }


  if(rangeIndex.upper()==section.size()-1 &&
     sectionIndex!=path.size()-1 &&
     curvilinearAbscissaResearchInterval.upper() > section.getCurvilinearAbscissa().finalValue())
  {

    const auto & nextSection = path.getSection(sectionIndex+1);

    romea::Interval<size_t> nextRangeIndex = nextSection.
        findIntervalBoundIndexes(0,curvilinearAbscissaResearchInterval);

    auto nextMatchedPoint =match_impl(nextSection,
                                        vehiclePose,
                                        vehicleSpeed,
                                        time_horizon,
                                        nextRangeIndex,
                                        researchRadius);

    if(nextMatchedPoint.is_initialized())
    {
      nextMatchedPoint->sectionIndex=sectionIndex+1;
      matchedPoints.push_back(*nextMatchedPoint);
    }
  }
}

}

namespace romea {



//----------------------------------------------------------------------------
std::vector<PathMatchedPoint2D>  match(const Path2D & path,
                                       const Pose2D & vehiclePose,
                                       const double & vehicleSpeed,
                                       const double & time_horizon,
                                       const double & researchRadius)
{

  std::list<PathMatchedPoint2D> matchedPoints;

  match_impl(path,
             vehiclePose,
             vehicleSpeed,
             time_horizon,
             researchRadius,
             matchedPoints);

  return std::vector<PathMatchedPoint2D>(matchedPoints.begin(),matchedPoints.end());
}

//----------------------------------------------------------------------------
std::vector<PathMatchedPoint2D> match(const Path2D & path,
                                      const Pose2D & vehiclePose,
                                      const double & vehicleSpeed,
                                      const PathMatchedPoint2D & previousMatchedPoint,
                                      const double & expectedTravelledDistance,
                                      const double & time_horizon,
                                      const double & researchRadius)
{

  double s = previousMatchedPoint.frenetPose.curvilinearAbscissa;
  double mins = s - expectedTravelledDistance/2.;
  double maxs = s + expectedTravelledDistance/2.;

  return match(path,
               vehiclePose,
               vehicleSpeed,
               previousMatchedPoint.sectionIndex,
               previousMatchedPoint.curveIndex,
               Interval<double>(mins,maxs),
               time_horizon,
               researchRadius);
}


//----------------------------------------------------------------------------
std::vector<PathMatchedPoint2D> match(const Path2D & path,
                                      const Pose2D & vehiclePose,
                                      const double & vehicleSpeed,
                                      const size_t & previousSectionIndex,
                                      const size_t & previousCurveIndex,
                                      const Interval<double> & curvilinearAbscissaResearchInterval,
                                      const double & time_horizon,
                                      const double &researchRadius)
{

  std::list<PathMatchedPoint2D> matchedPoints;

  match_impl(path,
             vehiclePose,
             vehicleSpeed,
             previousSectionIndex,
             previousCurveIndex,
             curvilinearAbscissaResearchInterval,
             time_horizon,
             researchRadius,
             matchedPoints);

  return std::vector<PathMatchedPoint2D>(matchedPoints.begin(),matchedPoints.end());

}

//-----------------------------------------------------------------------------
size_t bestMatchedPointIndex(const std::vector<PathMatchedPoint2D> & matchedPoints,
                             const double & vehicleSpeed)
{
  assert(!matchedPoints.empty());

  size_t bestIndex=matchedPoints.size();
  double minimalLateralDeviation = std::numeric_limits<double>::max();
  for(size_t n=0;n<matchedPoints.size();++n)
  {
    double lateralDeviation = std::abs(matchedPoints[n].frenetPose.lateralDeviation);
    if(std::signbit(matchedPoints[n].desiredSpeed) != std::signbit(vehicleSpeed))
    {
      lateralDeviation+=1000;
    }

    if(lateralDeviation< minimalLateralDeviation)
    {
      bestIndex=n;
      minimalLateralDeviation=lateralDeviation;
    }
  }

  return bestIndex;
}

//-----------------------------------------------------------------------------
boost::optional<PathMatchedPoint2D> match(const PathSection2D & section,
                                          const Pose2D & vehiclePose,
                                          const double & vehicleSpeed,
                                          const double & time_horizon,
                                          const double & researchRadius)
{
  return match_impl(section,vehiclePose,vehicleSpeed,time_horizon,researchRadius);
}



//-----------------------------------------------------------------------------
boost::optional<PathMatchedPoint2D> match(const PathSection2D & section,
                                          const Pose2D & vehiclePose,
                                          const double & vehicleSpeed,
                                          const PathMatchedPoint2D & previousMatchedPoint,
                                          const double & expectedTravelledDistance,
                                          const double & time_horizon,
                                          const double & researchRadius)
{

  double s = previousMatchedPoint.frenetPose.curvilinearAbscissa;
  double mins = s - expectedTravelledDistance/2.;
  double maxs = s + expectedTravelledDistance/2.;

  return match(section,
               vehiclePose,
               vehicleSpeed,
               previousMatchedPoint.curveIndex,
               Interval<double>(mins,maxs),
               time_horizon,
               researchRadius);
}

//-----------------------------------------------------------------------------
boost::optional<PathMatchedPoint2D> match(const PathSection2D & section,
                                          const Pose2D & vehiclePose,
                                          const double & vehicleSpeed,
                                          const size_t & previousCurveIndex,
                                          const Interval<double> & curvilinearAbscissaInterval,
                                          const double &time_horizon,
                                          const double & researchRadius)
{

  Interval<size_t> rangeIndex = section.
      findIntervalBoundIndexes(previousCurveIndex,curvilinearAbscissaInterval);

  return match_impl(section,
                    vehiclePose,
                    vehicleSpeed,
                    time_horizon,
                    rangeIndex,
                    researchRadius);
}


//-----------------------------------------------------------------------------
boost::optional<PathMatchedPoint2D> match(const PathCurve2D & curve,
                                          const Pose2D & vehiclePose,
                                          const double & desiredSpeed)
{

  double nearestCurvilinearAbscissa;

  if(curve.findNearestCurvilinearAbscissa(vehiclePose.position,nearestCurvilinearAbscissa))
  {
    double xp = curve.computeX(nearestCurvilinearAbscissa);
    double yp = curve.computeY(nearestCurvilinearAbscissa);
    double tangent = curve.computeTangent(nearestCurvilinearAbscissa);
    double curvature = curve.computeCurvature(nearestCurvilinearAbscissa);

    const double & xv = vehiclePose.position.x();
    const double & yv = vehiclePose.position.y();
    const double & o = vehiclePose.yaw;
    const double & cost= std::cos(tangent);
    const double & sint = std::sin(tangent);

    double courseDeviation = betweenMinusPiAndPi(o-tangent);
    double lateralDeviation = (yv-yp)*cost-(xv-xp)*sint;

    double tangentOffset = desiredSpeed < 0 ? M_PI : 0;
    if(std::abs(betweenMinusPiAndPi(courseDeviation+tangentOffset))<M_PI_2)
    {
      Eigen::Matrix3d J=Eigen::Matrix3d::Identity();
      J.block<2,2>(0,0) = eulerAngleToRotation2D(courseDeviation);
      Eigen::Matrix3d frenetPoseCovariance = J*vehiclePose.covariance*J.transpose();

      //Singularity
      if((std::abs(curvature) > 10e-6)&&(std::abs(lateralDeviation-(1/curvature))<=10e-6))
      {
        curvature = 0;
      }

      PathMatchedPoint2D matchedPoint;
      matchedPoint.pathPosture.position.x()=xp;
      matchedPoint.pathPosture.position.y()=yp;
      matchedPoint.pathPosture.course=tangent;
      matchedPoint.pathPosture.curvature=curvature;
      matchedPoint.frenetPose.curvilinearAbscissa=nearestCurvilinearAbscissa;
      matchedPoint.frenetPose.lateralDeviation=lateralDeviation;
      matchedPoint.frenetPose.courseDeviation=courseDeviation;
      matchedPoint.frenetPose.covariance=frenetPoseCovariance;
      return matchedPoint;
    }
//    else
//    {
//      std::cout <<" cap rejection "<< std::endl;
//    }

  }

  return {};
}

}
