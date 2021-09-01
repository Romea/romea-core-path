#include "romea_path/PathSection2D.hpp"
#include <iostream>
#include <algorithm>

namespace {

}

namespace romea {



//-----------------------------------------------------------------------------
PathSection2D::PathSection2D(const double &interpolationWindowLength,
                             const double & initialCurvilinearAbcissa):
  X_(),
  Y_(),
  curvilinearAbscissa_(initialCurvilinearAbcissa),
  curves_(),
  interpolationWindowLength_(interpolationWindowLength),
  length_(0)
{
}

//-----------------------------------------------------------------------------
void PathSection2D::addWayPoint(const PathWayPoint2D & wayPoint)
{
  X_.push_back(wayPoint.position.x());
  Y_.push_back(wayPoint.position.y());
  incrementCurvilinearAbscissa_();
  curves_.push_back(boost::optional<PathCurve2D>());
  speeds_.push_back(wayPoint.desired_speed);
}

//-----------------------------------------------------------------------------
void PathSection2D::addWayPoints(const std::vector<PathWayPoint2D> & wayPoints)
{
  reserve(size()+wayPoints.size());
  for(const auto & wayPoint : wayPoints)
  {
    addWayPoint(wayPoint);
  }
}

//-----------------------------------------------------------------------------
void PathSection2D::incrementCurvilinearAbscissa_()
{
  if(X_.size()!=curvilinearAbscissa_.size())
  {
    size_t n = curvilinearAbscissa_.size();
    double dx = X_[n]-X_[n-1];
    double dy = Y_[n]-Y_[n-1];
    double ds = std::sqrt(dx*dx+dy*dy);
    //    std::cout << ds <<" "<< dx <<" "<<dy << std::endl;
    curvilinearAbscissa_.increment(ds);
    length_+=ds;
  }
}

//-----------------------------------------------------------------------------
const PathCurve2D &PathSection2D::getCurve(const size_t & pointIndex)const
{
  if(!curves_[pointIndex].is_initialized())
  {
    computePathCurve_(pointIndex);
  }

  return *curves_[pointIndex];
}

//-----------------------------------------------------------------------------
void PathSection2D::computePathCurve_(const size_t & pointIndex) const
{
  curves_[pointIndex].emplace();
  std::cout <<"point index"<< pointIndex<<" "<<interpolationWindowLength_<< std::endl;
  Interval<double> curvilinearAbscissaInterval = computeCurvilinearAbscissaInterval_(pointIndex,interpolationWindowLength_);
  Interval<size_t> indexRange = findIntervalBoundIndexes(pointIndex,curvilinearAbscissaInterval);
  assert(curves_[pointIndex]->estimate(X_,Y_,curvilinearAbscissa_.data(),indexRange,curvilinearAbscissaInterval));
}

//-----------------------------------------------------------------------------
void PathSection2D::reserve(size_t n)
{
  X_.reserve(n);
  Y_.reserve(n);
  curvilinearAbscissa_.reserve(n);
  curves_.reserve(n);
  speeds_.reserve(n);
}

//-----------------------------------------------------------------------------
const PathSection2D::Vector & PathSection2D::getX()const
{
  return X_;
}

//-----------------------------------------------------------------------------
const PathSection2D::Vector & PathSection2D::getY()const
{
  return Y_;
}

//-----------------------------------------------------------------------------
const PathSection2D::CurvilinearAbscissa &PathSection2D::getCurvilinearAbscissa()const
{
  return curvilinearAbscissa_;
}

//-----------------------------------------------------------------------------
const PathSection2D::Vector & PathSection2D::getSpeeds() const
{
  return speeds_;
}


//-----------------------------------------------------------------------------
const double & PathSection2D::getLength()const
{
  return length_;
}

//-----------------------------------------------------------------------------
size_t PathSection2D::size()const
{
  return curves_.size();
}

//-----------------------------------------------------------------------------
void PathSection2D::clear()
{
  X_.clear();
  Y_.clear();
  curvilinearAbscissa_.clear();
  curves_.clear();
  length_=0;
}


//-----------------------------------------------------------------------------
size_t PathSection2D::findIndex(const double & value, const size_t & startSearchIndex) const
{
  size_t n=startSearchIndex;
  while(n<curvilinearAbscissa_.size()-1 && curvilinearAbscissa_[n]<value)
  {
    n++;
  }
  return n;
}

//-----------------------------------------------------------------------------
size_t PathSection2D::findIndex(const double & value) const
{
  return findIndex(value,0);
}


//-----------------------------------------------------------------------------
Interval<double> PathSection2D::computeCurvilinearAbscissaInterval_(const size_t & intervalCenterIndex,
                                                                    const double & intervalWidth)const
{
  return Interval<double>(curvilinearAbscissa_[intervalCenterIndex]-intervalWidth/2.,
                          curvilinearAbscissa_[intervalCenterIndex]+intervalWidth/2.);

}

//-----------------------------------------------------------------------------
Interval<size_t> PathSection2D::findIntervalBoundIndexes(const size_t & intervalCenterIndex,
                                                         const double & intervalWidth) const
{
  auto interval = computeCurvilinearAbscissaInterval_(intervalCenterIndex,intervalWidth);
  return findIntervalBoundIndexes(intervalCenterIndex,interval);
}

//-----------------------------------------------------------------------------
Interval<size_t> PathSection2D::findIntervalBoundIndexes(const size_t & intervalCenterIndex,
                                                         const Interval<double> & interval)const
{

  //    std::cout << " intervalCenterIndex " << 0 <<" "<< intervalCenterIndex << " "<< cumsum_.size()<<std::endl;
  //    std::cout << "is inside interval" << cumsum_[0] <<" "<<cumsum_[intervalCenterIndex]<<"  "<< cumsum_.back() <<std::endl;
  //      std::cout << "is inside interval" <<interval.lower() <<" "<<cumsum_[intervalCenterIndex]<<"  "<< interval.upper() <<std::endl;
  //assert(interval.inside(cumsum_[intervalCenterIndex]));

  size_t minimalIndex = intervalCenterIndex;
  size_t maximalIndex = intervalCenterIndex;

  while(minimalIndex !=0 &&
        interval.inside(curvilinearAbscissa_[minimalIndex]))
  {
    minimalIndex--;
  }

  while(maximalIndex != curvilinearAbscissa_.size()-1 &&
        interval.inside(curvilinearAbscissa_[maximalIndex]))
  {
    maximalIndex++;
  }


  return {minimalIndex,maximalIndex};
}

}
