#include "romea_path/Path2D.hpp"
#include <iostream>

namespace {

}

namespace romea {


//-----------------------------------------------------------------------------
Path2D::Path2D():
  Path2D(0)
{

}

//-----------------------------------------------------------------------------
Path2D::Path2D(const double &interpolationWindowLength):
  X_(),
  Y_(),
  curvilinearAbscissa_(),
  curves_(),
  interpolationWindowLength_(interpolationWindowLength),
  isLoaded_(false)
{
}

//-----------------------------------------------------------------------------
void Path2D::setInterpolationWindowLength(const double & interpolationWindowLength)
{
  interpolationWindowLength_=interpolationWindowLength;
}

//-----------------------------------------------------------------------------
const double & Path2D::getInterpolationWindowLength()const
{
  return interpolationWindowLength_;
}

//-----------------------------------------------------------------------------
void Path2D::load(const VectorOfEigenVector2d &points)
{
  const auto numberOfPoints = points.size();
  assert(numberOfPoints>=2);

  X_.resize(numberOfPoints,0);
  Y_.resize(numberOfPoints,0);
  curvilinearAbscissa_.resize(numberOfPoints,0);

  X_[0]=points[0].x();
  Y_[0]=points[0].y();

  for(size_t n=1;n<numberOfPoints;++n)
  {
    X_[n]=points[n].x();
    Y_[n]=points[n].y();
    curvilinearAbscissa_[n] = curvilinearAbscissa_[n-1]+(points[n]-points[n-1]).norm();
  }

  curves_.resize(numberOfPoints);
  for(size_t n =0 ; n < numberOfPoints; ++n )
  {
    Interval<size_t> indexRange= findMinMaxIndexes(n,interpolationWindowLength_);
    assert(curves_[n].estimate(X_,Y_,curvilinearAbscissa_,indexRange));
  }

  isLoaded_=true;
}


//-----------------------------------------------------------------------------
size_t Path2D::findNearestIndex(const double & curvilinearAbscissa) const
{
  if(curvilinearAbscissa<=0)
  {
    return 0;
  }
  else if(curvilinearAbscissa>=curvilinearAbscissa_.back())
  {
    return  curvilinearAbscissa_.size()-1;
  }
  else
  {
    auto it = std::lower_bound(curvilinearAbscissa_.cbegin(),
                               curvilinearAbscissa_.cend(),
                               curvilinearAbscissa);

    return std::distance(curvilinearAbscissa_.cbegin(),it);
  }
}

//-----------------------------------------------------------------------------
Interval<size_t> Path2D::findMinMaxIndexes(const double & curvilinearAbscissa,
                                           const double & researchIntervalLength) const
{
  return findMinMaxIndexes(findNearestIndex(curvilinearAbscissa),
                           researchIntervalLength);
}

//-----------------------------------------------------------------------------
Interval<size_t> Path2D::findMinMaxIndexes(const size_t & pointIndex,
                                           const double & researchIntervalLength)const
{
  size_t minimalIndex = pointIndex;
  size_t maximalIndex = pointIndex;
  double pointCurvilinearAbscissa = curvilinearAbscissa_[pointIndex];

  while(minimalIndex !=0 &&
        pointCurvilinearAbscissa - curvilinearAbscissa_[minimalIndex] < researchIntervalLength/2.)
  {
    minimalIndex--;
  }

  while(maximalIndex != curvilinearAbscissa_.size()-1 &&
        curvilinearAbscissa_[maximalIndex] - pointCurvilinearAbscissa < researchIntervalLength/2.)
  {
    maximalIndex++;
  }

  return Interval<size_t>(minimalIndex,maximalIndex);
}


//-----------------------------------------------------------------------------
bool Path2D::isLoaded()const
{
  return isLoaded_;
}

//-----------------------------------------------------------------------------
const Path2D::Vector & Path2D::getX()const
{
  return X_;
}

//-----------------------------------------------------------------------------
const Path2D::Vector & Path2D::getY()const
{
  return Y_;
}

//-----------------------------------------------------------------------------
const Path2D::Vector & Path2D::getCurvilinearAbscissa()const
{
  return curvilinearAbscissa_;
}

//-----------------------------------------------------------------------------
double Path2D::getLength()const
{
  return curvilinearAbscissa_.back();
}

//-----------------------------------------------------------------------------
const std::vector<PathCurve2D> & Path2D::getCurves() const
{
  return curves_;
}


}
