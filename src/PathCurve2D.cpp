//romea
#include "romea_core_path/PathCurve2D.hpp"

//Eigen
#include <unsupported/Eigen/Polynomials>

//gsl
#include <gsl/gsl_poly.h>

//std
#include <iostream>

namespace {

inline bool computeSecondDegreePolynomialRegressionLight(const Eigen::ArrayXd &X,
                                                         const Eigen::ArrayXd &Y,
                                                         Eigen::Array3d & polynomCoefficient)
{
  double ide = 0;
  double MCx1 = 0, MCx2 = 0, MCx3 = 0, MCx4 = 0;
  double MCy1 = 0, MCxy = 0, MCx2y = 0;

  double Xoff = X[0];

  for (int i=0;i<X.size();i++)
  {
    double Xtemp = X[i] - Xoff;
    double Ytemp = Y[i];
    ide   = ide+1;
    MCx1  = MCx1 +Xtemp;
    MCx2  = MCx2 +Xtemp*Xtemp;
    MCx3  = MCx3 +Xtemp*Xtemp*Xtemp;
    MCx4  = MCx4 +Xtemp*Xtemp*Xtemp*Xtemp;
    MCy1  = MCy1 +Ytemp;
    MCxy  = MCxy +Xtemp*Ytemp;
    MCx2y = MCx2y+Xtemp*Xtemp*Ytemp;
  }

  Eigen::Matrix3d transform;
  transform(0,0) = ide; 	transform(0,1) = MCx1; 	transform(0,2) = MCx2;
  transform(1,0) = MCx1; 	transform(1,1) = MCx2; 	transform(1,2) = MCx3;
  transform(2,0) = MCx2; 	transform(2,1) = MCx3; 	transform(2,2) = MCx4;

  Eigen::Vector3d F;
  F(0) = MCy1;
  F(1) = MCxy;
  F(2) = MCx2y;

  bool success = true;
  Eigen::Matrix3d inverseTransform;
  transform.computeInverseWithCheck(inverseTransform, success);
  if(success == false)
  {
    std::cout<<"Matrix not intervertible"<<std::endl;
    return success;
  }

  polynomCoefficient = inverseTransform * F;
  polynomCoefficient(1) = polynomCoefficient(1) -2*polynomCoefficient(2)*Xoff;
  polynomCoefficient(0) = polynomCoefficient(0) -polynomCoefficient(1)*Xoff -polynomCoefficient(2)*Xoff*Xoff;
  return success;
}

}

namespace romea {

//-----------------------------------------------------------------------------
PathCurve2D::PathCurve2D():
  fxPolynomCoefficient_(Eigen::Array3d::Zero()),
  fyPolynomCoefficient_(Eigen::Array3d::Zero())
{

}

//-----------------------------------------------------------------------------
bool PathCurve2D::estimate(const Vector & X,
                           const Vector & Y,
                           const Vector & S,
                           const Interval<size_t> &indexInterval,
                           const Interval<double> & curvilinearAbscissaInterval)
{

  assert(indexInterval.width()>2);

  indexInterval_ = indexInterval;
  curvilinearAbscissaInterval_=curvilinearAbscissaInterval;

  Eigen::Map<const Eigen::ArrayXd> Xmap(X.data()+indexInterval.lower(),indexInterval.width()+1);
  Eigen::Map<const Eigen::ArrayXd> Ymap(Y.data()+indexInterval.lower(),indexInterval.width()+1);
  Eigen::Map<const Eigen::ArrayXd> Smap(S.data()+indexInterval.lower(),indexInterval.width()+1);

  return (computeSecondDegreePolynomialRegressionLight(Smap, Xmap, fxPolynomCoefficient_) &&
          computeSecondDegreePolynomialRegressionLight(Smap, Ymap, fyPolynomCoefficient_) );

}

//-----------------------------------------------------------------------------
bool PathCurve2D::findNearestCurvilinearAbscissa(const Eigen::Vector2d & vehiclePosition,
                                                 double & curvilinearAbscissa)const
{

#warning remettre le assert ailleurs
//  assert(curvilinearAbscissa >= minimalCurvilinearAbscissa_ &&
//         curvilinearAbscissa<=maximalCurvilinearAbscissa_);

  //  assert(maximalCurvilinearAbscissa >= 0);
  //  assert()
  //      //-----------------------------------------------------------
  //      // Recherche dMin(tracteur,courbe) : echantillonnage courbe
  //      //-----------------------------------------------------------
  //      if(maximalCurvilinearAbscissa < 2*active_window_)
  //  { // extrapolation possible
  //    // Depart recherche : X +/-2m (rayon cercle attaque)
  //    minimalCurvilinearAbscissa = maximalCurvilinearAbscissa - 2*active_window_;
  //  }

  double cx = fxPolynomCoefficient_[2], bx = fxPolynomCoefficient_[1], ax = fxPolynomCoefficient_[0];
  double cy = fyPolynomCoefficient_[2], by = fyPolynomCoefficient_[1], ay = fyPolynomCoefficient_[0];

  // d(r^2) / dt = d t^3 + c t^2 + b t + a = 0
  Eigen::Vector4d coeff;
  coeff[0] = 2*(cx*cx+cy*cy);
  coeff[1] = 3*(bx*cx+by*cy);
  coeff[2] = bx*bx+by*by+2*(ax*cx+ay*cy-cx*vehiclePosition.x()-cy*vehiclePosition.y());
  coeff[3] = ax*bx+ay*by-bx*vehiclePosition.x()-by*vehiclePosition.y();

  //  /// Using the Eigen library is very slow
  //    Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
  //    solver.compute(coeff);
  //    bool have_root = false;
  //    curvilinearAbscissa = solver.greatestRealRoot(have_root);

  //Using gsl
  double root_0, root_1=-100, root_2=-100;
  int nb_roots = gsl_poly_solve_cubic(coeff[1]/coeff[0],
      coeff[2]/coeff[0],
      coeff[3]/coeff[0],
      &root_0,
      &root_1,
      &root_2);


  //no solution
  if(nb_roots < 1)
  {
    std::cout <<"  no solution!!! "<< std::endl;
    return false;
  }

  //choice the nearest solution according previous curvilinear abscissa
  if(nb_roots==1)
  {
//    std::cout << " one root "<< root_0 << std::endl;
    curvilinearAbscissa=root_0;
  }
  else
  {
    double diff0 = std::abs(root_0-curvilinearAbscissa);
    double diff1 = std::abs(root_1-curvilinearAbscissa);
    double diff2 = std::abs(root_2-curvilinearAbscissa);

//    std::cout << diff0 << " "<< diff1 << " "<< diff2 << std::endl;
    if(diff1 < diff2)
    {
      if(diff1 < diff0)
        curvilinearAbscissa = root_1;
    }
    else
    {
      if(diff2 < diff0)
        curvilinearAbscissa = root_2;
    }
  }

  std::cout << " find curvilinearAbscissa "
            << curvilinearAbscissaInterval_.lower()<<" "
            <<curvilinearAbscissa <<" "
           << curvilinearAbscissaInterval_.upper()<<std::endl;


//  return curvilinearAbscissa>= minimalCurvilinearAbscissa_-5 && curvilinearAbscissa<=maximalCurvilinearAbscissa_+5;
  return curvilinearAbscissaInterval_.inside(curvilinearAbscissa);

}


//-----------------------------------------------------------------------------
double PathCurve2D::computeX(const double & curvilinearAbscissa)const
{
  return fxPolynomCoefficient_[2]*std::pow(curvilinearAbscissa,2)+
      fxPolynomCoefficient_[1]*curvilinearAbscissa+
      fxPolynomCoefficient_[0];
}

//-----------------------------------------------------------------------------
double PathCurve2D::computeY(const double & curvilinearAbscissa)const
{
  return fyPolynomCoefficient_[2]*std::pow(curvilinearAbscissa,2) +
      fyPolynomCoefficient_[1]*curvilinearAbscissa +
      fyPolynomCoefficient_[0];
}

//-----------------------------------------------------------------------------
double PathCurve2D::computeTangent(const double & curvilinearAbscissa)const
{
  return std::atan2((2*fyPolynomCoefficient_[2]*curvilinearAbscissa +fyPolynomCoefficient_[1]) ,
      (2*fxPolynomCoefficient_[2]*curvilinearAbscissa +fxPolynomCoefficient_[1]));

}

//-----------------------------------------------------------------------------
double PathCurve2D::computeCurvature(const double & curvilinearAbscissa)const
{
  double cy = fyPolynomCoefficient_[2];
  double by = fyPolynomCoefficient_[1];

  double cx = fxPolynomCoefficient_[2];
  double bx = fxPolynomCoefficient_[1];

  double Xdot         = bx+(2*cx* curvilinearAbscissa);
  double Ydot         = by+(2*cy* curvilinearAbscissa);
  double Xdotdot      = 2*cx;
  double Ydotdot      = 2*cy;
  double denominator = Xdot*Ydotdot-Ydot*Xdotdot;

  if(std::abs(denominator) <= std::numeric_limits<double>::epsilon())
  {
    return 0;
  }
  else
  {
    double tempo = sqrt(Xdot*Xdot+Ydot*Ydot);
    double radius = (tempo*tempo*tempo)/denominator;
    return 1/radius;
  }
}

//-----------------------------------------------------------------------------
const Interval<double> & PathCurve2D::getCurvilinearAbscissaInterval()const
{
  return curvilinearAbscissaInterval_;
}

//-----------------------------------------------------------------------------
const Interval<size_t> & PathCurve2D::getIndexInterval()const
{
  return indexInterval_;
}

}
