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


// Eigen
#include <unsupported/Eigen/Polynomials>

// std
#include <optional>
#include <algorithm>
#include <cassert>
#include <iostream>
#include <limits>


// gsl
#include "gsl/gsl_poly.h"

// romea
#include "romea_core_path/PathCurve2D.hpp"

namespace
{

inline bool computeSecondDegreePolynomialRegressionLight(
  const Eigen::ArrayXd & X,
  const Eigen::ArrayXd & Y,
  Eigen::Array3d & polynomCoefficient)
{
  double ide = 0;
  double MCx1 = 0, MCx2 = 0, MCx3 = 0, MCx4 = 0;
  double MCy1 = 0, MCxy = 0, MCx2y = 0;

  double Xoff = X[0];

  for (int i = 0; i < X.size(); i++) {
    double Xtemp = X[i] - Xoff;
    double Ytemp = Y[i];
    ide = ide + 1;
    MCx1 = MCx1 + Xtemp;
    MCx2 = MCx2 + Xtemp * Xtemp;
    MCx3 = MCx3 + Xtemp * Xtemp * Xtemp;
    MCx4 = MCx4 + Xtemp * Xtemp * Xtemp * Xtemp;
    MCy1 = MCy1 + Ytemp;
    MCxy = MCxy + Xtemp * Ytemp;
    MCx2y = MCx2y + Xtemp * Xtemp * Ytemp;
  }

  Eigen::Matrix3d transform;
  transform(0, 0) = ide; transform(0, 1) = MCx1; transform(0, 2) = MCx2;
  transform(1, 0) = MCx1; transform(1, 1) = MCx2; transform(1, 2) = MCx3;
  transform(2, 0) = MCx2; transform(2, 1) = MCx3; transform(2, 2) = MCx4;

  Eigen::Vector3d F;
  F(0) = MCy1;
  F(1) = MCxy;
  F(2) = MCx2y;

  bool success = true;
  Eigen::Matrix3d inverseTransform;
  transform.computeInverseWithCheck(inverseTransform, success);
  if (success == false) {
    std::cout << "Matrix not intervertible" << std::endl;
    return success;
  }

  polynomCoefficient = inverseTransform * F;
  polynomCoefficient(1) = polynomCoefficient(1) -
    2 * polynomCoefficient(2) * Xoff;
  polynomCoefficient(0) = polynomCoefficient(0) -
    polynomCoefficient(1) * Xoff -
    polynomCoefficient(2) * Xoff * Xoff;
  return success;
}

}  // namespace

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
PathCurve2D::PathCurve2D()
: fxPolynomCoefficient_(Eigen::Array3d::Zero()),
  fyPolynomCoefficient_(Eigen::Array3d::Zero()),
  indexInterval_(0, std::numeric_limits<size_t>::max()),
  curvilinearAbscissaInterval_()
{
}

//-----------------------------------------------------------------------------
bool PathCurve2D::estimate(
  const Vector & X,
  const Vector & Y,
  const Vector & S,
  const Interval<size_t> & indexInterval,
  const Interval<double> & curvilinearAbscissaInterval)
{
  assert(indexInterval.width() > 2);

  indexInterval_ = indexInterval;
  curvilinearAbscissaInterval_ = curvilinearAbscissaInterval;

  Eigen::Map<const Eigen::ArrayXd> Xmap(X.data() + indexInterval.lower(),
    indexInterval.width() + 1);
  Eigen::Map<const Eigen::ArrayXd> Ymap(Y.data() + indexInterval.lower(),
    indexInterval.width() + 1);
  Eigen::Map<const Eigen::ArrayXd> Smap(S.data() + indexInterval.lower(),
    indexInterval.width() + 1);

  return computeSecondDegreePolynomialRegressionLight(Smap, Xmap, fxPolynomCoefficient_) &&
         computeSecondDegreePolynomialRegressionLight(Smap, Ymap, fyPolynomCoefficient_);
}

//-----------------------------------------------------------------------------
std::optional<double> PathCurve2D::findNearestCurvilinearAbscissa(
  const Eigen::Vector2d & vehiclePosition)const
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

  double cx = fxPolynomCoefficient_[2];
  double bx = fxPolynomCoefficient_[1];
  double ax = fxPolynomCoefficient_[0];
  double cy = fyPolynomCoefficient_[2];
  double by = fyPolynomCoefficient_[1];
  double ay = fyPolynomCoefficient_[0];

  // d(r^2) / dt = d t^3 + c t^2 + b t + a = 0
  Eigen::Vector4d coeff;
  coeff[0] = 2 * (cx * cx + cy * cy);
  coeff[1] = 3 * (bx * cx + by * cy);
  coeff[2] = bx * bx + by * by + 2 *
    (ax * cx + ay * cy - cx * vehiclePosition.x() - cy * vehiclePosition.y());
  coeff[3] = ax * bx + ay * by - bx * vehiclePosition.x() - by * vehiclePosition.y();

  //  /// Using the Eigen library is very slow
  //    Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
  //    solver.compute(coeff);
  //    bool have_root = false;
  //    curvilinearAbscissa = solver.greatestRealRoot(have_root);

  if (std::abs(coeff[0]) < 1e-8) {
    double dx = vehiclePosition.x() - ax;
    double dy = vehiclePosition.y() - ay;
    double s = std::sqrt(dx * dx + dy * dy);

    if (curvilinearAbscissaInterval_.inside(s)) {
      return s;
    }
  } else {

    // Using gsl
    double root_0, root_1 = -100, root_2 = -100;
    int nb_roots = gsl_poly_solve_cubic(
      coeff[1] / coeff[0],
      coeff[2] / coeff[0],
      coeff[3] / coeff[0],
      &root_0,
      &root_1,
      &root_2);

    if (nb_roots == 1) {
      if (curvilinearAbscissaInterval_.inside(root_0)) {
        return root_0;
      }
    } else if (nb_roots == 3) {
      for (double root : {root_0, root_1, root_2}) {
        if (curvilinearAbscissaInterval_.inside(root)) {
          return root;
        }
      }
    }
  }

  return std::nullopt;
}


//-----------------------------------------------------------------------------
double PathCurve2D::computeX(const double & curvilinearAbscissa)const
{
  return fxPolynomCoefficient_[2] * std::pow(curvilinearAbscissa, 2) +
         fxPolynomCoefficient_[1] * curvilinearAbscissa +
         fxPolynomCoefficient_[0];
}

//-----------------------------------------------------------------------------
double PathCurve2D::computeY(const double & curvilinearAbscissa)const
{
  return fyPolynomCoefficient_[2] * std::pow(curvilinearAbscissa, 2) +
         fyPolynomCoefficient_[1] * curvilinearAbscissa +
         fyPolynomCoefficient_[0];
}

//-----------------------------------------------------------------------------
double PathCurve2D::computeTangent(const double & curvilinearAbscissa)const
{
  return std::atan2(
    (2 * fyPolynomCoefficient_[2] * curvilinearAbscissa + fyPolynomCoefficient_[1]),
    (2 * fxPolynomCoefficient_[2] * curvilinearAbscissa + fxPolynomCoefficient_[1]));
}

//-----------------------------------------------------------------------------
double PathCurve2D::computeCurvature(const double & curvilinearAbscissa)const
{
  double cy = fyPolynomCoefficient_[2];
  double by = fyPolynomCoefficient_[1];

  double cx = fxPolynomCoefficient_[2];
  double bx = fxPolynomCoefficient_[1];

  double Xdot = bx + (2 * cx * curvilinearAbscissa);
  double Ydot = by + (2 * cy * curvilinearAbscissa);
  double Xdotdot = 2 * cx;
  double Ydotdot = 2 * cy;
  double denominator = Xdot * Ydotdot - Ydot * Xdotdot;

  if (std::abs(denominator) <= std::numeric_limits<double>::epsilon()) {
    return 0;
  } else {
    double tempo = sqrt(Xdot * Xdot + Ydot * Ydot);
    double radius = (tempo * tempo * tempo) / denominator;
    return 1 / radius;
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

}  // namespace core
}  // namespace romea
