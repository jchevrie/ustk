/****************************************************************************
 *
 * This file is part of the ustk software.
 * Copyright (C) 2016 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ustk with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at ustk@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Authors:
 * Jason Chevrie
 *
 *****************************************************************************/

#include <visp3/ustk_needle_detection/usPolynomialCurve3D.h>

#include <algorithm>

#include <visp3/core/vpException.h>


usPolynomialCurve3D::usPolynomialCurve3D():
    m_order(0),
    m_startParameter(0),
    m_endParameter(1),
    m_polynomialCoefficients(3,1,0)
{
  
}

usPolynomialCurve3D::usPolynomialCurve3D(const usPolynomialCurve3D &curve):
    m_order(curve.m_order),
    m_startParameter(curve.m_startParameter),
    m_endParameter(curve.m_endParameter),
    m_polynomialCoefficients(curve.m_polynomialCoefficients)
{

}

const usPolynomialCurve3D &usPolynomialCurve3D::operator=(const usPolynomialCurve3D &curve)
{
    m_order = curve.m_order;
    m_startParameter = curve.m_startParameter;
    m_endParameter = curve.m_endParameter;
    m_polynomialCoefficients = curve.m_polynomialCoefficients;
    
    return *this;
}

usPolynomialCurve3D::~usPolynomialCurve3D()
{
    
}

usPolynomialCurve3D::usPolynomialCurve3D(unsigned int order):
    m_order(order),
    m_startParameter(0),
    m_endParameter(1),
    m_polynomialCoefficients(3, m_order+1, 0)
{

}

void usPolynomialCurve3D::setOrder(unsigned int order)
{  
  if(order==m_order) return;
  else if(order>m_order)
  {
      // Keep same polynomial
      m_order = order;
      m_polynomialCoefficients.resize(3,order+1,false);
  }
  else
  {
      // Find polynomial to have the same properties at the extremities (position, direction, curvature, ...)
      int nb_coef = order+1;
      int nb_constraints_begin = nb_coef/2 + nb_coef%2;
      int nb_constraints_end = nb_coef/2;

      vpColVector startParameter_power(m_order+1,0);
      vpColVector endParameter_power(m_order+1,0);
      startParameter_power[0] = 1;
      endParameter_power[0] = 1;
      for(unsigned int i=1 ; i<=m_order ; i++)
      {
          startParameter_power[i] = m_startParameter*startParameter_power[i-1];
          endParameter_power[i] = m_endParameter*endParameter_power[i-1];
      }

      vpMatrix A(nb_coef,nb_coef,0);
      for(int i=0 ; i<nb_constraints_begin ; i++)
      {
          for(int j=i ; j<nb_coef ; j++)
          {
              A[i][j] = startParameter_power[j-i];
              for(int n=0 ; n<i ; n++) A[i][j] *= j-n;
          }
      }
      for(int i=0 ; i<nb_constraints_end ; i++)
      {
          for(int j=0 ; j<nb_coef ; j++)
          {
              A[nb_constraints_begin+i][j] = endParameter_power[j-i];
              for(int n=0 ; n<i ; n++) A[i][j] *= j-n;
          }
      }

      vpMatrix Ainvt = A.inverseByLU().t();

      vpMatrix B(3,nb_coef);
      for(int i=0 ; i<nb_constraints_begin ; i++)
      {
          vpColVector col(3,0);
          for(unsigned int j=nb_constraints_begin ; j<=m_order ; j++)
          {
              double coef = startParameter_power[j-i];
              for(int n=0 ; n<i ; n++) coef *= j-n;
              col += coef*m_polynomialCoefficients.getCol(j);
          }
          B.insert(col,0,i);
      }
      for(int i=0 ; i<nb_constraints_end ; i++)
      {
          vpColVector col(3,0);
          for(unsigned int j=nb_constraints_begin ; j<=m_order ; j++)
          {
              double coef = endParameter_power[j-i];
              for(int n=0 ; n<i ; n++) coef *= j-n;
              col += coef*m_polynomialCoefficients.getCol(j);
          }
          B.insert(col,0,nb_constraints_begin+i);
      }

      m_polynomialCoefficients.resize(3,nb_coef);
      m_polynomialCoefficients = B * Ainvt;
      m_order = order;
  } 
}

unsigned int usPolynomialCurve3D::getOrder() const
{
    return m_order;
}

void usPolynomialCurve3D::setStartParameter(double startParameter)
{
    m_startParameter = startParameter;
}

double usPolynomialCurve3D::getStartParameter() const
{
    return m_startParameter;
}

void usPolynomialCurve3D::setEndParameter(double endParameter)
{
    m_endParameter = endParameter;
}

double usPolynomialCurve3D::getEndParameter() const
{
    return m_endParameter;
}

void usPolynomialCurve3D::setBoundaries(double startParameter, double endParameter)
{
    if(startParameter<=endParameter)
    {
        m_startParameter = startParameter;
        m_endParameter = endParameter;
    }
    else
    {
        m_startParameter = endParameter;
        m_endParameter = startParameter;
        this->reverse();
    }
}

void usPolynomialCurve3D::setParametricLength(double length)
{
    m_endParameter = m_startParameter+length;
}

double usPolynomialCurve3D::getParametricLength() const
{
    return m_endParameter - m_startParameter;
}

void usPolynomialCurve3D::setLength(double length, double precision)
{
    if(precision<=0) throw vpException(vpException::badValue, "usPolynomialCurve3D::setLength: precision should be strictly positive");
    if(length<=0) throw vpException(vpException::badValue, "usPolynomialCurve3D::setLength: length should be strictly positive");

    double lMetric = this->getLength();
    double lParam = this->getParametricLength();
    double lMetricPrev = 0;
    double lParamPrev = 0;
    double diffMetric = length-lMetric;
    
    while(fabs(diffMetric) > length*precision)
    {
        double slope = (lParam-lParamPrev) / (lMetric-lMetricPrev);
        m_endParameter += diffMetric * slope;
        lMetricPrev = lMetric;
        lParamPrev = lParam;
        lMetric = this->getLength();
        lParam = this->getParametricLength();
        diffMetric = length-lMetric;
    }
}

double usPolynomialCurve3D::getLength(int nbCountSeg) const
{
    vpColVector params(nbCountSeg+1);
    double step = this->getParametricLength() / nbCountSeg;
    params[0] = m_startParameter;
    for(int i=1 ; i<nbCountSeg+1 ; i++) params[i] = params[i-1] + step;

    vpMatrix points = this->getPoints(params);
    
    double length = 0.0;
    for(int i=0 ; i<nbCountSeg ; i++) length += (points.getCol(i) - points.getCol(i+1)).euclideanNorm();
    return length;
}

void usPolynomialCurve3D::setPolynomialCoefficients(const vpMatrix &polynomialCoefficients)
{
    if(polynomialCoefficients.getCols()<2) throw vpException(vpException::dimensionError, "usPolynomialCurve3D::setPolynomialCoefficients: empty coefficient matrix");
    if(polynomialCoefficients.getRows()!=3) throw vpException(vpException::dimensionError, "usPolynomialCurve3D::setPolynomialCoefficients: space dimension should be 3");
    
    m_order = polynomialCoefficients.getCols()-1;
    m_polynomialCoefficients = polynomialCoefficients;
  
   /*vpMatrix coords(m_order, m_order);
  for(unsigned int j = 0; j < m_order; ++j) {
    coords[0][j] = 1.0;
    double t = static_cast<double>(j) / static_cast<double>(m_order - 1);
    for (unsigned int i = 1; i < m_order; ++i)
      coords[i][j] = coords[i - 1][j] * t;
  }
  m_controlPoints = m_polynomialCoefficients * coords;
  coords = vpMatrix(m_order, m_nRenderingLines + 1);
  for (unsigned int j = 0; j < m_nRenderingLines + 1; ++j) {
    coords[0][j] = 1.0;
    double t = static_cast<double>(j) / static_cast<double>(m_nRenderingLines);
    for (unsigned int i = 1; i < m_order; ++i)
      coords[i][j] = coords[i - 1][j] * t;
  }
  m_renderingPoints = m_polynomialCoefficients * coords;*/
}

vpMatrix usPolynomialCurve3D::getPolynomialCoefficients() const
{ 
    return m_polynomialCoefficients;
}

vpColVector usPolynomialCurve3D::getPoint(double parameter) const
{
  vpColVector T(m_order+1);
  T[0] = 1.0;
  for(unsigned int i=1 ; i<m_order+1 ; i++) T[i] = T[i-1] * parameter;
  return m_polynomialCoefficients * T;
}

vpMatrix usPolynomialCurve3D::getPoints(vpColVector parameters) const
{
  vpMatrix T(m_order+1, parameters.size());
  for(unsigned int j=0 ; j<parameters.size() ; j++)
  {
        T[0][j] = 1.0;
        for(unsigned int i=1 ; i<m_order+1 ; i++) T[i][j] = T[i-1][j] * parameters[j];
  }
  return m_polynomialCoefficients * T;
}

vpColVector usPolynomialCurve3D::getStartPoint() const
{
    return this->getPoint(m_startParameter);
}

vpColVector usPolynomialCurve3D::getEndPoint() const
{
    return this->getPoint(m_endParameter);
}

vpColVector usPolynomialCurve3D::getTangent(double parameter) const
{
    vpColVector T(m_order+1);
    double tt = 1.0;
    T[0] = 0.0;
    for(unsigned int i=1 ; i<m_order+1 ; i++)
    {
        T[i] = i * tt;
        tt *= parameter;
    }
  return (m_polynomialCoefficients * T).normalize();
}

vpColVector usPolynomialCurve3D::getStartTangent() const
{
    return this->getTangent(m_startParameter);
}

vpColVector usPolynomialCurve3D::getEndTangent() const
{
    return this->getTangent(m_endParameter);
}

vpColVector usPolynomialCurve3D::getDerivative(double parameter, unsigned int order) const
{
    vpColVector P(3,0);

    if(order>0 && order<=m_order)
    {
        vpColVector factor_coef(m_order+1,0);
        factor_coef[order] = 1;
        for(unsigned int i=order+1 ; i<=m_order ; i++) factor_coef[i] = parameter * factor_coef[i-1];

        for(unsigned int i=order ; i<=m_order ; i++)
        {
            for(unsigned int n=0 ; n<order ; n++) factor_coef[i] *= i-n;
        }

        P = m_polynomialCoefficients * factor_coef;
    }

    return P;
}

void usPolynomialCurve3D::defineFromPoints(const std::vector<vpColVector> &points, const std::vector<double> &param, unsigned int order)
{
    unsigned int nbPoints = points.size();
    if(nbPoints < 2) throw vpException(vpException::dimensionError, "usPolynomialCurve3D::defineFromPoints(const std::vector<vpColVector> &points, const std::vector<double> &param, unsigned int order): need at least two points to fit");    
    if(param.size() != nbPoints) throw vpException(vpException::dimensionError, "usPolynomialCurve3D::defineFromPoints(const std::vector<vpColVector> &points, const std::vector<double> &param, unsigned int order): mismatching number of points and parametric values");
    
    if(order<1) order = m_order;

    unsigned int nbCoef = order+1;

    vpMatrix M(nbCoef, nbCoef, 0);
    vpMatrix B(nbCoef, 3, 0);

    unsigned int nb_pow = 2*nbCoef-1;
    vpMatrix t_pow(nbPoints,nb_pow);
    for(unsigned int i=0 ; i<nbPoints ; i++)
    {
        double ti = param.at(i);
        t_pow[i][0] = 1;
        for(unsigned int j=1 ; j<nb_pow ; j++)
        {
            t_pow[i][j] = ti*t_pow[i][j-1];
        }
    }

    for(unsigned int line=0 ; line<nbCoef ; line++)
    {
        for(unsigned int i=0 ; i<nbPoints ; i++)
        {
            for(unsigned int j=0 ; j<nbCoef ; j++)
            {
                M[line][j] += t_pow[i][line+j];
            }
            for(unsigned int dim=0 ; dim<3 ; dim++)
            {
                B[line][dim] += t_pow[i][line] * points.at(i)[dim];
            }
        }
    }

    vpMatrix A;
    try
    {
        if(nbCoef > nbPoints) A = M.pseudoInverse(std::numeric_limits<double>::epsilon()) * B;
        else A = M.inverseByCholeskyLapack() * B;
    }
    catch(std::exception &e)
    {
        throw vpException(vpException::fatalError, "usPolynomialCurve3D::defineFromPoints(const std::vector<vpColVector> &points, const std::vector<double> &param, unsigned int order): %s", e.what());
    }

    this->setPolynomialCoefficients(A.t());
    this->setBoundaries(param.front(), param.back());
}
  
void usPolynomialCurve3D::defineFromPoints(const vpMatrix points, const vpColVector &param, unsigned int order)
{
    unsigned int nbPoints = points.getCols();
    if(nbPoints < 2) throw vpException(vpException::dimensionError, "usPolynomialCurve3D::defineFromPoints(const std::vector<vpColVector> &points, const std::vector<double> &param, unsigned int order): need at least two points to fit");    
    if(param.size() != nbPoints) throw vpException(vpException::dimensionError, "usPolynomialCurve3D::defineFromPoints(const std::vector<vpColVector> &points, const std::vector<double> &param, unsigned int order): mismatching number of points and parametric values");
    
    if(order<1) order = m_order;

    unsigned int nbCoef = order+1;

    vpMatrix M(nbCoef, nbCoef, 0);
    vpMatrix B(nbCoef, 3, 0);

    unsigned int nb_pow = 2*nbCoef-1;
    vpMatrix t_pow(nbPoints,nb_pow);
    for(unsigned int i=0 ; i<nbPoints ; i++)
    {
        double ti = param[i];
        t_pow[i][0] = 1;
        for(unsigned int j=1 ; j<nb_pow ; j++)
        {
            t_pow[i][j] = ti*t_pow[i][j-1];
        }
    }

    for(unsigned int line=0 ; line<nbCoef ; line++)
    {
        for(unsigned int i=0 ; i<nbPoints ; i++)
        {
            for(unsigned int j=0 ; j<nbCoef ; j++)
            {
                M[line][j] += t_pow[i][line+j];
            }
            for(unsigned int dim=0 ; dim<3 ; dim++)
            {
                B[line][dim] += t_pow[i][line] * points[dim][i];
            }
        }
    }

    vpMatrix A;
    try
    {
        if(nbCoef > nbPoints) A = M.pseudoInverse(std::numeric_limits<double>::epsilon()) * B;
        else A = M.inverseByCholeskyLapack() * B;
    }
    catch(std::exception &e)
    {
        throw vpException(vpException::fatalError, "usPolynomialCurve3D::defineFromPoints(const std::vector<vpColVector> &points, const std::vector<double> &param, unsigned int order): %s", e.what());
    }

    this->setPolynomialCoefficients(A.t());
    this->setBoundaries(param[0], param[param.size()-1]);
}
  
void usPolynomialCurve3D::defineFromPointsAuto(const std::vector<vpColVector> &points, unsigned int order)
{
    unsigned int nbPoints = points.size();
    if(nbPoints < 2) throw vpException(vpException::dimensionError, "usPolynomialCurve3D::defineFromPointsAuto(const std::vector<vpColVector> &points, unsigned int order): need at least two points to fit");
 
    if(order<1) order = m_order;

    int max_i = 0;
    int max_j = 0;
    double max = 0;
    for(unsigned int i=0 ; i<nbPoints ; i++)
    {
        for(unsigned int j=0 ; j<i ; j++)
        {
            double v = (points.at(i)-points.at(j)).sumSquare();
            if(v > max)
            {
                max = v;
                max_i = i;
                max_j = j;
            }
        }
    }
    vpColVector dir((points.at(max_i)-points.at(max_j)).normalize());

    return this->defineFromPointsAuto(points, dir, order);
}

void usPolynomialCurve3D::defineFromPointsAuto(const vpMatrix &points, unsigned int order)
{
    unsigned int nbPoints = points.getCols();
    if(nbPoints < 2) throw vpException(vpException::dimensionError, "usPolynomialCurve3D::defineFromPointsAuto(const vpMatrix &points, unsigned int order): need at least two points to fit");
 
    if(order<1) order = m_order;

    int max_i = 0;
    int max_j = 0;
    double max = 0;
    for(unsigned int i=0 ; i<nbPoints ; i++)
    {
        for(unsigned int j=0 ; j<i ; j++)
        {
            double v = (points.getCol(i)-points.getCol(j)).sumSquare();
            if(v > max)
            {
                max = v;
                max_i = i;
                max_j = j;
            }
        }
    }
    vpColVector dir((points.getCol(max_i)-points.getCol(max_j)).normalize());

    return this->defineFromPointsAuto(points, dir, order);
}

void usPolynomialCurve3D::defineFromPointsAuto(const std::vector<vpColVector> &points, const vpColVector &direction, unsigned int order)
{
    unsigned int nbPoints = points.size();
    if(nbPoints < 2) throw vpException(vpException::dimensionError, "usPolynomialCurve3D::defineFromPointsAuto(const std::vector<vpColVector> &points, const vpColVector &direction, unsigned int order): need at least two points to fit");
    
    if(order<1) order = m_order;

    std::vector<double> t(nbPoints, 0);
    
    double min = std::numeric_limits<double>::max();
    for(unsigned int i=0 ; i<nbPoints; i++)
    {
        t.at(i) = vpColVector::dotProd(points.at(i), direction);
        if(t.at(i) < min) min = t.at(i);
    }
    for(unsigned int i=0 ; i<nbPoints; i++) t.at(i) -= min;
    
    std::vector<int> index(nbPoints);
    for(unsigned int i=0 ; i<nbPoints; i++) index.at(i) = i;
    
    std::sort(index.begin(), index.end(), [&t](int i, int j){return t[i] < t[j];});
    
    std::vector<vpColVector> newPoints(nbPoints);
    std::vector<double> param(nbPoints);
    for(unsigned int i=0 ; i<nbPoints; i++)
    {
        newPoints.at(i) = points.at(index[i]);
        param.at(i) = t.at(index[i]);
    }

    return this->defineFromPoints(newPoints, param, order);
}

void usPolynomialCurve3D::defineFromPointsAuto(const vpMatrix &points, const vpColVector &direction, unsigned int order)
{
    unsigned int nbPoints = points.getCols();
    if(nbPoints < 2) throw vpException(vpException::dimensionError, "usPolynomialCurve3D::defineFromPointsAuto(const vpMatrix &points, const vpColVector &direction, unsigned int order): need at least two points to fit");
    
    if(order<1) order = m_order;

    std::vector<double> t(nbPoints, 0);
    
    double min = std::numeric_limits<double>::max();
    for(unsigned int i=0 ; i<nbPoints; i++)
    {
        t.at(i) = vpColVector::dotProd(points.getCol(i), direction);
        if(t.at(i) < min) min = t.at(i);
    }
    for(unsigned int i=0 ; i<nbPoints; i++) t.at(i) -= min;
    
    std::vector<int> index(nbPoints);
    for(unsigned int i=0 ; i<nbPoints; i++) index.at(i) = i;
    
    std::sort(index.begin(), index.end(), [&t](int i, int j){return t[i] < t[j];});
    
    vpMatrix newPoints(3, nbPoints);
    vpColVector param(nbPoints);
    for(unsigned int i=0 ; i<nbPoints; i++)
    {
        for(unsigned int j=0 ; j<3; j++) newPoints[j][i] = points[j][index[i]];
        param[i] = t.at(index[i]);
    }

    return this->defineFromPoints(newPoints, param, order);
}

void usPolynomialCurve3D::defineFromWeightedPoints(const std::vector<vpColVector> &points, const std::vector<double> &param, const std::vector<double> &weights, unsigned int order)
{
    unsigned int nbPoints = points.size();
    if(nbPoints < 2) throw vpException(vpException::dimensionError, "usPolynomialCurve3D::defineFromWeightedPoints(const std::vector<vpColVector> &points, const std::vector<double> &param, const std::vector<double> &weights, unsigned int order): need at least two points to fit");
    if(param.size() != nbPoints) throw vpException(vpException::dimensionError, "usPolynomialCurve3D::defineFromWeightedPoints(const std::vector<vpColVector> &points, const std::vector<double> &param, const std::vector<double> &weights, unsigned int order): mismatching number of points and parametric values or weights");
    if(weights.size() != nbPoints) throw vpException(vpException::dimensionError, "usPolynomialCurve3D::defineFromWeightedPoints(const std::vector<vpColVector> &points, const std::vector<double> &param, const std::vector<double> &weights, unsigned int order): mismatching number of points and weights");
    
    if(order<1) order = m_order;

    unsigned int nbCoef = order+1;

    vpMatrix M(nbCoef, nbCoef, 0);
    vpMatrix B(nbCoef, 3, 0);

    unsigned int nb_pow = 2*nbCoef-1;
    vpMatrix t_pow(nbPoints,nb_pow);
    for(unsigned int i=0 ; i<nbPoints ; i++)
    {
        double ti = param.at(i);
        t_pow[i][0] = weights.at(i);
        for(unsigned int j=1 ; j<nb_pow ; j++)
        {
            t_pow[i][j] = ti*t_pow[i][j-1];
        }
    }

    for(unsigned int line=0 ; line<nbCoef ; line++)
    {
        for(unsigned int i=0 ; i<nbPoints ; i++)
        {
            for(unsigned int j=0 ; j<nbCoef ; j++)
            {
                M[line][j] += t_pow[i][line+j];
            }
            for(unsigned int dim=0 ; dim<3 ; dim++)
            {
                B[line][dim] += t_pow[i][line] * points.at(i)[dim];
            }
        }
    }

    vpMatrix A;
    try
    {
        A = M.pseudoInverse(std::numeric_limits<double>::epsilon()) * B;
    }
    catch(std::exception &e)
    {
        throw vpException(vpException::fatalError, "usPolynomialCurve3D::defineFromWeightedPoints(const vpMatrix &points, const vpColVector &param, const vpColVector &weights, unsigned int order): %s", e.what());
    }

    this->setPolynomialCoefficients(A.t());
    this->setBoundaries(param.front(), param.back());
}

void usPolynomialCurve3D::defineFromWeightedPoints(const vpMatrix &points, const vpColVector &param, const vpColVector &weights, unsigned int order)
{
    unsigned int nbPoints = points.getCols();
    if(nbPoints < 2) throw vpException(vpException::dimensionError, "usPolynomialCurve3D::defineFromWeightedPoints(const vpMatrix &points, const vpColVector &param, const vpColVector &weights, unsigned int order): need at least two points to fit");
    if(param.size() != nbPoints) throw vpException(vpException::dimensionError, "usPolynomialCurve3D::defineFromWeightedPoints(const vpMatrix &points, const vpColVector &param, const vpColVector &weights, unsigned int order): mismatching number of points and parametric values or weights");
    if(weights.size() != nbPoints) throw vpException(vpException::dimensionError, "usPolynomialCurve3D::defineFromWeightedPoints(const vpMatrix &points, const vpColVector &param, const vpColVector &weights, unsigned int order): mismatching number of points and weights");
    
    if(order<1) order = m_order;

    unsigned int nbCoef = order+1;

    vpMatrix M(nbCoef, nbCoef, 0);
    vpMatrix B(nbCoef, 3, 0);

    unsigned int nb_pow = 2*nbCoef-1;
    vpMatrix t_pow(nbPoints,nb_pow);
    for(unsigned int i=0 ; i<nbPoints ; i++)
    {
        double ti = param[i];
        t_pow[i][0] = weights[i];
        for(unsigned int j=1 ; j<nb_pow ; j++)
        {
            t_pow[i][j] = ti*t_pow[i][j-1];
        }
    }

    for(unsigned int line=0 ; line<nbCoef ; line++)
    {
        for(unsigned int i=0 ; i<nbPoints ; i++)
        {
            for(unsigned int j=0 ; j<nbCoef ; j++)
            {
                M[line][j] += t_pow[i][line+j];
            }
            for(unsigned int dim=0 ; dim<3 ; dim++)
            {
                B[line][dim] += t_pow[i][line] * points[dim][i];
            }
        }
    }

    vpMatrix A;
    try
    {
        A = M.pseudoInverse(std::numeric_limits<double>::epsilon()) * B;
    }
    catch(std::exception &e)
    {
        throw vpException(vpException::fatalError, "usPolynomialCurve3D::defineFromWeightedPoints(const vpMatrix &points, const vpColVector &param, const vpColVector &weights, unsigned int order): %s", e.what());
    }

    this->setPolynomialCoefficients(A.t());
    this->setBoundaries(param[0], param[nbPoints-1]);
}

void usPolynomialCurve3D::defineFromWeightedPointsAuto(const std::vector<vpColVector> &points, const std::vector<double> &weights, unsigned int order)
{
    unsigned int nbPoints = points.size();
    if(nbPoints < 2) throw vpException(vpException::dimensionError, "usPolynomialCurve3D::defineFromWeightedPointsAuto(const std::vector<vpColVector> &points, const std::vector<double> &weights, unsigned int order): need at least two points to fit");
    if(weights.size() != nbPoints) throw vpException(vpException::dimensionError, "usPolynomialCurve3D::defineFromWeightedPointsAuto(const std::vector<vpColVector> &points, const std::vector<double> &weights, unsigned int order): mismatching number of points and weights");

    if(order<1) order = m_order;

    std::vector<double> t(nbPoints, 0);
        
    vpColVector Pmean(3,0);
    for(unsigned int i=0 ; i<nbPoints ; i++) Pmean += points.at(i);
    Pmean /= nbPoints;
    
    vpMatrix M(nbPoints,3);
    for(unsigned int i=0 ; i<nbPoints; i++) for(int j=0 ; j<3; j++) M[i][j] = points.at(i)[j]-Pmean[j];

    // Reduction to one principal component using singular value decomposition
    vpColVector w;
    vpMatrix V;
    M.svd(w, V);

    vpColVector dir(V.getCol(0));
    double min = std::numeric_limits<double>::max();
    for(unsigned int i=0 ; i<nbPoints; i++)
    {
        t.at(i) = vpColVector::dotProd(points.at(i), dir);
        if(t.at(i) < min) min = t.at(i);
    }
    for(unsigned int i=0 ; i<nbPoints; i++) t.at(i) -= min;

    std::vector<int> index(nbPoints);
    for(unsigned int i=0 ; i<nbPoints; i++) index.at(i) = i;
    
    std::sort(index.begin(), index.end(), [&t](int i, int j){return t[i] < t[j];});
    
    std::vector<vpColVector> newPoints(nbPoints);
    std::vector<double> newWeights(nbPoints);
    std::vector<double> param(nbPoints);
    for(unsigned int i=0 ; i<nbPoints; i++)
    {
        newPoints.at(i) = points.at(index[i]);
        param.at(i) = t.at(index[i]);
        newWeights.at(i) = weights.at(index[i]);
    }

    this->defineFromWeightedPoints(newPoints, param, newWeights, order);
}

void usPolynomialCurve3D::defineFromWeightedPointsAuto(const vpMatrix &points, const vpColVector &weights, unsigned int order)
{
    unsigned int nbPoints = points.getCols();
    if(nbPoints < 2) throw vpException(vpException::dimensionError, "usPolynomialCurve3D::defineFromWeightedPointsAuto(const vpMatrix &points, const vpColVector &weights, unsigned int order): need at least two points to fit");
    if(weights.size() != nbPoints) throw vpException(vpException::dimensionError, "usPolynomialCurve3D::defineFromWeightedPointsAuto(const vpMatrix &points, const vpColVector &weights, unsigned int order): mismatching number of points and weights");

    if(order<1) order = m_order;

    std::vector<double> t(nbPoints, 0);
        
    vpColVector Pmean(3,0);
    for(unsigned int i=0 ; i<nbPoints ; i++) Pmean += points.getCol(i);
    Pmean /= nbPoints;
    
    vpMatrix M(nbPoints,3);
    for(unsigned int i=0 ; i<nbPoints; i++) for(int j=0 ; j<3; j++) M[i][j] = points[j][i]-Pmean[j];

    // Reduction to one principal component using singular value decomposition
    vpColVector w;
    vpMatrix V;
    M.svd(w, V);

    vpColVector dir(V.getCol(0));
    double min = std::numeric_limits<double>::max();
    for(unsigned int i=0 ; i<nbPoints; i++)
    {
        t.at(i) = vpColVector::dotProd(points.getCol(i), dir);
        if(t.at(i) < min) min = t.at(i);
    }
    for(unsigned int i=0 ; i<nbPoints; i++) t.at(i) -= min;

    std::vector<int> index(nbPoints);
    for(unsigned int i=0 ; i<nbPoints; i++) index.at(i) = i;
    
    std::sort(index.begin(), index.end(), [&t](int i, int j){return t[i] < t[j];});
    
    vpMatrix newPoints(3, nbPoints);
    vpColVector newWeights(nbPoints);
    vpColVector param(nbPoints);
    for(unsigned int i=0 ; i<nbPoints; i++)
    {
        for(unsigned int j=0 ; j<3; j++) newPoints[j][i] = points[j][index[i]];
        param[i] = t.at(index[i]);
        newWeights[i] = weights[index[i]];
    }

    this->defineFromWeightedPoints(newPoints, param, newWeights, order);
}

void usPolynomialCurve3D::defineFromWeightedPointsAuto(const std::vector<vpColVector> &points, const std::vector<double> &weights, const vpColVector &direction, unsigned int order)
{
    unsigned int nbPoints = points.size();
    if(nbPoints < 2) throw vpException(vpException::dimensionError, "usPolynomialCurve3D::defineFromWeightedPointsAuto(const std::vector<vpColVector> &points, const std::vector<double> &weights, const vpColVector &direction, unsigned int order): need at least two points to fit");
    if(weights.size() != nbPoints) throw vpException(vpException::dimensionError, "usPolynomialCurve3D::defineFromWeightedPointsAuto(const std::vector<vpColVector> &points, const std::vector<double> &weights, const vpColVector &direction, unsigned int order): mismatching number of points and weights");
    
    if(order<1) order = m_order;

    std::vector<double> t(nbPoints, 0);
    
    double min = std::numeric_limits<double>::max();
    for(unsigned int i=0 ; i<nbPoints; i++)
    {
        t.at(i) = vpColVector::dotProd(points.at(i), direction);
        if(t.at(i) < min) min = t.at(i);
    }
    for(unsigned int i=0 ; i<nbPoints; i++) t.at(i) -= min;
    
    std::vector<int> index(nbPoints);
    for(unsigned int i=0 ; i<nbPoints; i++) index.at(i) = i;
    
    std::sort(index.begin(), index.end(), [&t](int i, int j){return t[i] < t[j];});
    
    std::vector<vpColVector> newPoints(nbPoints);
    std::vector<double> newWeights(nbPoints);
    std::vector<double> param(nbPoints);
    for(unsigned int i=0 ; i<nbPoints; i++)
    {
        newPoints.at(i) = points.at(index[i]);
        param.at(i) = t.at(index[i]);
        newWeights.at(i) = weights.at(index[i]);
    }

    return this->defineFromWeightedPoints(newPoints, param, newWeights, order);
}

void usPolynomialCurve3D::defineFromWeightedPointsAuto(const vpMatrix &points, const vpColVector &weights, const vpColVector &direction, unsigned int order)
{
    unsigned int nbPoints = points.getCols();
    if(nbPoints < 2) throw vpException(vpException::dimensionError, "usPolynomialCurve3D::defineFromWeightedPointsAuto(const vpMatrix &points, const vpColVector &weights, const vpColVector &direction, unsigned int order): need at least two points to fit");
    if(weights.size() != nbPoints) throw vpException(vpException::dimensionError, "usPolynomialCurve3D::defineFromWeightedPointsAuto(const vpMatrix &points, const vpColVector &weights, const vpColVector &direction, unsigned int order): mismatching number of points and weights");
    
    if(order<1) order = m_order;

    std::vector<double> t(nbPoints, 0);
    
    double min = std::numeric_limits<double>::max();
    for(unsigned int i=0 ; i<nbPoints; i++)
    {
        t.at(i) = vpColVector::dotProd(points.getCol(i), direction);
        if(t.at(i) < min) min = t.at(i);
    }
    for(unsigned int i=0 ; i<nbPoints; i++) t.at(i) -= min;
    
    std::vector<int> index(nbPoints);
    for(unsigned int i=0 ; i<nbPoints; i++) index.at(i) = i;
    
    std::sort(index.begin(), index.end(), [&t](int i, int j){return t[i] < t[j];});
    
    vpMatrix newPoints(3, nbPoints);
    vpColVector newWeights(nbPoints);
    vpColVector param(nbPoints);
    for(unsigned int i=0 ; i<nbPoints; i++)
    {
        for(unsigned int j=0 ; j<3; j++) newPoints[j][i] = points[j][index[i]];
        param[i] = t.at(index[i]);
        newWeights[i] = weights[index[i]];
    }

    return this->defineFromWeightedPoints(newPoints, param, newWeights, order);
}

double usPolynomialCurve3D::getCurvature(double param) const
{
    vpColVector dX_dl = this->getDerivative(param,1);
    vpColVector dX2_dl2 = this->getDerivative(param,2);

    double norm = dX_dl.euclideanNorm();
    double curvature = vpColVector::crossProd(dX_dl, dX2_dl2).euclideanNorm()/pow(norm,3);

    return curvature;
}

double usPolynomialCurve3D::getMeanAxisDeviation(int nbCountSeg) const
{
    if(nbCountSeg<2) throw vpException(vpException::badValue, "usPolynomialCurve3D::getMeanAxisDeviation: should use at least 2 segment to compute approximate deviation from axis");
    
    vpColVector params(nbCountSeg+1);
    double step = this->getParametricLength() / nbCountSeg;
    params[0] = m_startParameter;
    for(int i=1 ; i<nbCountSeg+1 ; i++) params[i] = params[i-1] + step;
    
    vpMatrix points(this->getPoints(params));
    
    vpColVector axis = (points.getCol(nbCountSeg) - points.getCol(0)).normalize();
    vpColVector origin = points.getCol(0);
    double meanDeviation = 0;
    for(int i=0 ; i<nbCountSeg+1; i++) meanDeviation += vpColVector::dotProd(points.getCol(i) - origin, axis);
    
    meanDeviation /= nbCountSeg+1;
    return meanDeviation;
}

void usPolynomialCurve3D::setControlPoints(const vpMatrix &controlPoints)
{
    vpColVector direction = controlPoints.getCol(controlPoints.getCols()-1) - controlPoints.getCol(0);
    this->defineFromPointsAuto(controlPoints, direction, controlPoints.getCols());
}

void usPolynomialCurve3D::setControlPoints(double **controlPoints)
{
    vpMatrix M(3, m_order);
    memcpy(M.data, *controlPoints, M.size()*sizeof(double));
    this->setControlPoints(M);
}

vpMatrix usPolynomialCurve3D::getControlPoints() const
{
    if(m_order==0) return this->getStartPoint();
    
    vpColVector params(m_order+1);
    double step = this->getParametricLength() / m_order;
    params[0] = m_startParameter;
    for(unsigned int i=1 ; i<m_order+1 ; i++) params[i] = params[i-1] + step;
    
    return this->getPoints(params);
}

vpMatrix usPolynomialCurve3D::getRenderingPoints() const
{
    int nbRenderingPoints = (m_order<2)?2:10;
    vpColVector params(nbRenderingPoints);
    double step = this->getParametricLength() / (nbRenderingPoints-1);
    params[0] = m_startParameter;
    for(int i=1 ; i<nbRenderingPoints ; i++) params[i] = params[i-1] + step;
    
    return this->getPoints(params);
}

double usPolynomialCurve3D::curveDistance(const usPolynomialCurve3D &n1, const usPolynomialCurve3D &n2)
{
  unsigned int order1 = n1.getOrder();
  unsigned int order2 = n2.getOrder();
  vpMatrix coords1(order1, 50);
  vpMatrix coords2(order2, 50);
  for (unsigned int j = 0; j < 50; ++j) {
    coords1[0][j] = 1.0;
    coords2[0][j] = 1.0;
    double t = static_cast<double>(j) / 49.0;
    for (unsigned int i = 1; i < order1; ++i)
      coords1[i][j] = coords1[i - 1][j] * t;
    for (unsigned int i = 1; i < order2; ++i)
      coords2[i][j] = coords2[i - 1][j] * t;
  }
  vpMatrix p1 = n1.getPolynomialCoefficients() * coords1;
  vpMatrix p2 = n2.getPolynomialCoefficients() * coords2;
  double distance = 0.0;
  for (unsigned int i = 0; i < 50; ++i)
    distance += (p1.getCol(i) - p2.getCol(i)).euclideanNorm();
  distance /= 50;
  return distance;
}

usPolynomialCurve3D usPolynomialCurve3D::getSubPolynomialCurve(double startParameter, double endParameter) const
{
    usPolynomialCurve3D seg(*this);

    seg.setBoundaries(startParameter, endParameter);

    return seg;
}

usPolynomialCurve3D usPolynomialCurve3D::getNewOrderPolynomialCurve(unsigned int order) const
{
    usPolynomialCurve3D seg(*this);

    seg.setOrder(order);

    return seg;
}

void usPolynomialCurve3D::changeCoefficientsToFitBoundaries(double startParameter, double endParameter)
{
    if(startParameter==m_startParameter && endParameter==m_endParameter) return;
    if(startParameter==endParameter) throw vpException(vpException::dimensionError, "usPolynomialCurve3D::changeCoefficientsToFitBoundaries(double startParameter, double endParameter): new parametric boundaries should be different");

    
    double beta = (m_startParameter*endParameter - m_endParameter*startParameter) / (endParameter-startParameter);
    double beta_pow[m_order+1];
    beta_pow[0] = 1;
    for(unsigned int i=1 ; i<=m_order ; i++) beta_pow[i] = beta*beta_pow[i-1];

    double alpha = (m_endParameter-m_startParameter) / (endParameter-startParameter);
    double alpha_pow[m_order+1];
    alpha_pow[0] = 1;
    for(unsigned int i=1 ; i<=m_order ; i++) alpha_pow[i] = alpha*alpha_pow[i-1];

    vpMatrix M(m_order+1,m_order+1,0);
    M[0][0] = 1;
    for(unsigned int i=1 ; i<=m_order ; i++)
    {
        M[i][0] = 1;
        for(unsigned int j=1 ; j<=i ; j++)
        {
            M[i][j] = M[i-1][j-1] + M[i-1][j];
        }
    }

    for(unsigned int i=1 ; i<=m_order ; i++)
    {
        for(unsigned int j=0 ; j<=i ; j++)
        {
            M[i][j] *=  alpha_pow[j] * beta_pow[i-j];
        }
    }

    m_polynomialCoefficients = m_polynomialCoefficients*M;

    m_startParameter = startParameter;
    m_endParameter = endParameter;
}

void usPolynomialCurve3D::reverse()
{
    this->changeCoefficientsToFitBoundaries(m_endParameter, m_startParameter);
}

void usPolynomialCurve3D::changeCoefficientsToFitMetricLength()
{
    double endParameter = m_endParameter;
    this->changeCoefficientsToFitBoundaries(m_startParameter, m_startParameter+this->getLength());
    this->setBoundaries(m_startParameter, endParameter);
}

void usPolynomialCurve3D::move(const vpHomogeneousMatrix &H)
{
    vpMatrix R(H.getRotationMatrix());
    vpMatrix M = R * m_polynomialCoefficients;
    for(int i=0 ; i<3 ; i++) M[i][0] += H[i][3];
    
    this->setPolynomialCoefficients(M);
}

void usPolynomialCurve3D::move(double x, double y, double z, double tx, double ty, double tz)
{
    this->move(vpHomogeneousMatrix(x,y,z,tx,ty,tz));
}

void usPolynomialCurve3D::scale(double s)
{
    m_polynomialCoefficients = s * m_polynomialCoefficients;
}