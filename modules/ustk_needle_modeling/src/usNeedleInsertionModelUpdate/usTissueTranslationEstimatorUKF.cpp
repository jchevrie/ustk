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
 * Author:
 * Jason Chevrie
 *
 *****************************************************************************/

#include <visp3/ustk_needle_modeling/usTissueTranslationEstimatorUKF.h>

#include <visp3/ustk_core/usGeometryTools.h>


usTissueTranslationEstimatorUKF::usTissueTranslationEstimatorUKF(): 
    usUnscentedKalmanFilter(),
    m_var_measure_p(1e-3),
    m_var_measure_d(1e-3),
    m_var_measure_f(1e-3),
    m_var_measure_t(1e-6),
    m_var_process_p(1e-3),
    m_var_process_v(0),
    m_var_process_a(1e-3),
    m_var_process_f(1e-2),
    m_var_process_phi(1e-1),
    m_stateDynamicsType(usTissueTranslationEstimatorUKF::CONSTANT_POSITION),
    m_tissueTranslationType(usTissueTranslationEstimatorUKF::LATERAL_TRANSLATIONS_ONLY),
    m_measureType(usTissueTranslationEstimatorUKF::NEEDLE_BODY_POINTS),
    m_propagationTime(0),
    m_lastMeasureTime(0),
    m_needle()
{
    this->setStateDimension(3);
    m_computeProcessNoiseCovarianceMatrixAutomatically = true;
    m_computeMeasureNoiseCovarianceMatrixAutomatically = true;
}

usTissueTranslationEstimatorUKF::~usTissueTranslationEstimatorUKF()
{
    
}

double usTissueTranslationEstimatorUKF::getPositionMeasureNoiseVariance() const
{
    return m_var_measure_p;
}

void usTissueTranslationEstimatorUKF::setPositionMeasureNoiseVariance(double sigma)
{
    if(sigma >= 0) m_var_measure_p = sigma;
}

double usTissueTranslationEstimatorUKF::getTipDirectionMeasureNoiseVariance() const
{
    return m_var_measure_d;
}

void usTissueTranslationEstimatorUKF::setTipDirectionMeasureNoiseVariance(double sigma)
{
    if(sigma >= 0) m_var_measure_d = sigma;
}

double usTissueTranslationEstimatorUKF::getForceMeasureNoiseVariance() const
{
    return m_var_measure_f;
}

void usTissueTranslationEstimatorUKF::setForceMeasureNoiseVariance(double sigma)
{
    if(sigma >= 0) m_var_measure_f = sigma;
}

double usTissueTranslationEstimatorUKF::getTorqueMeasureNoiseVariance() const
{
    return m_var_measure_t;
}

void usTissueTranslationEstimatorUKF::setTorqueMeasureNoiseVariance(double sigma)
{
    if(sigma >= 0) m_var_measure_t = sigma;
}

double usTissueTranslationEstimatorUKF::getTissuePositionProcessNoiseVariance() const
{
    return m_var_process_p;
}

void usTissueTranslationEstimatorUKF::setTissuePositionProcessNoiseVariance(double sigma)
{
    if(sigma >= 0) m_var_process_p = sigma;
}

double usTissueTranslationEstimatorUKF::getTissueVelocityProcessNoiseVariance() const
{
    return m_var_process_v;
}

void usTissueTranslationEstimatorUKF::setTissueVelocityProcessNoiseVariance(double sigma)
{
    if(sigma >= 0) m_var_process_v = sigma;
}

double usTissueTranslationEstimatorUKF::getTissueSinusoidalAmplitudeProcessNoiseVariance() const
{
    return m_var_process_a;
}

void usTissueTranslationEstimatorUKF::setTissueSinusoidalAmplitudeProcessNoiseVariance(double sigma)
{
    if(sigma >= 0) m_var_process_a = sigma;
}

double usTissueTranslationEstimatorUKF::getTissueSinusoidalFrequencyProcessNoiseVariance() const
{
    return m_var_process_f;
}

void usTissueTranslationEstimatorUKF::setTissueSinusoidalFrequencyProcessNoiseVariance(double sigma)
{
    if(sigma >= 0) m_var_process_f = sigma;
}

double usTissueTranslationEstimatorUKF::getTissueSinusoidalPhaseProcessNoiseVariance() const
{
    return m_var_process_phi;
}

void usTissueTranslationEstimatorUKF::setTissueSinusoidalPhaseProcessNoiseVariance(double sigma)
{
    if(sigma >= 0) m_var_process_phi = sigma;
}

usTissueTranslationEstimatorUKF::StateDynamicsType usTissueTranslationEstimatorUKF::getStateDynamicsType() const
{
    return m_stateDynamicsType;
}

void usTissueTranslationEstimatorUKF::setStateDynamicsType(usTissueTranslationEstimatorUKF::StateDynamicsType type)
{
    m_stateDynamicsType = type;
    
    switch(type)
    {
        case usTissueTranslationEstimatorUKF::CONSTANT_POSITION:
        {
            this->setStateDimension(3);
            break;
        }
        case usTissueTranslationEstimatorUKF::CONSTANT_VELOCITY:
        {
            this->setStateDimension(6);
            break;
        }
        case usTissueTranslationEstimatorUKF::SINUSOIDAL_POSITION:
        {
            this->setStateDimension(8);
            break;
        }
    }
}

usTissueTranslationEstimatorUKF::TissueTranslationType usTissueTranslationEstimatorUKF::getTissueTranslationType() const
{
    return m_tissueTranslationType;
}

void usTissueTranslationEstimatorUKF::setTissueTranslationType(usTissueTranslationEstimatorUKF::TissueTranslationType type)
{
    m_tissueTranslationType = type;
}

usTissueTranslationEstimatorUKF::MeasureType usTissueTranslationEstimatorUKF::getMeasureType() const
{
    return m_measureType;
}

void usTissueTranslationEstimatorUKF::setMeasureType(usTissueTranslationEstimatorUKF::MeasureType type)
{
    m_measureType = type;
    
    if(type == usTissueTranslationEstimatorUKF::TIP_POSITION_AND_DIRECTION || type == usTissueTranslationEstimatorUKF::BASE_FORCE_TORQUE) this->setMeasureDimension(6);
}

void usTissueTranslationEstimatorUKF::setPropagationTime(double time)
{
    m_propagationTime = time;
}

double usTissueTranslationEstimatorUKF::getLastMeasureTime() const
{
    return m_lastMeasureTime;
}

void usTissueTranslationEstimatorUKF::setLastMeasureTime(double time)
{
    m_lastMeasureTime = time;
}

void usTissueTranslationEstimatorUKF::setCurrentNeedle(const usNeedleInsertionModelRayleighRitzSpline& needle)
{
    m_needle = needle;
    switch(m_stateDynamicsType)
    {
        case usTissueTranslationEstimatorUKF::CONSTANT_POSITION:
        case usTissueTranslationEstimatorUKF::CONSTANT_VELOCITY:
        {
            m_state.insert(0, vpColVector(m_needle.accessTissue().getPose(), 0,3));
            break;
        }
        case usTissueTranslationEstimatorUKF::SINUSOIDAL_POSITION:
        {
            break;
        }
    }
}

void usTissueTranslationEstimatorUKF::applyStateToNeedle(usNeedleInsertionModelRayleighRitzSpline& needle) const
{
    vpPoseVector p(needle.accessTissue().accessSurface().getPose());
    
    switch(m_stateDynamicsType)
    {
        case usTissueTranslationEstimatorUKF::CONSTANT_POSITION:
        case usTissueTranslationEstimatorUKF::CONSTANT_VELOCITY:
        {
            for(int i=0 ; i<3 ; i++) p[i] = m_state[i];
            break;
        }
        case usTissueTranslationEstimatorUKF::SINUSOIDAL_POSITION:
        {
            for(int i=0 ; i<3 ; i++)
            {
                p[i] = m_state[i] + m_state[i+3] * sin(2*M_PI * m_state[6] * (m_lastMeasureTime + m_propagationTime) + m_state[7]);
            }      
            break;
        }
    }
    
    needle.accessTissue().setPose(p);
}

bool usTissueTranslationEstimatorUKF::checkConsistency(const vpColVector &measure)
{
    if(!m_needle.IsNeedleInserted()) return false;
    if(m_stateDynamicsType==usTissueTranslationEstimatorUKF::CONSTANT_POSITION || m_stateDynamicsType==usTissueTranslationEstimatorUKF::CONSTANT_VELOCITY)
    {
        if((vpColVector(m_state, 0,3) - vpColVector(m_needle.accessTissue().getPose(), 0,3)).euclideanNorm() > std::numeric_limits<double>::epsilon()) std::cout << "usTissueTranslationEstimatorUKF::checkConsistency: the state does not correspond to the needle model, make sure you used the method 'setCurrentNeedle' to initialize the state" << std::endl;
    }
    
    if(m_measureType == usTissueTranslationEstimatorUKF::NEEDLE_BODY_POINTS)
    {
        if(measure.size()%3 != 0) throw vpException(vpException::notInitialized, "usTissueTranslationEstimatorUKF::checkConsistency: measure is set as needle body points, but the mesure dimension is not a multiple of 3");
        this->setMeasureDimension(measure.size());
        this->setMeasureNoiseDimension(measure.size());
    }
    
    return this->usUnscentedKalmanFilter::checkConsistency(measure);
}

void usTissueTranslationEstimatorUKF::computeProcessNoiseCovarianceMatrix()
{
    vpMatrix normalizedCovarianceMatrix;
    normalizedCovarianceMatrix.eye(3);
    
    if(m_tissueTranslationType == usTissueTranslationEstimatorUKF::LATERAL_TRANSLATIONS_ONLY)
    {
        vpColVector d_ins = m_needle.accessTissue().accessSurface().getDirection().normalize();
        normalizedCovarianceMatrix = normalizedCovarianceMatrix - d_ins*d_ins.t();
    }
    
    switch(m_stateDynamicsType)
    {
        case usTissueTranslationEstimatorUKF::CONSTANT_POSITION:
        {
            m_processNoiseCovarianceMatrix.resize(3,3);
            m_processNoiseCovarianceMatrix = m_var_process_p * normalizedCovarianceMatrix;
            break;
        }
        case usTissueTranslationEstimatorUKF::CONSTANT_VELOCITY:
        {
            m_processNoiseCovarianceMatrix.resize(6,6);
            m_processNoiseCovarianceMatrix.insert(m_var_process_p * normalizedCovarianceMatrix, 0,0);
            m_processNoiseCovarianceMatrix.insert(m_var_process_v * normalizedCovarianceMatrix, 3,3);
            break;
        }
        case usTissueTranslationEstimatorUKF::SINUSOIDAL_POSITION:
        {
            m_processNoiseCovarianceMatrix.resize(8,8);
            m_processNoiseCovarianceMatrix.insert(m_var_process_p * normalizedCovarianceMatrix, 0,0);
            vpMatrix M(3,3,0);
            for(int i=0 ; i<3 ; i++)
            {
                if(m_state[i+3] > sqrt(std::numeric_limits<double>::epsilon())) M[i][i] = 1. / m_state[i+3];
                else M[i][i] = 1. / sqrt(std::numeric_limits<double>::epsilon());
            }
            vpMatrix Cov = M * m_var_process_a * normalizedCovarianceMatrix * M.t();
            for(int i=0 ; i<3 ; i++)
              for(int j=0 ; j<3 ; j++)
                Cov[i][j] = log(1 + Cov[i][j]);
            m_processNoiseCovarianceMatrix.insert(Cov, 3,3);
            if(m_state[6] > sqrt(std::numeric_limits<double>::epsilon())) m_processNoiseCovarianceMatrix[6][6] = log(1 + m_var_process_f / vpMath::sqr(m_state[6]));
            else m_processNoiseCovarianceMatrix[6][6] = m_var_process_f / std::numeric_limits<double>::epsilon();            
            m_processNoiseCovarianceMatrix[7][7] = m_var_process_phi;
            break;
        }
    }
}

void usTissueTranslationEstimatorUKF::computeMeasureNoiseCovarianceMatrix()
{
    switch(m_measureType)
    {
        case usTissueTranslationEstimatorUKF::NEEDLE_BODY_POINTS:
        {
            m_measureNoiseCovarianceMatrix.resize(m_measureDimension,m_measureDimension);
            m_measureNoiseCovarianceMatrix.diag(m_var_measure_p);
            break;
        }
        case usTissueTranslationEstimatorUKF::TIP_POSITION_AND_DIRECTION:
        {
            m_measureNoiseCovarianceMatrix.resize(6,6);
            for(int i=0 ; i<3 ; i++)
            {
                m_measureNoiseCovarianceMatrix[i][i] = m_var_measure_p;
                m_measureNoiseCovarianceMatrix[i+3][i+3] = m_var_measure_d;
            }
            break;
        }
        case usTissueTranslationEstimatorUKF::BASE_FORCE_TORQUE:
        {
            m_measureNoiseCovarianceMatrix.resize(6,6);
            for(int i=0 ; i<3 ; i++)
            {
                m_measureNoiseCovarianceMatrix[i][i] = m_var_measure_f;
                m_measureNoiseCovarianceMatrix[i+3][i+3] = m_var_measure_t;
            }
            break;
        }
    }
}

vpColVector usTissueTranslationEstimatorUKF::propagateSigmaPoint(const vpColVector &sigmaPoint)
{
    switch(m_stateDynamicsType)
    {
        case usTissueTranslationEstimatorUKF::CONSTANT_POSITION:
        {
            return sigmaPoint;
        }
        case usTissueTranslationEstimatorUKF::CONSTANT_VELOCITY:
        {
            vpColVector propagatedSigmaPoint(sigmaPoint);
            for(int i=0 ; i<3 ; i++) propagatedSigmaPoint[i] += m_propagationTime * propagatedSigmaPoint[i+3];
            return propagatedSigmaPoint;
        }
        case usTissueTranslationEstimatorUKF::SINUSOIDAL_POSITION:
        {
            vpColVector propagatedSigmaPoint(sigmaPoint);
            //if(propagatedSigmaPoint[6] < 0) propagatedSigmaPoint[6] = 0;
            //for(int i=0 ; i<3 ; i++) if(propagatedSigmaPoint[i+3] < 0) propagatedSigmaPoint[i+3] = 0;
            return propagatedSigmaPoint;
        }
        default:
        {
            return sigmaPoint;
        }
    }
}

vpColVector usTissueTranslationEstimatorUKF::computeMeasureFromSigmaPoint(const vpColVector &sigmaPoint)
{
    usNeedleInsertionModelRayleighRitzSpline testNeedle(m_needle);
    
    vpPoseVector tissuePose(testNeedle.accessTissue().getPose());
    
    switch(m_stateDynamicsType)
    {
        case usTissueTranslationEstimatorUKF::CONSTANT_POSITION:
        case usTissueTranslationEstimatorUKF::CONSTANT_VELOCITY:
        {
            for(int i=0 ; i<3 ; i++) tissuePose[i] = sigmaPoint[i];
            break;
        }
        case usTissueTranslationEstimatorUKF::SINUSOIDAL_POSITION:
        {
            vpColVector state = this->stateExp(sigmaPoint, m_state);
            for(int i=0 ; i<3 ; i++)
            {
                tissuePose[i] = state[i] + state[i+3] * sin(2*M_PI * state[6] * (m_lastMeasureTime + m_propagationTime) + state[7]);
            }
            break;
        }
        default:
        {
            break;
        }
    }
    
    testNeedle.accessTissue().setPose(tissuePose);
    testNeedle.updateState();
    
    switch(m_measureType)
    {
        case usTissueTranslationEstimatorUKF::NEEDLE_BODY_POINTS:
        {
            int nbPoints = m_measure.size()/3;

            std::vector<int> index(nbPoints);
            std::vector<double> l(nbPoints);
            for(int i=0 ; i<nbPoints ; i++)
            {
                usGeometryTools::projectPointOnCurve(vpColVector(m_measure, 3*i,3), m_needle.accessNeedle(), -1, &(index.at(i)), &(l.at(i)));
            }

            vpColVector measure(m_measure.size());
            for(int i=0 ; i<nbPoints ; i++) measure.insert(3*i, testNeedle.accessNeedle().accessSegment(index.at(i)).getPoint(l.at(i)));            
            
            return measure;
        }
        case usTissueTranslationEstimatorUKF::TIP_POSITION_AND_DIRECTION:
        {
            vpColVector measure(6);
            measure.insert(0, testNeedle.accessNeedle().getTipPosition());
            measure.insert(3, testNeedle.accessNeedle().getTipDirection());

            return measure;
        }
        case usTissueTranslationEstimatorUKF::BASE_FORCE_TORQUE:
        {
            vpColVector measure(testNeedle.accessNeedle().getBaseStaticTorsor());
    
            return measure;
        }
        default:
        {
            throw vpException(vpException::notImplementedError, "usTissueTranslationEstimatorUKF::computeMeasureFromSigmaPoint: measure type not implemented");
        }
    }
}

double usTissueTranslationEstimatorUKF::stateNorm(const vpColVector &state) const
{
    switch(m_stateDynamicsType)
    {
        case usTissueTranslationEstimatorUKF::CONSTANT_POSITION:
        {
            return vpColVector(state, 0,3).euclideanNorm();
        }
        case usTissueTranslationEstimatorUKF::CONSTANT_VELOCITY:
        {
            return vpColVector(state, 0,3).euclideanNorm() + m_propagationTime * vpColVector(state, 3,3).euclideanNorm();
        }
        case usTissueTranslationEstimatorUKF::SINUSOIDAL_POSITION:
        {
            vpColVector realState = this->stateExp(state, m_state);
            return vpColVector(realState, 0,3).euclideanNorm() + vpColVector(realState, 3,3).euclideanNorm();
        }  
        default:
        {
            return state.euclideanNorm();
        }
    }
}

vpColVector usTissueTranslationEstimatorUKF::measureLog(const vpColVector& measure, const vpColVector &measureCenter) const
{
    switch(m_measureType)
    {
        case usTissueTranslationEstimatorUKF::NEEDLE_BODY_POINTS:
        {
            return measure;
        }
        case usTissueTranslationEstimatorUKF::TIP_POSITION_AND_DIRECTION:
        {
            vpColVector measureL(6);
            for(int i=0 ; i<3 ; i++) measureL[i] = measure[i];
    
            vpColVector d_init(measureCenter, 3,3);
            vpColVector d(measure, 3,3);
            double angle = fabs(atan2(vpColVector::crossProd(d_init, d).euclideanNorm(), vpColVector::dotProd(d_init,d)));
            measureL.insert(3, angle * vpColVector::crossProd(d_init, d).normalize());
            
            return measureL;
        }
        case usTissueTranslationEstimatorUKF::BASE_FORCE_TORQUE:
        {
            return measure;
        }
        default:
        {
            return measure;
        }
    }
}

vpColVector usTissueTranslationEstimatorUKF::stateLog(const vpColVector &state, const vpColVector &stateCenter) const
{
  if(m_stateDynamicsType==usTissueTranslationEstimatorUKF::SINUSOIDAL_POSITION) {
    vpColVector logState(8);
    for(int i=0 ; i<3 ; i++) logState[i] = state[i];
    for(int i=3 ; i<7 ; i++) {
      if(state[i] > 0) {
        if(stateCenter[i] > state[i]*std::numeric_limits<double>::epsilon()) 
          logState[i] = log(state[i] / stateCenter[i]);
        else
          logState[i] = log(1. / std::numeric_limits<double>::epsilon());
      }
      else {
        if(stateCenter[i] > 0)
          logState[i] = log(std::numeric_limits<double>::epsilon());
        else
          logState[i] = 0.;
      }
    }
    logState[7] = state[7] - stateCenter[7];
    return logState;
  }
  else return state;
}

vpColVector usTissueTranslationEstimatorUKF::stateExp(const vpColVector &state, const vpColVector &stateCenter) const
{
  if(m_stateDynamicsType==usTissueTranslationEstimatorUKF::SINUSOIDAL_POSITION) {
    vpColVector expState(8);
    for(int i=0 ; i<3 ; i++)
      expState[i] = state[i];
    for(int i=3 ; i<7 ; i++) {
      if(stateCenter[i] > std::numeric_limits<double>::epsilon())
        expState[i] = stateCenter[i] * exp(state[i]);
      else
        expState[i] = std::numeric_limits<double>::epsilon() * exp(state[i]);
    }
    for(int i=3 ; i<6 ; i++) {
      if(expState[i] > 0.02) expState[i] = 0.02;
    }
    if(expState[6] > 1) expState[6] = 1;
    expState[7] = stateCenter[7] + state[7];
    while(expState[7] > M_PI) expState[7] -= 2*M_PI;
    while(expState[7] <= -M_PI) expState[7] += 2*M_PI;
    return expState;
  }
  else return state;
}

vpMatrix usTissueTranslationEstimatorUKF::parallelTransport(const vpMatrix &covarianceMatrix, const vpColVector &previousState, const vpColVector &newState) const
{
  if(m_stateDynamicsType==usTissueTranslationEstimatorUKF::SINUSOIDAL_POSITION) {

    vpColVector conversionGains(8);
    
    for(int i=0 ; i<3 ; i++) conversionGains[i] = 1;
    
    for(int i=3 ; i<7 ; i++) 
    {
      if(newState[i] > previousState[i] * sqrt(std::numeric_limits<double>::epsilon()))
      {
        if(log(1 + (exp(covarianceMatrix[i][i]) - 1) * vpMath::sqr(previousState[i]) / vpMath::sqr(newState[i])) * std::numeric_limits<double>::epsilon() < covarianceMatrix[i][i])
          conversionGains[i] = log(1 + (exp(covarianceMatrix[i][i]) - 1) * vpMath::sqr(previousState[i]) / vpMath::sqr(newState[i])) / covarianceMatrix[i][i];
        else
          conversionGains[i] = 0;
      }
      else {
        if(log(1 + (exp(covarianceMatrix[i][i]) - 1) / std::numeric_limits<double>::epsilon()) * std::numeric_limits<double>::epsilon() < covarianceMatrix[i][i])
          conversionGains[i] = log(1 + (exp(covarianceMatrix[i][i]) - 1) / std::numeric_limits<double>::epsilon()) / covarianceMatrix[i][i];
        else
          conversionGains[i] = 0;
      }
    }

    conversionGains[7] = 1;
    
    vpMatrix conversionMatrix;
    conversionMatrix.diag(conversionGains);
    vpMatrix transportedCovMatrix(conversionMatrix*covarianceMatrix*conversionMatrix.t());
    return transportedCovMatrix;
  }
  else return covarianceMatrix;
}
