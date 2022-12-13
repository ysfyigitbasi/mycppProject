#include "Arduino.h"
#include "SimpleKalmanFilter.h"
#include <math.h>

SimpleKalmanFilter::SimpleKalmanFilter(float mea_e, float est_e, float q)
{
  _err_measure=mea_e;
  _err_estimate=est_e;
  _q = q;
}

float SimpleKalmanFilter::updateEstimateFloat(float mea)
{
  _kalman_gain = _err_estimate/(_err_estimate + _err_measure);
  _current_estimateF = _last_estimateF + _kalman_gain * (mea - _last_estimateF);
  _err_estimate =  (1.0 - _kalman_gain)*_err_estimate + fabs(_last_estimateF-_current_estimateF)*_q;
  _last_estimateF=_current_estimateF;

  return _current_estimateF;
}
long SimpleKalmanFilter::updateEstimateLong(long mea){

  _kalman_gain = _err_estimate/(_err_estimate + _err_measure);
  _current_estimateL = _last_estimateL + _kalman_gain * (mea - _last_estimateL);
  _err_estimate =  (1.0 - _kalman_gain)*_err_estimate + fabs(_last_estimateL-_current_estimateL)*_q;
  _last_estimateL=_current_estimateL;

  return _current_estimateL;
}

void SimpleKalmanFilter::setMeasurementError(float mea_e)
{
  _err_measure=mea_e;
}

void SimpleKalmanFilter::setEstimateError(float est_e)
{
  _err_estimate=est_e;
}

void SimpleKalmanFilter::setProcessNoise(float q)
{
  _q=q;
}

float SimpleKalmanFilter::getKalmanGain() {
  return _kalman_gain;
}

float SimpleKalmanFilter::getEstimateError() {
  return _err_estimate;
}