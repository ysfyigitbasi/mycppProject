#ifndef SimpleKalmanFilter_h
#define SimpleKalmanFilter_h

class SimpleKalmanFilter
{
  float _err_measure;  // Measurement Uncertainity. How much do we expect to our measurement vary.

// Estimation Uncertainty. Can be initilized with the same value as e_mea since the kalman filter will adjust its value.
  float _err_estimate;
  float _q;  // Process Variance. Usually a small number, 0.001 - 1. How fast your measurement moves.
  // Recommended 0.01 . Should be tuned to your needs.
  float _current_estimateF = 0.0f;
  float _last_estimateF = 0.0f;
  long _current_estimateL = 0L;
  long _last_estimateL = 0L;
  float _kalman_gain = 0.0f;

public:
  SimpleKalmanFilter(float mea_e, float est_e, float q);
  float updateEstimateFloat(float mea);
  long updateEstimateLong(long mea);
  void setMeasurementError(float mea_e);
  void setEstimateError(float est_e);
  void setProcessNoise(float q);
  float getKalmanGain();
  float getEstimateError();

};

#endif