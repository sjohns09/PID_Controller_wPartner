/** @file PIDController.cpp
 * @brief This class performs PID control on a set of given parameters.
 *
 * @author Samantha Johnson
 * @author Michael Kam
 * @date September 24, 2017
 * @copyright [2017] <Samantha Johnson>
 *
 * @details This class allows for the parameters for a PID controller to be manually tuned and then calculates the control
 * of that system until a set stopping point is reached.
 */

#include "PIDController.h"
#include <vector>

using std::vector;


PIDController::PIDController(double setPoint, double initialPoint) {
  _Kp = 0.2;        // Proportional Gain
  _Ki = 0.4;        // Integral Gain
  _Kd = 0.01;        // Derivative Gain
  _timeStep = 1;  // dt
  _setPoint = setPoint;
  _error = _setPoint - initialPoint;
  _integralSum = 0;

}

vector<double> PIDController::compute_control(int maxIterations) {
  vector<double> outputPID(maxIterations,0);

  double error = _error;
  for (auto& elements : outputPID)
  {
    elements = compute_proportional_control(_Kp, error)
        + compute_integral_control(_Ki, error, _timeStep, _integralSum)
        + compute_derivative_control(_Kd, _timeStep, error);
    error = _setPoint - elements;
  }
  return outputPID;
}

double PIDController::compute_proportional_control(const double _Kp, const double _error) {
  double pControl = 0;

  pControl = _Kp * _error;

  return pControl;
}

double PIDController::compute_integral_control(const double _Ki, const double _error,
                                               const double _timeStep, double& _integralSum) {
  double iControl = 0;

  _integralSum = _integralSum + _error*_timeStep;
  iControl = _Ki * _integralSum;

  return iControl;
}

double PIDController::compute_derivative_control(const double _Kd, const double _timeStep,
                                                 const double _derror) {
  double dControl = 0;

  dControl = _Kd * (_derror / _timeStep);

  return dControl;
}



