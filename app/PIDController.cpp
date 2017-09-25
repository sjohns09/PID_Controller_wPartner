/** @file PIDController.cpp
 * @brief This class performs PID control on a set of given parameters.
 *
 * @author Samantha Johnson
 * @author Michael Kam
 * @date September 24, 2017
 * @copyright GNU Public License
 *
 * @details This class allows for the parameters for a PID controller to be manually tuned and then calculates the control
 * of that system until a set stopping point is reached.
 */

#include "PIDController.h"
#include <vector>

using std::vector;


PIDController::PIDController(double setPoint, double initialPoint) {
  _Kp = 0;        // Proportional Gain
  _Ki = 0;        // Integral Gain
  _Kd = 0;        // Derivative Gain
  _timeStep = 1;  // dt
  _setPoint = setPoint;
  _error = _setPoint - initialPoint;
  _integralSum = 0;

}

vector<double> PIDController::compute_control(int maxIterations) {
  vector<double> outputPID(maxIterations,0);

  // TODO: Do PID calculations by making calls to PID functions and loop until max iterations is reached

  return outputPID;
}

double PIDController::compute_proportional_control(const double _Kp, const double _error) {
  double pControl = 0;

  // TODO: Implement P controller

  return pControl;
}

double PIDController::compute_integral_control(const double _Ki, const double _error, const double _timeStep, double& _integralSum) {
  double iControl = 0;

  // TODO: Implement I controller

  return iControl;
}

double PIDController::compute_derivative_control(const double _Kd, const double _timeStep, const double _derror) {
  double dControl = 0;

  // TODO: Implement D controller

  return dControl;
}



