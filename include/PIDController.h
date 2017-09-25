/** @file PIDController.h
 * @brief This class performs PID control on a set of given parameters.
 *
 * @author Samantha Johnson
 * @author Michael Kam
 * @date September 24, 2017
 * @copyright GNU Public License
 *
 * @details This class allows for the parameters for a PID controller to be tuned and then calculates the control
 * of that system until a set stopping point is reached.
 */

#include <vector>

#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

class PIDController {
 public:

  /**
   * @brief A constructor that sets up the PID parameters for a system.
   * @param double setPoint is the target control point, double initialPoint is the starting point of the system,
   */
  PIDController(double setPoint, double initialPoint);

  /**
   * @brief A method that computes the PID control of a given system until a set stopping point is reached
   * @param int maxIterations is the number of iterations for which the PID controller will operate
   * @return vector<double> which contains each of the points calculated during the PID control
   * @details This method will utilize the function KpControl+KiControl+KdControl, and will add each output to the return
   * vector until maxIterations is reached. This function will also calculate and keep track of the changes and differences in error
   * and the sum of the integral which needs to be passed to the PID functions
   */
  std::vector<double> compute_control(const int maxIterations);


 private:
  double _Kp, _Ki, _Kd, _timeStep, _error, _setPoint, _integralSum;
  /**
   * @brief A method that computes the proportional control for the PID controller
   * @param _Kp is the proportional gain and _error is the error between the set point and actual point
   * @return double which is the proportional component of the PID
   * @details This method will utilize the function Kp*e
   */
  double compute_proportional_control(const double _Kp, const double _error);

  /**
   * @brief A method that computes the integral control for the PID controller
   * @param _Ki is the integral gain, _error is the error between the set point and actual point, _timeStep is the change in time,
   * _integralSum is the sum of the previous integral calculations
   * @return double which is the integral component of the PID
   * @details This method will utilize the function Ki*(integralSum + error*dt)
   */
  double compute_integral_control(const double _Ki, const double _error, const double _timeStep, double& _integralSum);

  /**
   * @brief A method that computes the derivative control for the PID controller
   * @param _Kd is the derivative gain, _timeStep is the change in time, _derror is the change in error from the last calculation
   * @return double which is the derivative component of the PID
   * @details This method will utilize the function Kd*(derror/dt)
   */
  double compute_derivative_control(const double _Kd, const double _timeStep, const double _derror);
};

#endif /* PIDCONTROLLER_H_ */
