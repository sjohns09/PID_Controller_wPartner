/** @file main.cpp
 * @brief Performs PID control on a set of given parameters.
 *
 * @author Samantha Johnson
 * @author Michael Kam
 * @date September 24, 2017
 * @copyright [2017] <Samantha Johnson>
 */

#include <iostream>
#include <vector>
#include "PIDController.h"

using std::vector;
using std::cout;
using std::endl;

int main() {
  double setPoint = 30;
  double initialPoint = 10;
  int maxIterations = 50;

  /**
   * @brief A constructor that sets up the PID parameters for a system.
   * @param double setPoint is the target control point, double initialPoint is the starting point of the system,
   * @details Each of these parameters can be set above
   */
  PIDController controllerPID(setPoint, initialPoint);

  /**
   * @brief A method that computes the PID control of a given system until maxIterations is reached
   * @param int maxIterations is the number of iterations for which the PID controller will operate
   * @return vector<double> which contains each of the points calculated during the PID control
   */
  vector<double> PID = controllerPID.compute_control(maxIterations);

  // Displays the controlled velocity outputs
  for (auto element : PID) {
    cout << element << endl;
  }
  return 0;
}
