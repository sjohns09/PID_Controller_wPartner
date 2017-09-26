/** @file main.cpp
 * @brief Performs PID control on a set of given parameters.
 *
 * @author Samantha Johnson
 * @author Michael Kam
 * @date September 24, 2017
 * @copyright GNU Public License
 */

#include <iostream>
#include <lib.hpp>
#include <vector>
#include "PIDController.h"

using std::vector;
using std::cout;
using std::endl;

int main()
{

  double setPoint = 50;
  double initialPoint = 0;
  int maxIterations = 10;

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
