/** @file PIDController.cpp
 * @brief This tests the PIDController to ensure that it works within set standards
 *
 * @author Samantha Johnson
 * @date September 24, 2017
 * @copyright GNU Public License
 *
 */

#include "PIDController.h"
#include <gtest/gtest.h>
#include <vector>

using std::vector;

TEST(PIDController_test, testComputeControlOutputSettlesToSetPointFromStartBelow) {

  double setPoint = 50;
  double initialPoint = 10;
  int maxIterations = 100;

  PIDController PID(setPoint, initialPoint);
  vector<double> outputPID = PID.compute_control(maxIterations);

  EXPECT_NEAR(setPoint, outputPID.back(),0.5);
}

TEST(PIDController_test, testComputeControlOutputSettlesToSetPointFromStartAbove) {

  double setPoint = 50;
  double initialPoint = 70;
  int maxIterations = 100;

  PIDController PID(setPoint, initialPoint);
  vector<double> outputPID = PID.compute_control(maxIterations);

  EXPECT_NEAR(setPoint, outputPID.back(),0.5);
}

TEST(PIDController_test, testComputeControlOutputNoHighOvershoot) {

  double setPoint = 50;
  double initialPoint = 25;
  int maxIterations = 100;

  PIDController PID(setPoint, initialPoint);
  vector<double> outputPID = PID.compute_control(maxIterations);

  double maxCompare = 1;
  for (auto output : outputPID)
  {
    if (output > maxCompare)
    {
      maxCompare = output;
    }
  }

  EXPECT_NEAR(setPoint, maxCompare , 5);
}
