#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

double dbz_guard(double x)
{
  if (x == 0.0)
      return 0.000000001;
  return x;
}

double normalize(double rads)
{
  return atan2(sin(rads), cos(rads));
}

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  auto sum = VectorXd(4);
  sum << 0, 0, 0, 0;

  for (int i = 0; i < estimations.size(); i++) {
    VectorXd diff = estimations[i] - ground_truth[i];
    diff = diff.array() * diff.array();
    sum += diff;
  }

  return (sum.array() / estimations.size()).sqrt();
}