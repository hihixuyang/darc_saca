#ifndef UNCERTAINTY_SAMPLING_H
#define UNCERTAINTY_SAMPLING_H

#include <Eigen/Dense>

// Generate a uniform random number from 0 to 1
float UniformRandom(void);

// Find a normal distribution from the Box-Muller equations
float Normal(void);

// Find an X-dimension sample form a normal distribution
//   with a given mean and variance
Eigen::VectorXf SampleGaussian(const Eigen::VectorXf& mean,
															 const Eigen::MatrixXf& variance);

#endif
