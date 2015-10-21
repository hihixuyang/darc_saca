#include "uncertainty_sampling.h"

#include <cmath>

float UniformRandom(void) {
	return static_cast<float>(rand() / static_cast<float>(RAND_MAX));
}

float Normal(void) {
	float u, v, s = 0;
	while (s == 0 || s >= 1) {
		u = 2.0*UniformRandom() - 1.0;
		v = 2.0*UniformRandom() - 1.0;
		s = u*u + v*v;
	}
	return u*sqrt(-2.0*log(s)/s);
}

Eigen::VectorXf SampleGaussian(const Eigen::VectorXf& mean,
															 const Eigen::MatrixXf& variance) {
	int size = mean.rows();
	Eigen::VectorXf sample(size);
	for (int j = 0; j < size; ++j) {
		sample[j] = Normal();
	}

	Eigen::JacobiSVD<Eigen::MatrixXf>
	svd(variance, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::MatrixXf D = Eigen::MatrixXf::Zero(size, size);
	for (int j = 0; j < size; ++j) {
		D(j,j) = sqrt((svd.singularValues())[j]);
	}
	return svd.matrixU()*D*sample + mean;
}
