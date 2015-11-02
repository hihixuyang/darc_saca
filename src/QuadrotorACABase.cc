#include "QuadrotorACABase.h"
#include <stdio.h>

void QuadrotorACABase::set_desired_u(const Input& desired_u) {
	desired_u_ = desired_u;
}

void QuadrotorACABase::ResetDeltaU(void) {
	delta_u_ = Input::Zero();
}

void QuadrotorACABase::set_time_horizon(float time_horizon) {
	time_horizon_ = time_horizon;
}

QuadrotorBase::XXmat
QuadrotorACABase::MotionVarianceDerivative(const QuadrotorBase::XXmat& Mtau) {
	return A_*Mtau + Mtau*A_.transpose() + M_;
}  // MotionVarianceDerivative

void QuadrotorACABase::MotionVarianceIntegration(void) {
	XXmat M1 = MotionVarianceDerivative(Mtau_);
	XXmat M2 = MotionVarianceDerivative(Mtau_ + 0.5*dt_*M1);
	XXmat M3 = MotionVarianceDerivative(Mtau_ + 0.5*dt_*M2);
	XXmat M4 = MotionVarianceDerivative(Mtau_ + dt_*M3);
	Mtau_ =  Mtau_ + (dt_/6.0)*(M1 + 2.0*M2 + 2.0*M3 + M4);
}  // MotionVarianceIntegration

float QuadrotorACABase::VarianceProjection(const Eigen::MatrixXf& A,
																				   const Eigen::VectorXf& b) {
	float c = 3.841;  // TODO: update this to be a percent?
	Eigen::LLT<Eigen::MatrixXf> lltOfA(A);
	Eigen::MatrixXf L = lltOfA.matrixL();
	return c*b.transpose()*L*b;
}  // VarianceProjection

Eigen::VectorXf QuadrotorACABase::PositionUncertaintyProjection(
	const Eigen::VectorXf& normal) {
	return normal*VarianceProjection(Mtau_.block(0,0,3,3), normal);
}  // PositionUncertaintyProjection

Eigen::VectorXf QuadrotorACABase::SensingUncertaintyProjection(
	const Eigen::VectorXf& normal) {
	return normal*VarianceProjection(Z_, normal);;
}  // SensingUncertaintyProjection

float QuadrotorACABase::sigma(const Eigen::VectorXf& normal) {
	return VarianceProjection(Mtau_.block(0,0,3,3) + Z_, normal);
}  // sigma

void QuadrotorACABase::CreateHalfplane(const Eigen::VectorXf pos_colliding,
																		   const Eigen::VectorXf normal) {
	halfplane tmp_plane;
	tmp_plane.pos_colliding_ = pos_colliding;
	printf("p_col: %0.2f, %0.2f\n", pos_colliding[0], pos_colliding[1]);
	tmp_plane.normal_ = normal;
	printf("n: %0.2f, %0.2f\n", normal[0], normal[1]);
	halfplanes_.push_back(tmp_plane);
	printf("num planes: %d\n", static_cast<int>(halfplanes_.size()));
	int k;
	std::cin >> k;
}  // CreateHalfplane

void QuadrotorACABase::ClearHalfplanes(void) {
	halfplanes_.clear();
}  // ClearHalfplanes

