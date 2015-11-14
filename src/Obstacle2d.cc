#include "Obstacle2d.h"

Obstacle2d::Obstacle2d(void) {
	true_intersection_calculated_ = translated_intersection_calculated_ =
		noise_intersection_calculated_ = false;
}  // Obstacle2d

Obstacle2d::Obstacle2d(const Eigen::Vector2f& v0, const Eigen::Vector2f& v1,
											 const Eigen::Vector2f& normal, float radius) {
	true_intersection_calculated_ = translated_intersection_calculated_ =
		noise_intersection_calculated_ = false;

	true_vertices_.push_back(v0);
	true_vertices_.push_back(v1);
	normal_ = normal.normalized();

	// Naive Minkowski sum
	Eigen::Vector2f v0_tilde = v0 + radius*normal_;
	Eigen::Vector2f v1_tilde = v1 + radius*normal_;

	translated_vertices_.push_back(v0_tilde);
	translated_vertices_.push_back(v1_tilde);

	noise_vertices_.push_back(v0_tilde);
	noise_vertices_.push_back(v1_tilde);
}  // Obstacle2d

Eigen::Vector2f Obstacle2d::true_vertices(int index) {
	return true_vertices_[index];
}  // true_vertices

Eigen::Vector2f Obstacle2d::translated_vertices(int index) {
	return translated_vertices_[index];
}  // translated_vertices

Eigen::Vector2f Obstacle2d::noise_vertices(int index) {
	return noise_vertices_[index];
}  // noise_vertices

Eigen::Vector2f Obstacle2d::normal(void) {
	return normal_;
}  // normal

bool Obstacle2d::IsTrueIntersecting(const Eigen::Vector2f& current_position,
																	 const Eigen::Vector2f& desired_position) {
	SolveForTrueIntersection(current_position, desired_position);
	if ((true_c_ != 0) && (true_a_ == true_b_ && true_a_ != 0)) {
		return (0 <= true_a_/true_c_ <= 1) && (0 <= true_b_/true_c_ <= 1);
	}
	return false;
}  // IsTrueItersecting

bool Obstacle2d::IsTranslatedIntersecting(const Eigen::Vector2f& current_position,
																					const Eigen::Vector2f& desired_position) {
	SolveForTranslatedIntersection(current_position, desired_position);
	if ((translated_c_ != 0) &&
			(translated_a_ == translated_b_ && translated_a_ != 0)) {
		return (0 <= translated_a_/translated_c_ <= 1) &&
			(0 <= translated_b_/translated_c_ <= 1);
	}
	return false;
}  // IsTranslatedIntersecting

bool Obstacle2d::IsNoiseIntersecting(const Eigen::Vector2f& current_position,
																		 const Eigen::Vector2f& desired_position) {
	SolveForNoiseIntersection(current_position, desired_position);
	if ((noise_c_ != 0) && (noise_a_ == noise_b_ && noise_a_ != 0)) {
		return (0 <= noise_a_/noise_c_ <= 1) && (0 <= noise_b_/noise_c_ <= 1);
	}
	return false;
}  // IsNoiseIntersecting

Eigen::Vector2f Obstacle2d::TrueIntersectionPoint(const Eigen::Vector2f& current_position,
																									const Eigen::Vector2f& desired_position) {
	if (!true_intersection_calculated_)
		SolveForTrueIntersection(current_position, desired_position);
	return current_position*(1.0 - true_a_/true_c_) +
		desired_position*(true_a_/true_c_);
}  // TrueIntersectionPoint

Eigen::Vector2f Obstacle2d::TranslatedIntersectionPoint(const Eigen::Vector2f& current_position,
																												const Eigen::Vector2f& desired_position) {
	if (!translated_intersection_calculated_)
		SolveForTranslatedIntersection(current_position, desired_position);
	return current_position*(1.0 - translated_a_/translated_c_) +
	desired_position*(translated_a_/translated_c_);
}  // TranslatedIntersectionPoint

Eigen::Vector2f Obstacle2d::NoiseIntersectionPoint(const Eigen::Vector2f& current_position,
																									 const Eigen::Vector2f& desired_position) {
	if (!noise_intersection_calculated_)
		SolveForNoiseIntersection(current_position, desired_position);
	return current_position*(1.0 - noise_a_/noise_b_) +
		desired_position*(noise_a_/noise_b_);
}  // NoiseIntersectionPoint

bool Obstacle2d::IsTrueSeeable(const Eigen::Vector2f& position) {
	return normal_.transpose()*(position - true_vertices_[0]) > 0.0;
}  // IsTrueSeeable

bool Obstacle2d::IsTranslatedSeeable(const Eigen::Vector2f& position) {
	return normal_.transpose()*(position - translated_vertices_[0]) > 0.0;
}  // IsTranslatedSeeable

void Obstacle2d::SolveForTrueIntersection(const Eigen::Vector2f& current_position,
																					const Eigen::Vector2f& desired_position) {
	float x1 = current_position[0]; float y1 = current_position[1];
	float x2 = desired_position[0]; float y2 = desired_position[1];
	float x3 = true_vertices_[0][0]; float y3 = true_vertices_[0][1];
	float x4 = true_vertices_[1][0]; float y4 = true_vertices_[1][1];
	true_a_ = (x4 - x3) * (y1 - y3) - (x4 - x3) * (x1 - x3);
	true_b_ = (x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3);
	true_c_ = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);

	float eps = 0.00001;
	if (true_a_ <=  eps && true_a_ >= -eps)
		true_a_ = 0.0f;
	if (true_b_ <= eps && true_b_ >= -eps)
		true_b_ = 0.0f;
	if (true_c_ <= eps && true_c_ >= -eps)
		true_c_ = 0.0f;
	
	true_intersection_calculated_ = true;
}  // SolveForTrueIntersection

void Obstacle2d::SolveForTranslatedIntersection(const Eigen::Vector2f& current_position,
																								const Eigen::Vector2f& desired_position) {
	float x1 = current_position[0]; float y1 = current_position[1];
	float x2 = desired_position[0]; float y2 = desired_position[1];
	float x3 = translated_vertices_[0][0]; float y3 = translated_vertices_[0][1];
	float x4 = translated_vertices_[1][0]; float y4 = translated_vertices_[1][1];
	translated_a_ = (x4 - x3) * (y1 - y3) - (x4 - x3) * (x1 - x3);
	translated_b_ = (x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3);
	translated_c_ = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);

	float eps = 0.00001;
	if (translated_a_ <=  eps && translated_a_ >= -eps)
		translated_a_ = 0.0f;
	if (translated_b_ <= eps && translated_b_ >= -eps)
		translated_b_ = 0.0f;
	if (translated_c_ <= eps && translated_c_ >= -eps)
		translated_c_ = 0.0f;
	
	translated_intersection_calculated_ = true;
}  // SolveForTranslatedIntersection

void Obstacle2d::SolveForNoiseIntersection(const Eigen::Vector2f& current_position,
																					 const Eigen::Vector2f& desired_position) {
	float x1 = current_position[0]; float y1 = current_position[1];
	float x2 = desired_position[0]; float y2 = desired_position[1];
	float x3 = noise_vertices_[0][0]; float y3 = noise_vertices_[0][1];
	float x4 = noise_vertices_[1][0]; float y4 = noise_vertices_[1][1];
	noise_a_ = (x4 - x3) * (y1 - y3) - (x4 - x3) * (x1 - x3);
	noise_b_ = (x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3);
	noise_c_ = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);

	float eps = 0.00001;
	if (noise_a_ <=  eps && noise_a_ >= -eps)
		noise_a_ = 0.0f;
	if (noise_b_ <= eps && noise_b_ >= -eps)
		noise_b_ = 0.0f;
	if (noise_c_ <= eps && noise_c_ >= -eps)
		noise_c_ = 0.0f;
	
	noise_intersection_calculated_ = true;
}  // SolveForNoiseIntersection
