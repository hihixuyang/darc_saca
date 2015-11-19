#include "MinkowskiSum2d.h"

MinkowskiSum2d::MinkowskiSum2d(void) {
	;
}  // MinkowskiSum2d

MinkowskiSum2d::MinkowskiSum2d(const std::vector<Eigen::Vector2f>& points_in,
															 float radius) {
	original_points_ = points_in;
	radius_ = radius;
}  // MinkowskiSum2d

std::vector<Eigen::Vector2f> MinkowskiSum2d::CalculateMinkowskiSum(void) {
	return minkowski_points_;
}  // CalculateMinkowskiSum


