#include "MinkowskiSum2d.h"
#include <iostream>

MinkowskiSum2d::MinkowskiSum2d(void) {
	;
}  // MinkowskiSum2d

MinkowskiSum2d::MinkowskiSum2d(const std::vector<Eigen::Vector2f>& points_in,
															 float radius) {
	original_points_ = points_in;
	radius_ = radius;
}  // MinkowskiSum2d

std::vector<Eigen::Vector2f> MinkowskiSum2d::CalculateMinkowskiSum(void) {
	if (original_points_.size() > 0) {
		std::vector<Eigen::Vector2f> normals;
		for (int i = 1; i < original_points_.size(); ++i) {
			Eigen::Vector2f normal;
			normal << -(original_points_[i-1][1] - original_points_[i][1]),
				original_points_[i-1][0] - original_points_[i][0];
			normal.normalize();
			normals.push_back(normal);
		}

		Eigen::Vector2f minkowski_point;
		float theta = acos(normals.back().transpose()*normals.front());
		float a = tan(0.5*theta);
		Eigen::Vector2f new_normal;
		new_normal[0] = normals.back()[0] + a*normals.back()[1];
		new_normal[1] = normals.back()[1] - a*normals.back()[0];
		minkowski_point = original_points_.front() + radius_*new_normal;
		minkowski_points_.push_back(minkowski_point);
		/*
		std::cout << "theta: " << theta << std::endl;
		std::cout << "a: " << a << std::endl;
		std::cout << "new norm: " << new_normal.transpose() << std::endl;
		int k;
		std::cin >> k;
		*/
		for (int i = 1; i < original_points_.size()-1; ++i) {
			theta = acos(normals[i-1].transpose()*normals[i]);
			a = tan(0.5*theta);
			new_normal[0] = normals[i][0] - a*normals[i][1];
			new_normal[1] = normals[i][1] + a*normals[i][0];
			minkowski_point = original_points_[i] + radius_*new_normal;
			minkowski_points_.push_back(minkowski_point);
			/*
			std::cout << "Theta: " << theta << std::endl;
			std::cout << "a: " << std::endl;
			std::cout << "new_normal: " << new_normal.transpose() << std::endl;
			std::cin >> k;
			*/
		}
		minkowski_points_.push_back(minkowski_points_.front());
	}
	return minkowski_points_;
}  // CalculateMinkowskiSum


