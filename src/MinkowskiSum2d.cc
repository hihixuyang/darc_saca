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

		// First check between first point and last point
		//if (normals.back().transpose()*normals.front() < 0.0) { // Find Intersection
			Eigen::Vector2f minkowski_point = FindIntersection(original_points_.back(),
																												 original_points_[0],
																												 original_points_[1]);
			minkowski_points_.push_back(minkowski_point);
			/*
		} else {  // Add Circle
		std::vector<Eigen::Vector2f> minkowski_points =
				FindCircle(original_points_.back(),
									 original_points_[0],
									 original_points_[1]);
			for (int i = 0; i < minkowski_points.size(); ++i) {
				minkowski_points_.push_back(minkowski_points[i]);
			}
		}
			*/
		for (int i = 1; i < original_points_.size()-1; ++i) {
			//if (normals[i-1].transpose()*normals[i] < 0.0) { // Find intersection
				Eigen::Vector2f minkowski_point = FindIntersection(original_points_[i-1],
																													original_points_[i],
																													original_points_[i+1]);
				minkowski_points_.push_back(minkowski_point);
				/*
			} else {  // Add circle
				std::vector<Eigen::Vector2f> minkowski_points =
					FindCircle(original_points_[i-1],
										 original_points_[i],
										 original_points_[i+1]);
				for (int i = 0; i < minkowski_points.size(); ++i) {
					minkowski_points_.push_back(minkowski_points[i]);
				}
			}
				*/
		}
		minkowski_points_.push_back(minkowski_points_.front());
	}
	std::cout << original_points_.size() << std::endl;
	std::cout << minkowski_points_.size() << std::endl << std::endl;
	return minkowski_points_;
}  // CalculateMinkowskiSum

Eigen::Vector2f MinkowskiSum2d::FindIntersection(const Eigen::Vector2f& A,
																								 const Eigen::Vector2f& B,
																								 const Eigen::Vector2f& C) {
	radius_ = 1.0;
	Eigen::Vector2f normal_AB; normal_AB << -((A-B)/(A-B).norm())[1],
															 ((A-B)/(A-B).norm())[0];
	Eigen::Vector2f normal_BC; normal_BC << -((B-C)/(B-C).norm())[1],
															 ((B-C)/(B-C).norm())[0];

	Eigen::Vector2f a = A + radius_*normal_AB;
	Eigen::Vector2f b = B + radius_*normal_AB;
	Eigen::Vector2f c = B + radius_*normal_BC;
	Eigen::Vector2f d = C + radius_*normal_BC;

	float a_num = (d[0] - c[0]) * (a[1] - c[1]) - (d[1] - c[1]) * (a[0] - c[0]);
	float b_num = (b[0] - a[0]) * (a[1] - c[1]) - (b[1] - a[1]) * (a[0] - c[0]);
	float denom = (d[1] - c[1]) * (b[0] - a[0]) - (d[0] - c[0]) * (b[1] - a[1]);
/*
	std::cout << "A: " << A.transpose() << std::endl;
	std::cout << "B: " << B.transpose() << std::endl;
	std::cout << "C: " << C.transpose() << std::endl;
	std::cout << "n_ab: " << normal_AB.transpose() << std::endl;
	std::cout << "n_bc: " << normal_BC.transpose() << std::endl;
	std::cout << "a: " << a.transpose() << std::endl;
	std::cout << "b: " << b.transpose() << std::endl;
	std::cout << "c: " << c.transpose() << std::endl;
	std::cout << "d: " << d.transpose() << std::endl;
	std::cout << "a_num: " << a_num << std::endl;
	std::cout << "b_num: " << b_num << std::endl;
	std::cout << "denom: " << denom << std::endl;
	std::cout << "P: " << (a + a_num/denom*(b-a).normalized()).transpose() << std::endl;
	int k;
	std::cin >> k;
*/
	return a + a_num/denom*(b-a).normalized();
}  // FindIntersection

std::vector<Eigen::Vector2f> MinkowskiSum2d::FindCircle(const Eigen::Vector2f& A,
																												const Eigen::Vector2f& B,
																												const Eigen::Vector2f& C) {
	Eigen::Vector2f normal_AB; normal_AB << -((A-B)/(A-B).norm())[1],
															 ((A-B)/(A-B).norm())[0];
	Eigen::Vector2f normal_BC; normal_BC << -((B-C)/(B-C).norm())[1],
															 ((B-C)/(B-C).norm())[0];

	Eigen::Vector2f a = A + radius_*normal_AB;
	Eigen::Vector2f b = B + radius_*normal_AB;
	Eigen::Vector2f c = B + radius_*normal_BC;
	Eigen::Vector2f d = C + radius_*normal_BC;

	float a_num = (d[0] - c[0]) * (a[1] - c[1]) - (d[1] - c[1]) * (a[0] - c[0]);
	float b_num = (b[0] - a[0]) * (a[1] - c[1]) - (b[1] - a[1]) * (a[0] - c[0]);
	float denom = (d[1] - c[1]) * (b[0] - a[0]) - (d[0] - c[0]) * (b[1] - a[1]);

	std::vector<Eigen::Vector2f> points;
	points.push_back(a + a_num/denom*(b-a).normalized());

	return points;
}  // FindCircle



