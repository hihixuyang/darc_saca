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
		std::vector<Eigen::Vector3f> normals(original_points_.size() - 1);
		for (int i = 1; i < original_points_.size(); ++i) {
			Eigen::Vector3f normal;
			normal << -(original_points_[i-1][1] - original_points_[i][1]),
				original_points_[i-1][0] - original_points_[i][0], 0;
			normal.normalize();
			normals[i-1] = normal;
		}
		
		// First check between first point and last point
		if (normals.back().cross(normals.front())[2] < 0.0) { // Find Intersection
			Eigen::Vector2f minkowski_point = FindIntersection(original_points_[original_points_.size()-2],
																												 original_points_[0],
																												 original_points_[1]);
			minkowski_points_.push_back(minkowski_point);
		} else {  // Add Circle
		std::vector<Eigen::Vector2f> minkowski_points =
				FindCircle(original_points_[original_points_.size()-2],
									 original_points_[0],
									 original_points_[1]);
			for (int i = 0; i < minkowski_points.size(); ++i) {
				minkowski_points_.push_back(minkowski_points[i]);
			}
		}
		for (int i = 1; i < original_points_.size()-1; ++i) {
			if (normals[i-1].cross(normals[i])[2] < 0.0) { // Find intersection
				Eigen::Vector2f minkowski_point = FindIntersection(original_points_[i-1],
																													original_points_[i],
																													original_points_[i+1]);
				minkowski_points_.push_back(minkowski_point);
			} else {  // Add circle
				std::vector<Eigen::Vector2f> minkowski_points =
					FindCircle(original_points_[i-1],
										 original_points_[i],
										 original_points_[i+1]);
				for (int i = 0; i < minkowski_points.size(); ++i) {
					minkowski_points_.push_back(minkowski_points[i]);
				}
			}
		}
		minkowski_points_.push_back(minkowski_points_.front());
	}
	return minkowski_points_;
}  // CalculateMinkowskiSum

Eigen::Vector2f MinkowskiSum2d::FindIntersection(const Eigen::Vector2f& A,
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

	return a + a_num/denom*(b-a);
}  // FindIntersection

std::vector<Eigen::Vector2f> MinkowskiSum2d::FindCircle(const Eigen::Vector2f& A,
																												const Eigen::Vector2f& B,
																												const Eigen::Vector2f& C) {
	Eigen::Vector2f normal_AB; normal_AB << -((A-B)/(A-B).norm())[1],
															 ((A-B)/(A-B).norm())[0];
	Eigen::Vector2f normal_BC; normal_BC << -((B-C)/(B-C).norm())[1],
															 ((B-C)/(B-C).norm())[0];
	
	Eigen::Vector2f a2 = radius_*normal_AB;
	Eigen::Vector2f b2 = radius_*normal_BC;

	Eigen::Vector3f a; a << a2[0], a2[1], 0.0;
	Eigen::Vector3f b; b << b2[0], b2[1], 0.0;

	std::vector<Eigen::Vector2f> points;
	points.reserve(1000);
	points.push_back(B + a.head(2));
	points.push_back(B + b.head(2));
	
	std::vector<Eigen::Vector2f>::iterator it = points.begin();
	int i = 1;
	while (i < points.size()) {
		//std::cout << "i: " << i << std::endl;
		a << points[i-1][0] - B[0], points[i-1][1] - B[1], 0.0;
		b << points[i][0] - B[0], points[i][1] - B[1], 0.0;
		float theta = atan2(((a.normalized()).cross(b.normalized())).norm(),
											(a.normalized()).dot(b.normalized()));
		//std::cout << "a: " << a.transpose() << std::endl;
		//std::cout << "b: " << b.transpose() << std::endl;
		//std::cout << "theta: " << theta << std::endl;
		if (theta > 30.0*M_PI/180.0) { // Split the line
			//std::cout << "Split" << std::endl;
			//int k;
			//std::cin >> k;
			Eigen::Vector2f c;
			c = radius_*(a + 0.5*(b-a)).normalized().head(2);
			//std::cout << "c: " << c.transpose() << std::endl;
			points.insert(it+i, B + c);
			//int k;
			//std::cin >> k;
		} else {
			i++;
		}
	}
	//std::cout << "Done" << std::endl;
	return points;
}  // FindCircle



